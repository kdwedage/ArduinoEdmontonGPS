/*****************************
* Names: Kevin Wedage, Aryan Singh
* IDs: 1532557, 1533732
* Comput 275, Winter 2019
* Assignment 2 Part 2
*****************************/

#include <Arduino.h>
#include <Adafruit_ILI9341.h>
#include <SD.h>
#include "consts_and_types.h"
#include "map_drawing.h"

typedef long long ll;

// the variables to be shared across the project, they are declared here!
shared_vars shared;

Adafruit_ILI9341 tft = Adafruit_ILI9341(clientpins::tft_cs, clientpins::tft_dc);

// initialize input buffer
char* inputBuffer = (char *) malloc(buf_size);
uint16_t inputBuf_len = 0;
int currentWaypoint = 0;

// Forward declartions
bool process_line();
ll * parseRequestStringToll(String& line);
ll stringToll(String& str);
void drawWayPoints();

void setup() {
  // initialize Arduino
  init();

  // initialize zoom pins
  pinMode(clientpins::zoom_in_pin, INPUT_PULLUP);
  pinMode(clientpins::zoom_out_pin, INPUT_PULLUP);

  // initialize joystick pins and calibrate centre reading
  pinMode(clientpins::joy_button_pin, INPUT_PULLUP);
  // x and y are reverse because of how our joystick is oriented
  shared.joy_centre = xy_pos(analogRead(clientpins::joy_y_pin), analogRead(clientpins::joy_x_pin));

  // initialize serial port
  Serial.begin(9600);
  Serial.flush(); // get rid of any leftover bits

  // initially no path is stored
  shared.num_waypoints = 0;

  // initialize display
  shared.tft = &tft;
  shared.tft->begin();
  shared.tft->setRotation(3);
  shared.tft->fillScreen(ILI9341_BLUE); // so we know the map redraws properly

  // initialize SD card
  if (!SD.begin(clientpins::sd_cs)) {
      Serial.println("Initialization has failed. Things to check:");
      Serial.println("* Is a card inserted properly?");
      Serial.println("* Is your wiring correct?");
      Serial.println("* Is the chipSelect pin the one for your shield or module?");

      while (1) {} // nothing to do here, fix the card issue and retry
  }

  // initialize the shared variables, from map_drawing.h
  // doesn't actually draw anything, just initializes values
  initialize_display_values();

  // initial draw of the map, from map_drawing.h
  draw_map();
  draw_cursor();

  // initial status message
  status_message("FROM?");
}

void process_input() {
  // read the zoom in and out buttons
  shared.zoom_in_pushed = (digitalRead(clientpins::zoom_in_pin) == LOW);
  shared.zoom_out_pushed = (digitalRead(clientpins::zoom_out_pin) == LOW);

  // read the joystick button
  shared.joy_button_pushed = (digitalRead(clientpins::joy_button_pin) == LOW);

  // joystick speed, higher is faster
  const int16_t step = 64;

  // get the joystick movement, dividing by step discretizes it
  // currently a far joystick push will move the cursor about 5 pixels
  xy_pos delta(
    // the funny x/y swap is because of our joystick orientation
    (analogRead(clientpins::joy_y_pin)-shared.joy_centre.x)/step,
    (analogRead(clientpins::joy_x_pin)-shared.joy_centre.y)/step
  );
  delta.x = -delta.x; // horizontal axis is reversed in our orientation

  // check if there was enough movement to move the cursor
  if (delta.x != 0 || delta.y != 0) {
    // if we are here, there was noticeable movement

    // the next three functions are in map_drawing.h
    erase_cursor();       // erase the current cursor
    move_cursor(delta);   // move the cursor, and the map view if the edge was nudged
    if (shared.redraw_map == 0) {
      // it looks funny if we redraw the cursor before the map scrolls
      draw_cursor();      // draw the new cursor position
    }
  }
}

int main() {
  setup();

  // very simple finite state machine:
  // which endpoint are we waiting for?
  enum {WAIT_FOR_START, WAIT_FOR_STOP} curr_mode = WAIT_FOR_START;

  // the two points that are clicked
  lon_lat_32 start, end;

  while (true) {
    // clear entries for new state
    shared.zoom_in_pushed = 0;
    shared.zoom_out_pushed = 0;
    shared.joy_button_pushed = 0;
    shared.redraw_map = 0;

    // reads the three buttons and joystick movement
    // updates the cursor view, map display, and sets the
    // shared.redraw_map flag to 1 if we have to redraw the whole map
    // NOTE: this only updates the internal values representing
    // the cursor and map view, the redrawing occurs at the end of this loop
    process_input();

    // if a zoom button was pushed, update the map and cursor view values
    // for that button push (still need to redraw at the end of this loop)
    // function zoom_map() is from map_drawing.h
    if (shared.zoom_in_pushed) {
      zoom_map(1);
      shared.redraw_map = 1;
    }
    else if (shared.zoom_out_pushed) {
      zoom_map(-1);

      // Write A when button two is pressed
      shared.redraw_map = 1;
    }

    // if the joystick button was clicked
    if (shared.joy_button_pushed) {

      if (curr_mode == WAIT_FOR_START) {
        // if we were waiting for the start point, record it
        // and indicate we are waiting for the end point
        start = get_cursor_lonlat();
        curr_mode = WAIT_FOR_STOP;
        status_message("TO?");

        // wait until the joystick button is no longer pushed
        while (digitalRead(clientpins::joy_button_pin) == LOW) {}
        delay(100); // To remove bounce in button
      }
      else {
        // if we were waiting for the end point, record it
        // and then communicate with the server to get the path
        end = get_cursor_lonlat();

        // TODO: communicate with the server to get the waypoints
        Serial.println("R " + String(start.lat) + " " + String(start.lon) + " "
          + String(end.lat) + " " + String(end.lon));

        // Based off client_base
        unsigned long time = millis();
        bool serialAvailable = false;
        bool valid = false;
        bool timedOut = false;
        unsigned long timeOutPreference = 10000;
        while(serialAvailable || (!timedOut)) {
          // read the incoming byte:
          serialAvailable = Serial.available();
          timedOut = (millis() - time) > timeOutPreference;

          if(serialAvailable){
            char in_char = Serial.read();

            // if end of line is received, waiting for line is done:
            if (in_char == '\n' || in_char == '\r') {
              valid = process_line();

              // Updates timeout time
              if(valid){
                time = millis();
                timeOutPreference = 1000;
              }      
            }else {
              // add character to buffer, provided that we don't overflow.
              // drop any excess characters.
              if ( inputBuf_len < buf_size-1 ) {
                  inputBuffer[inputBuf_len] = in_char;
                  inputBuf_len++;
                  inputBuffer[inputBuf_len] = 0;
              }
            }
          }
        }

        if(timedOut){
          curr_mode = WAIT_FOR_START;
          status_message("FROM?");
        }
        
        // now we have stored the path length in
        // shared.num_waypoints and the waypoints themselves in
        // the shared.waypoints[] array, switch back to asking for the
        // start point of a new request
        curr_mode = WAIT_FOR_START;

        // wait until the joystick button is no longer pushed
        while (digitalRead(clientpins::joy_button_pin) == LOW) {}
      }
    }

    if (shared.redraw_map) {
      // redraw the map and cursor
      draw_map();
      draw_cursor();
      drawWayPoints();


      // redraw the status message
      if (curr_mode == WAIT_FOR_START) {
        status_message("FROM?");
      }
      else {
        status_message("TO?");
      }
    }
  }

  Serial.flush();
  return 0;
}

bool process_line() {
  bool valid = true;

  if(inputBuffer[0] == 'N'){
    // Gets a string of the line
    String nStr = String(inputBuffer).substring(2,inputBuf_len);

    // Parses line for number of waypoints
    shared.num_waypoints = nStr.toInt();

    // Checks if number of waypoints is less than maxium number of waypoints
    if(shared.num_waypoints <= max_waypoints){
      Serial.println("A");
    }

  }else if(inputBuffer[0] == 'W'){
    String line = String(inputBuffer).substring(2, inputBuf_len);
    ll * values = parseRequestStringToll(line); // Parse the two values

    // Outputed values are Lat then Lon, but stored as lon then lat
    shared.waypoints[currentWaypoint] = lon_lat_32(*(values + 1), *(values));
    currentWaypoint++;

    Serial.println("A");
  }else if(inputBuffer[0] == 'E'){
    Serial.println("A");

    currentWaypoint = 0;
    drawWayPoints();
    status_message("FROM?");
  }else{ // If not a valid request
    valid = false;
  }

  // clear the buffer
  inputBuf_len = 0;
  inputBuffer[inputBuf_len] = 0;

  return valid;
}

// Referenced tutorialspoint.com/cplusplus/cpp_return_arrays_from_functions.htm
// Parses waypoint string for the two long long values
ll * parseRequestStringToll(String& line){
    int wordCount = 0;
    String strll[2];
    static ll numbers[2];

    char temp;
    for(int i = 0; i < line.length(); i++){
        temp = line.charAt(i);
        if(temp == ' '){
            wordCount++;
        }else{
            strll[wordCount] += temp;
        }
    }
    for(int i = 0; i < 2; i++)
        numbers[i] = stringToll(strll[i]);
    return numbers;
}

// Referenced arduino.stackexchange.com/questions/24447/convert-string-to-unsigned-long-long
// Converts a string to long long
ll stringToll(String& str){
  ll result = 0;
  bool isNegative = false;
  for(int i = 0; i < str.length(); i++){
    char c = str.charAt(i);
    if(c == '-')
      isNegative = true;
    else if(c < '0' || c > '9') break;
    else{
      result *= 10; // Shift results left by 1 digit
      result += (c - '0'); // Converts char to int
    }
  }
  if(isNegative)
    result *= -1;

  return result;
}

// Draws the waypoints
void drawWayPoints(){
  for(int i = 0; i < shared.num_waypoints - 1; i++){
    
    lon_lat_32 p1 = shared.waypoints[i];
    lon_lat_32 p2 = shared.waypoints[i + 1];

    shared.tft->drawLine(
        longitude_to_x(shared.map_number, p1.lon) - shared.map_coords.x, 
        latitude_to_y(shared.map_number, p1.lat) - shared.map_coords.y,
        longitude_to_x(shared.map_number,p2.lon) - shared.map_coords.x, 
        latitude_to_y(shared.map_number,p2.lat) - shared.map_coords.y,
         0x001F);
  }
}