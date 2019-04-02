/*****************************
* Names: Kevin Wedage, Aryan Singh
* IDs: 1532557, 1533732
* Comput 275, Winter 2019
* Assignment 2 Part 2
*****************************/

#include <iostream>
#include <unordered_map>
#include <vector>
#include <fstream>
#include <unordered_map>
#include <list>
#include <string>
#include "serialport.h"
#include "wdigraph.h"
#include "dijkstra.h"
#include "heap.h"

using namespace std;

// Define type abreviations
typedef long long ll;
typedef pair<ll, int> PLI;

// Used to store locations
struct Point{
    ll lat;
    ll lon;
};

// Forward Declarations
void parseVertex(WDigraph& graph, unordered_map<int, Point>& points, const string& line);
void parseEdge(WDigraph& graph, unordered_map<int, Point>& points, const string& line);
void readGraph(string filename, WDigraph& graph, unordered_map<int, Point>& points);
int findIndexOfCommaAfter(const string& line, const int offset);
ll scaledllCast(const double f);
ll manhattan(const Point& pt1, const Point& pt2);
int nearestVertex(const unordered_map<int, Point>& points, ll lat, ll lon);
void manageSerial(SerialPort& Serial, const WDigraph& graph, unordered_map<int, Point>& points);
void readRequest(SerialPort& Serial, const WDigraph& graph, unordered_map<int, Point>& points,
    ll startLat, ll startLon, ll finalLat, ll finalLon);
ll * parseRequestStringToll(string& line);
bool readATimedOut(SerialPort& Serial, int timeout);
void noPath(SerialPort& Serial);

int main(){
    // Initialize serial port
    SerialPort Serial("/dev/ttyACM0");

    // Create a weighted graph
    WDigraph graph;
    // Created an unordered_map of points
    unordered_map<int, Point> points;
    // Read in the edmonton map, and set the graph, and points
    readGraph("edmonton-roads-2.0.1.txt", graph, points);
    
    manageSerial(Serial, graph, points);
    
    return 0;
}

// Handles Serial
void manageSerial(SerialPort& Serial, const WDigraph& graph, unordered_map<int, Point>& points){
    string line;
    while(true){
        // Check if a request was sent.
        do {
            line = Serial.readline();
        }while (line.substr(0,1) != "R");

        // Remove R and Space
        line = line.substr(2, line.size());
        ll *values = parseRequestStringToll(line); // Determines the values in the request
        cout << "Successfully read in request" << endl;
        readRequest(Serial, graph, points, *(values), *(values + 1), *(values + 2), *(values + 3));
        cout << "Finished Request" << endl;
    }
}

// Handles a request
void readRequest(SerialPort& Serial, const WDigraph& graph, unordered_map<int, Point>& points,
    ll startLat, ll startLon, ll finalLat, ll finalLon){
    bool timedOut = false;

    // Gets the nearest vertex to each location specified
    int startVertex = nearestVertex(points, startLat, startLon);
    int finalVertex = nearestVertex(points, finalLat, finalLon);

    cout << "Start Vertex: " << startVertex << ", End Vertex: " << finalVertex << endl;
    
    unordered_map<int, PLI> searchTree;
    dijkstra(graph, startVertex, searchTree);
    // Gets a search tree from the start vertex
    
    // Checks if the final vertex is reachable from the start vertex
    // Code referenced from dijkstra.cpp file from eclass
    if(searchTree.find(finalVertex) == searchTree.end()){
        noPath(Serial);
    }else{
        // Initalize a list path
        list<int> path;
        int waypoints = 1;
        int stepping = finalVertex;

        // Crawls back up the search tree using the predcessor of 
        // the current vertex, saving it the the path list.
        while(stepping != startVertex){

            path.push_front(stepping);
            stepping = searchTree[stepping].second;
            waypoints++;
        }
        path.push_front(startVertex);

        // If the number of wavepoints exceeds max amount
        if(waypoints > 500){
            noPath(Serial);
        }else{
            // Prints out the number of wavepoints
            cout << "N " << waypoints << endl;
            Serial.writeline("N " + to_string(waypoints) + "\n");
            
            // Prints wavepoints and waits for arduino response
            string line;
            for (auto it : path){
                timedOut = readATimedOut(Serial, 1000);

                if(timedOut) break;

                cout << "W " << points[it].lat << " " << points[it].lon << endl;
                Serial.writeline("W " + to_string(points[it].lat) + " " + to_string(points[it].lon) + "\n");
            }        
            if(timedOut) return;

            timedOut = readATimedOut(Serial, 1000);
            if(timedOut) return;

            // End of request
            cout << 'E' << endl;
            Serial.writeline("E\n");
        }
    }
}

// Outputs result, when no path was reachable
void noPath(SerialPort& Serial){
    cout << "N 0" << endl; // No path
    Serial.writeline("N 0\n");
}

// Reads in the textfile, and parses it for vertexes and edges
void readGraph(string filename, WDigraph& graph, unordered_map<int, Point>& points){
    ifstream mapFile(filename);

    // Open file corectly
    if(mapFile.is_open()){
        string line;
        string type;
        while(getline(mapFile,line)){
            type = line.substr(0,1); // Gets the first character
            if(type == "V"){
                parseVertex(graph, points, line.substr(2, line.size()));
            }else{
                parseEdge(graph, points, line.substr(2, line.size()));
            }
        }
    }

    mapFile.close();
}

// Parse a vertex from a string
void parseVertex(WDigraph& graph, unordered_map<int, Point>& points, const string& line){
    int vertexId;
    double lat, lon;
    int commaIndex1, commaIndex2; 
    Point point;

    // Index of the first comma in the textfile
    commaIndex1 = findIndexOfCommaAfter(line,0);
    vertexId = stoi(line.substr(0,commaIndex1));
    // Gets a substring of the line upto the comma, then converts to integer.

    // Index of the second comma
    // Likewise gets substrings and converts strings to double
    commaIndex2 = findIndexOfCommaAfter(line, commaIndex1 + 1);
    lat = stod(line.substr(commaIndex1 + 1, commaIndex2 - 1));
    lon = stod(line.substr(commaIndex2 + 1, line.size()));

    // Scales and casts the double to long long
    point.lat = scaledllCast(lat);
    point.lon = scaledllCast(lon);

    // Adds the vertex to the graph
    graph.addVertex(vertexId);
    points[vertexId] = point;
}

// Parses an edge from a string
void parseEdge(WDigraph& graph, unordered_map<int, Point>& points, const string& line){
    int vertex1, vertex2, commaIndex1, commaIndex2;

    // Gets substrings from line based on indexes of commas,
    // then converts to integers
    commaIndex1 = findIndexOfCommaAfter(line, 0);
    vertex1 = stoi(line.substr(0, commaIndex1));
    commaIndex2 = findIndexOfCommaAfter(line, commaIndex1 + 1);
    vertex2 = stoi(line.substr(commaIndex1 + 1, commaIndex2 - 1));

    // Adds the edge to graph, after determining cost using manhanttan distance
    ll cost = manhattan(points[vertex1],points[vertex2]);
    graph.addEdge(vertex1, vertex2, cost);
}

// Determines the nearest vertex to the lat and lon provided
int nearestVertex(const unordered_map<int, Point>& points, ll lat, ll lon){
    int minMHD = 0, tempMHD;
    int vertex = -1;
    Point location;
    location.lat = lat;
    location.lon = lon;

    // Determines the min manhattan distance to the location, based on 
    // the points of each vertex.
    bool initalized = false;
    for(auto p : points){
        tempMHD = manhattan(p.second,location);
        if(!initalized || (tempMHD < minMHD)){
            minMHD = tempMHD;
            vertex = p.first;
            initalized = true;
        }
    }
    return vertex;
}

// Scaled cast of the double to long long
ll scaledllCast(const double d){
    double offset = 0.0000001; // Offset to fix rounding issue
    if(d < 0)
        offset *= -1;
    return static_cast <ll> ((d + offset)*100000);
}

// Determines the manhattan distance
ll manhattan(const Point& pt1, const Point& pt2){
    return abs(pt1.lat - pt2.lat) + abs(pt1.lon - pt2.lon);
}
// Finds the first comma in string
int findIndexOfCommaAfter( const string& line, const int offset){
    int commaIndex = -1;
    unsigned index = offset;
    while(index < line.size()){
        if(line.at(index) == ','){
            commaIndex = index;
            break;
        }
        index++;
    }
    return commaIndex;
}

// Referenced tutorialspoint.com/cplusplus/cpp_return_arrays_from_functions.htm
// Parses request string for the 4 long long values within
ll * parseRequestStringToll(string& line){
    int wordCount = 0;
    string strll[4];
    static ll numbers[4];

    string temp;
    for(auto it = line.begin(); it != line.end(); ++it){
        temp = *it;
        if(temp == " "){ // If space is reached, increment word count
            wordCount++;
        }else{
            strll[wordCount] += temp;
        }
    }
    // If last character did not have a space at the end, missing last character
    if(line.substr(line.size() - 1, line.size()) != " "){
        strll[3] += temp;
    }

    // Convert strings to long longs
    for(int i = 0; i < 4; i++)
        numbers[i] = stoll(strll[i]);
    return numbers;
}

// Waits for Serial A, and has timeout
// Returns whether timeout occured
bool readATimedOut(SerialPort& Serial, int timeout){
    string line = "Empty";
    do{
        if(timeout == -1){ // No timeout
            line = Serial.readline();
        }else{
            line = Serial.readline(timeout);   
        }
        if(line == "") break;
    }while (line.substr(0,1) != "A");

    if(line == ""){
        cout << "Server: Request timed out" << endl;
        cout << "Waiting for new request" << endl;
        return true;
    }else{
        return false;
    }

}