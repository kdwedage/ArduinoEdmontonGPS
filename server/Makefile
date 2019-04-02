CC=g++
CFLAGS=-c -Wall -O2 -std=c++11
PROGS= server
OBJS= server.o dijkstra.o digraph.o serialport.o
INPUT= edmonton-roads-2.0.1.txt

# executable targets
server: $(OBJS) $(INPUT)
	$(CC) $(OBJS) -o server

# object targets
server.o: server.cpp
	$(CC) server.cpp -o server.o $(CFLAGS)

dijkstra.o: dijkstra.cpp dijkstra.h heap.h
	$(CC) dijkstra.cpp -o dijkstra.o $(CFLAGS)

digraph.o: digraph.cpp digraph.h
	$(CC) digraph.cpp -o digraph.o $(CFLAGS)

serialport.o: serialport.h serialport.cpp
	$(CC) serialport.cpp -o serialport.o $(CFLAGS)

clean:
	@rm $(OBJS) $(PROGS)
