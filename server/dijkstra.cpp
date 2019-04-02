/*****************************
* Names: Kevin Wedage, Aryan Singh
* IDs: 1532557, 1533732
* Comput 275, Winter 2019
* Assignment 2 Part 2
*****************************/

#include "dijkstra.h"

using namespace std;

// Define type abreviations 
typedef long long ll;
typedef pair<ll, int> PLI;
typedef pair<int,int> VU; // V vertex (current vertex), U vertex (predcessor vertex)

// Based Eclass dijkstra algorithm 3 from heap slides.
void dijkstra(const WDigraph& graph, int startVertex, unordered_map<int, PLI>& tree){
    BinaryHeap <VU, ll> events;
    events.insert(make_pair(startVertex,-1),0);
    // Insert the start vertex, with a predcessor of -1, and cost 0

    // While there is events
    while(events.size() > 0){

        // Pop the root event (min cost)
        pair <VU, ll> min = events.min();
        events.popMin();

        // Get the current vertex v, predcessor u, and cost c
        int v = min.first.first;
        int u = min.first.second;
        ll c = min.second;

        // If v is not in the search tree
        if(tree.find(v) == tree.end()){
            // Insert v into the tree
            tree.insert(make_pair(v,
                make_pair(c, u)));

            // Add all the neighbours of v to events
            for (auto iter = graph.neighbours(v); iter != graph.endIterator(v); iter++) {
                int nbh = *iter;
                if (tree.find(nbh) == tree.end()) {
                    events.insert(make_pair(nbh,v),
                        (c + graph.getCost(v,nbh)));
                }
            }
        }
    }
}


/*
    BinaryHeap <VU, ll> events;
    events.insert(make_pair(startVertex, -1), 0);

    
    while(events.size() > 0){
        
        pair <VU, ll> min = events.min();

        int v = min.first.first;
        int u = min.first.second;
        ll c = min.second;
        
        events.popMin();

        
        if (tree.find(v) == tree.end()){
            tree.insert(make_pair(v,make_pair(c, u)));

            // Get neigbours of v
            for (auto iter = graph.neighbours(v); iter != graph.endIterator(v); iter++) {
                int nbh = *iter;
                if (tree.find(nbh) == tree.end()) {
                    events.insert(make_pair(nbh,v), c);
                    // + graph.getCost(nbh,v)
                }
            }
        }
    }
*/  