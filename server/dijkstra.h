/*****************************
* Names: Kevin Wedage, Aryan Singh
* IDs: 1532557, 1533732
* Comput 275, Winter 2019
* Assignment 2 Part 1
*****************************/

#ifndef _DIJKSTRA_H_
#define _DIJKSTRA_H_

#include <unordered_map>
#include "wdigraph.h"
#include "heap.h"

typedef long long ll;
typedef pair<ll, int> PLI;
typedef pair<int,int> VU;

void dijkstra(const WDigraph& graph, int startVertex, unordered_map<int, PLI>& tree);

#endif
