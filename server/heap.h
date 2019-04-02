/*****************************
* Names: Kevin Wedage, Aryan Singh
* IDs: 1532557, 1533732
* Comput 275, Winter 2019
* Assignment 2 Part 2
*****************************/

#ifndef _HEAP_H_
#define _HEAP_H_

#include <vector>
#include <utility> // for pair

// T is the type of the item to be held
// K is the type of the key associated with each item in the heap
// The only requirement is that K is totally ordered and comparable via <
template <class T, class K>
class BinaryHeap {
public:
  // constructor not required because the only "initialization"
  // is through the constructor of the variable "heap" which is called by default

  // return the minimum element in the heap
  std::pair<T, K> min() const;

  // insert an item with the given key
  // if the item is already in the heap, will still insert a new copy with this key
  void insert(const T& item, const K& key);

  // pop the minimum item from the heap
  void popMin();

  // returns the number of items held in the heap
  int size() const;

private:
  // the array holding the heap
  std::vector< std::pair<T, K> > heap;

  // Swaps to indexs in the heap
  void swap(int currentIndex, int otherIndex);
};

template <class T, class K>
std::pair<T,K> BinaryHeap<T,K>::min() const{
  return heap.at(0);
}

template <class T, class K>
int BinaryHeap<T,K>::size() const{
  return heap.size();
}

// Based off eclass heap insert slides
template <class T, class K>
void BinaryHeap<T,K>::insert(const T& item, const K& key){
  heap.push_back(make_pair(item, key)); // Insert vertex to the end of heap

  int parentIndex;
  int currentIndex = size() - 1;

  // Check if min-heap property holds
  while(true){
    if(currentIndex == 0) // If vertex is the root break
      break; 

    parentIndex = (currentIndex - 1)/2;

    if(heap.at(currentIndex).second < heap.at(parentIndex).second){
      swap(currentIndex, parentIndex);
      currentIndex = parentIndex;
    }else{ // otherwise heap property holds and break
      break;
    }
  }
}

// Swaps the two indexes in the heap
template <class T, class K>
void BinaryHeap<T,K>::swap(int currentIndex, int otherIndex){
  T tempT = heap.at(otherIndex).first;
  K tempK = heap.at(otherIndex).second;
  heap.at(otherIndex).first = heap.at(currentIndex).first;
  heap.at(otherIndex).second = heap.at(currentIndex).second;
  heap.at(currentIndex).first = tempT;
  heap.at(currentIndex).second = tempK;
}

// Based on eclass heap slides
template <class T, class K>
void BinaryHeap<T,K>::popMin(){
  swap(0, size() - 1); // Swap first and last element
  heap.erase(heap.begin() + size() - 1); // Delete the last (first) after swaping

  int currentIndex = 0;
  int leftChild, rightChild, minIndex;
  
  // Check if heap property holds
  while(true){
    leftChild = (2*currentIndex + 1);
    rightChild = (2*currentIndex + 2);

    // Checking for index out bounds:
    if(rightChild >= size() && leftChild < size()){ // Current Index only has left child
      minIndex = leftChild;
    }else if(leftChild >= size()){ // Current Index has no children
      break;
    }else{ // Current Index has both children
      if(heap.at(leftChild).second < heap.at(rightChild).second){
        minIndex = leftChild;
      }else{
        minIndex = rightChild;
      }
    }

    // Compares the child with min key, with the current
    if(heap.at(minIndex).second < heap.at(currentIndex).second){
      swap(minIndex, currentIndex);
      currentIndex = minIndex;
    }else{ // Heap Property Holds
      break;
    }
  }
}

#endif