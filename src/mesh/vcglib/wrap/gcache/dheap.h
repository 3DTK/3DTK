/****************************************************************************
* GCache                                                                    *
* Author: Federico Ponchio                                                  *
*                                                                           *
* Copyright(C) 2011                                                         *
* Visual Computing Lab                                                      *
* ISTI - Italian National Research Council                                  *
*                                                                           *
* All rights reserved.                                                      *
*                                                                           *
* This program is free software; you can redistribute it and/or modify      *   
* it under the terms of the GNU General Public License as published by      *
* the Free Software Foundation; either version 2 of the License, or         *
* (at your option) any later version.                                       *
*                                                                           *
* This program is distributed in the hope that it will be useful,           *
* but WITHOUT ANY WARRANTY; without even the implied warranty of            *
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
* GNU General Public License (http://www.gnu.org/licenses/gpl.txt)          *
* for more details.                                                         *
*                                                                           *
****************************************************************************/

#ifndef DD_HEAP_H
#define DD_HEAP_H

/**
  Double ended heap inspired by
  Min-Max Heaps and Generalized Priority Queues
  M. D. ATKINSON,J.-R. SACK, N. SANTORO,and T. STROTHOTTE

  This structure allows for quick extraction of biggest and smaller item out of a set
  with linear reconstruction of the ordering.

  DHeap exposes the public interface of vector. (push_back(), resize() etc.).

  Compared to a stl heap, rebuild is 15% longer, extraction is 2x longer,
  but you get both min and max extraction in log(n) time.
*/

#include <assert.h>
#include <vector>


template <class T>
class DHeap: public std::vector<T> {
public:

  void push(const T& elt) {
    this->push_back(elt);
    bubbleUp(this->size()-1);
  }

  T &min() { return this->front(); } //root is smallest element

  T popMin() {
    T elt = this->front();
    //move the last element to the root and
    this->front() = this->back();
    this->pop_back();
    //enforce minmax heap property
    trickleDownMin(0);
    return elt;
  }

  //max is second element
  T &max() {
    if(this->size() == 1) return at(0);
    return at(1);
  }

  T popMax() {
    int p = 1;
    if(this->size() == 1) p = 0;
    T elt = at(p);
    //max is replaced with last item.
    at(p) = this->back();
    this->pop_back();
    trickleDownMax(p); //enforce minmax heap property
    return elt;
  }

  //just reinsert all elements
  void rebuild() { 
    for(unsigned int i = 0; i < this->size(); i++)
      bubbleUp(i);
  }

protected:
  T &at(int n) { return std::vector<T>::at(n); }

  int isMax(int e) const { return e & 1; }
  int parentMin(int i) const { return (((i+2)>>2)<<1) - 2; }
  int parentMax(int i) const { return (((i+2)>>2)<<1) - 1; }
  int leftChildMin(int i) const { return (((i+2)>>1)<<2) -2; }
  int leftChildMax(int i) const { return (((i+2)>>1)<<2) -1; }

  void swap(int a, int b) { T tmp = at(a); at(a) = at(b); at(b) = tmp; }

  //returns smallest elemennt of children intervals (or self if no children)
  int smallestChild(int i) {
    int l = leftChildMin(i);
    if(l >= this->size()) return i; //no children, return self

    int r = l+2; //right child
    if(r < this->size() && at(r) < at(l))
      return r;
    return l;
  }
  //return biggest children or self if no children
  int greatestChild(int i) {
    int l = leftChildMax(i);
    if(l >= this->size()) return i; //no children, return self

    int r = l+2; //right child
    if(r < this->size() && at(r) > at(l))
      return r;
    return l;
  }

  //all stuff involving swaps could be optimized perofming circular swaps
  // but you mantain the code after :)
  void trickleDownMin(int i) {
    while(1) {

      //find smallest child
      unsigned int m = leftChildMin(i);
      if(m >= this->size()) break;
      unsigned int r = m+2;
      if(r < this->size() && at(r) < at(m))
        m = r;

      if(at(m) < at(i)) { //if child is smaller swap
        swap(i, m);
        i = m; //check swapped children
      } else //no swap? finish
        break;

      m = i+1;       //enforce order in interval
      if(m >= this->size()) break;
      if(at(m) < at(i))
        swap(i, m);
    }
  }

  void trickleDownMax(int i) {
    while(1) {

      //find greatest child
      unsigned int m = leftChildMax(i);
      if(m >= this->size()) break;
      unsigned int r = m+2;
      if(r < this->size() && at(r) > at(m))
        m = r;

      if(at(m) > at(i)) {
        swap(i, m);
        i = m;
      } else
        break;

      m = i-1;       //enforce order in interval
      if(m >= this->size()) break;
      if(at(m) > at(i)) {
        swap(i, m);
      }
    }
  }

  void bubbleUpMin(int i) {
    while(1) {
      int m = parentMin(i);
      if(m < 0) break;
      if(at(m) > at(i)) {
        swap(i, m);
        i = m;
      } else
        break;
    }
  }

  void bubbleUpMax(int i) {
    while(1) {
      int m = parentMax(i);
      if(m < 0) break;
      if(at(m) < at(i)) {
        swap(i, m);
        i = m;
      } else
        break;
    }
  }

  void bubbleUp(int i) {
    if(isMax(i)) {
      int m = i-1;
      if(at(m) > at(i)) {
        swap(i, m);
        bubbleUpMin(m);
      } else
        bubbleUpMax(i);
    } else {
      int m = parentMax(i);
      if(m < 0) return;
      if(at(m) < at(i)) {
        swap(i, m);
        bubbleUpMax(m);
      } else
        bubbleUpMin(i);//just reinsert all elements, (no push back necessary, of course
    }
  }
  /* DEBUG */
 public:
  ///check the double heap conditions are met, mainly for debugging purpouses
  bool isHeap() { //checks everything is in order
    int s = this->size();
    for(int i = 0; i < s; i += 2) {
      if(i+1 < s && at(i) > at(i+1)) return false;
      int l = leftChildMin(i);
      if(l < s && at(i) > at(l)) return false;
      int r = l + 2;
      if(r < s && at(i) > at(r)) return false;
    }
    for(int i = 1; i < s; i += 2) {
      int l = leftChildMax(i);
      if(l < s && at(i) < at(l)) return false;
      int r = l + 2;
      if(r < s && at(i) < at(r)) return false;
    }
    return true;
  }
};

/** Same functionality as IHeap, but storing pointers instead of the objects */

template <class T>
class PtrDHeap {
 private:
  class Item {
    public:
    T *value;
    Item(T *val): value(val) {}
    bool operator<(const Item &i) const { return *value < *i.value; }
    bool operator>(const Item &i) const { return *value > *i.value; }
  };
  DHeap<Item> heap;

 public:
  T *push(T *t) {
    Item i(t);
    heap.push(i);
    return i.value;
  }
  void push_back(T *t) {
    heap.push_back(Item(t));
  }
  int size() { return heap.size(); }
  void resize(int n) { assert(n <= (int)heap.size()); return heap.resize(n, Item(NULL)); }
  void clear() { heap.clear(); }
  T &min() { Item &i = heap.min(); return *i.value; }
  T *popMin() { Item i = heap.popMin(); return i.value; }

  T &max() { Item &i = heap.max(); return *i.value; }
  T *popMax() { Item i = heap.popMax(); return i.value; }

  void rebuild() { heap.rebuild(); }
  T &operator[](int i) {
    return *(heap[i].value);
  }
  Item &at(int i) { return heap[i]; }
  bool isHeap() { return heap.isHeap(); }
};

#endif
