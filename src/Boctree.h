/**
 * @file 
 * @brief Efficient representation of an octree
 * @author Jan Elsberg. Automation Group, Jacobs University Bremen gGmbH, Germany. 
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef BOCTREE_H
#define BOCTREE_H

#include <stdio.h>

#include <vector>
using std::vector;
#include <deque>
using std::deque;
#include <set>
using std::set;
#include <list>
using std::list;
#include <iostream>
#include <fstream>
#include <string>

#include "globals.icc"

#if __GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4)
  #define POPCOUNT(mask) __builtin_popcount(mask)
#else
  #define POPCOUNT(mask) _my_popcount_3(mask)
#endif


// forward declaration
template <class T> union bitunion;

/**
 * This is our preferred representation for the leaf nodes (as it is the most compact). 
 * BOctTree makes an array of this, the first containing the number of points (not the 
 * number of coordinates) stored.
 */
template <class T> union dunion {
  T v;
  unsigned int    length;
  dunion() : length(0) {};

};
// typedefs in combination with templates are weird
//typedef dunion<T> pointrep<T>;
#define pointrep union dunion<T>

/**
 * This struct represents the nodes of the octree
 *
 * child_pointer is a relative pointer to the first child of this node, as it is only
 * 48 bit this will cause issues on systems with more than 268 TB of memory. All children
 * of this node must be stored sequentially. If one of the children is a leaf, that
 * child will be a pointer to however a set of points is represented (pointrep *).
 *
 * valid is a bitmask describing wether the corresponding buckets are filled.
 *
 * leaf is a bitmask describing wether the correpsonding bucket is a leaf node.
 *
 * The representation of the bitmask is somewhat inefficient. We use 16 bits for only 
 * 3^8 possible states, so in essence we could save 3 bits by compression.
 *
 */
template <class T> class bitoct{
  public:

#ifdef _MSC_VER
  __int64 child_pointer        : 48;
  unsigned valid              :  8;
  unsigned leaf               :  8;
#else
  signed long child_pointer   : 48;
  unsigned valid              :  8;
  unsigned leaf               :  8;
#endif
  /**
   * sets the child pointer of parent so it points to child 
   */
  static inline void link(bitoct<T> &parent, bitunion<T> *child) {
    parent.child_pointer = (long)((char*)child - (char*)&parent);
  }

  /**
   * Returns the children of this node (given as parent).
   */
  static inline void getChildren(bitoct<T> &parent, bitunion<T>* &children) {
    children = (bitunion<T>*)((char*)&parent + parent.child_pointer);
  }
 

};
  

/**
 * This union combines an octree node with a pointer to a set of points. This allows
 * us to use both nodes and leaves interchangeably.
 *
 * points is a pointer to the point representation in use
 *
 * node is simply the octree node
 *
 */
template <class T> union bitunion {
  pointrep *points;
  //union dunion<T> *points;
  bitoct<T> node;

  bitunion(pointrep *p) : points(p) {};
  bitunion(bitoct<T> b) : node(b) {};
  bitunion() : points(0) {
    node.child_pointer = 0;
    node.valid = 0;
  };           // needed for new []
};


/**
 * @brief Octree
 * 
 * A cubic bounding box is calculated
 * from the given 3D points. Then it
 * is recusivly subdivided into smaller
 * subboxes
 */
template <class T> class BOctTree {

public:
  
  BOctTree(T **pts, int n, T voxelSize, unsigned int pointdim = 3) {
    this->voxelSize = voxelSize;

    this->POINTDIM = pointdim;

    T xmin = pts[0][0], xmax = pts[0][0];
    T ymin = pts[0][1], ymax = pts[0][1];
    T zmin = pts[0][2], zmax = pts[0][2];
    for (int i = 1; i < n; i++) {
      xmin = min(xmin, pts[i][0]);
      xmax = max(xmax, pts[i][0]);
      ymin = min(ymin, pts[i][1]);
      ymax = max(ymax, pts[i][1]);
      zmin = min(zmin, pts[i][2]);
      zmax = max(zmax, pts[i][2]); 
    }
    center[0] = 0.5 * (xmin+xmax);
    center[1] = 0.5 * (ymin+ymax);
    center[2] = 0.5 * (zmin+zmax);
    size = max(max(0.5 * (xmax-xmin), 0.5 * (ymax-ymin)), 0.5 * (zmax-zmin));


    // calculate new buckets
    T newcenter[8][3];
    T sizeNew = size / 2.0;

    for (int i = 0; i < 8; i++) {
      childcenter(center, newcenter[i], size, i);
    }
    // set up values
    root = new bitoct<T>();

    countPointsAndQueue(pts, n, newcenter, sizeNew, *root);
  }

  BOctTree(std::string filename, T *minmax = 0) {deserialize(filename, minmax); }
  BOctTree(vector<T *> &pts, T voxelSize, unsigned int pointdim = 3) {
    this->voxelSize = voxelSize;

    this->POINTDIM = pointdim;

    T xmin = pts[0][0], xmax = pts[0][0];
    T ymin = pts[0][1], ymax = pts[0][1];
    T zmin = pts[0][2], zmax = pts[0][2];
    for (int i = 1; i < pts.size(); i++) {
      xmin = min(xmin, pts[i][0]);
      xmax = max(xmax, pts[i][0]);
      ymin = min(ymin, pts[i][1]);
      ymax = max(ymax, pts[i][1]);
      zmin = min(zmin, pts[i][2]);
      zmax = max(zmax, pts[i][2]); 
    }
    center[0] = 0.5 * (xmin+xmax);
    center[1] = 0.5 * (ymin+ymax);
    center[2] = 0.5 * (zmin+zmax);
    size = max(max(0.5 * (xmax-xmin), 0.5 * (ymax-ymin)), 0.5 * (zmax-zmin));


    // calculate new buckets
    T newcenter[8][3];
    T sizeNew = size / 2.0;

    for (int i = 0; i < 8; i++) {
      childcenter(center, newcenter[i], size, i);
    }
    // set up values
    root = new bitoct<T>();

    countPointsAndQueue(pts, newcenter, sizeNew, *root);
  }

  virtual ~BOctTree(){
    deletetNodes(*root);
    delete root;
  } 
  
  void GetOctTreeCenter(vector<T*>&c) { GetOctTreeCenter(c, *root, center, size); }
  void GetOctTreeRandom(vector<T*>&c) { GetOctTreeRandom(c, *root); }
  void GetOctTreeRandom(vector<T*>&c, unsigned int ptspervoxel) { GetOctTreeRandom(c, ptspervoxel, *root); }
  
  long countNodes() { return 1 + countNodes(*root); }
  long countLeaves() { return 1 + countLeaves(*root); }


  void deserialize(std::string filename, T *minmax = 0) {
    char buffer[sizeof(T) * 20];
    T *p = reinterpret_cast<T*>(buffer);

    std::ifstream file;
    file.open (filename.c_str(), std::ios::in | std::ios::binary);

    // read magic bits
    file.read(buffer, 2);
    if ( buffer[0] != 'X' || buffer[1] != 'T') {
      std::cerr << "Not an octree file!!" << endl;
      file.close();
    }

    // read header
    file.read(buffer, 5 * sizeof(T));
    voxelSize = p[0];
    center[0] = p[1];
    center[1] = p[2];
    center[2] = p[3];
    size = p[4];

    file.read(buffer, sizeof(int));
    int *ip = reinterpret_cast<int*>(buffer);
    POINTDIM = *ip;


    if (minmax) {
      file.read(reinterpret_cast<char*>(minmax), 2*POINTDIM * sizeof(T));
    } else {  // skip
      file.read(buffer, 2*POINTDIM * sizeof(T));
    }

    // read root node
    root = new bitoct<T>();
    deserialize(file, *root);
    file.close();
  }
  
  void serialize(std::string filename, T *minmax = 0) {
    char buffer[sizeof(T) * 20];
    T *p = reinterpret_cast<T*>(buffer);

    std::ofstream file;
    file.open (filename.c_str(), std::ios::out | std::ios::binary);

    // write magic bits
    buffer[0] = 'X';
    buffer[1] = 'T';
    file.write(buffer, 2);

    // write header
    p[0] = voxelSize;
    p[1] = center[0]; 
    p[2] = center[1]; 
    p[3] = center[2];
    p[4] = size;

    int *ip = reinterpret_cast<int*>(&(buffer[5 * sizeof(T)]));
    *ip = POINTDIM;

    file.write(buffer, 5 * sizeof(T) + sizeof(int));

    if (minmax) {
      for (unsigned int i = 0; i < 2*POINTDIM; i++) {
        p[i] = minmax[i];
      }
    } else {
      for (unsigned int i = 0; i < 2*POINTDIM; i++) {
        p[i] = 0.0; 
      }
    }
    file.write(buffer, 2*POINTDIM * sizeof(T));

    // write root node
    serialize(file, *root);

    file.close();
  }
 
protected:
  
  void deserialize(std::ifstream &f, bitoct<T> &node) {
    char buffer[2];
    f.read(buffer, 2);
    node.valid = buffer[0];
    node.leaf = buffer[1];

    unsigned short n_children = POPCOUNT(node.valid);

    // create children
    bitunion<T> *children = new bitunion<T>[n_children];
    bitoct<T>::link(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf read points 
          pointrep first;
          f.read(reinterpret_cast<char*>(&first), sizeof(pointrep));
          unsigned int length = first.length;  // read first element, which is the length
          pointrep *points = new pointrep[POINTDIM * length + 1];   // make room for points 
          children->points = points;
          points[0] = first;
          points++;
          f.read(reinterpret_cast<char*>(points), sizeof(pointrep) * length * POINTDIM); // read the points
        } else {  // write child 
          deserialize(f, children->node);
        }
        ++children; // next child
      }
    }
  }

  void serialize(std::ofstream &of, bitoct<T> &node) {
    char buffer[2];
    buffer[0] = node.valid;
    buffer[1] = node.leaf;
    of.write(buffer, 2);


    // write children
    bitunion<T> *children;
    bitoct<T>::getChildren(node, children);
    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf write points 
          pointrep *points = children->points;
          unsigned int length = points[0].length;
          of.write(reinterpret_cast<char*>(points), sizeof(pointrep) * (length * POINTDIM  +1));
        } else {  // write child 
          serialize(of, children->node);
        }
        ++children; // next child
      }
    }
  }


  void GetOctTreeCenter(vector<T*>&c, bitoct<T> &node, T *center, T size) {
    T ccenter[3];
    bitunion<T> *children;
    bitoct<T>::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        childcenter(center, ccenter, size, i);  // childrens center
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
          T * cp = new T[POINTDIM];
          for (unsigned int iterator = 0; iterator < POINTDIM; iterator++) {
            cp[iterator] = ccenter[iterator];
          }
          c.push_back(cp);
        } else { // recurse
          GetOctTreeCenter(c, children->node, ccenter, size/2.0);
        }
        ++children; // next child
      }
    }
  }

  void GetOctTreeRandom(vector<T*>&c, bitoct<T> &node) {
    bitunion<T> *children;
    bitoct<T>::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
          pointrep *points = children->points;
          int tmp = rand(points[0].length);
          T *point = &(points[POINTDIM*tmp+1].v);
          c.push_back(point);

        } else { // recurse
          GetOctTreeRandom(c, children->node);
        }
        ++children; // next child
      }
    }
  } 
  
  void GetOctTreeRandom(vector<T*>&c, unsigned int ptspervoxel, bitoct<T> &node) {
    bitunion<T> *children;
    bitoct<T>::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
          pointrep *points = children->points;
          unsigned int length = points[0].length;
          if (ptspervoxel >= length) {
            for (unsigned int j = 0; j < length; j++) 
              c.push_back(&(points[POINTDIM*j+1].v));

            ++children; // next child
            continue;
          }
          set<int> indices;
          while(indices.size() < ptspervoxel) {
            int tmp = rand(length-1);
            indices.insert(tmp);
          }
          for(set<int>::iterator it = indices.begin(); it != indices.end(); it++) 
            c.push_back(&(points[POINTDIM*(*it)+1].v));

        } else { // recurse
          GetOctTreeRandom(c, ptspervoxel, children->node);
        }
        ++children; // next child
      }
    }
  }

  long countNodes(bitoct<T> &node) {
    long result = 0;
    bitunion<T> *children;
    bitoct<T>::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
          ++result;
        } else { // recurse
          result += countNodes(children->node) + 1;
        }
        ++children; // next child
      }
    }
    return result;
  }

  long countLeaves(bitoct<T> &node) {
    long result = 0;
    bitunion<T> *children;
    bitoct<T>::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
          pointrep *points = children->points;
          long nrpts = points[0].length;
          result += POINTDIM*nrpts + 1;
        } else { // recurse
          result += countLeaves(children->node);
        }
        ++children; // next child
      }
    }
    return result;
  }


  void deletetNodes(bitoct<T> &node) {
    bitunion<T> *children;
    bitoct<T>::getChildren(node, children);
    bool haschildren = false;

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
          delete [] children->points;
        } else { // recurse
          deletetNodes(children->node);
        }
        ++children; // next child
        haschildren = true;
      }
    }
    // delete children
    if (haschildren) {
      bitoct<T>::getChildren(node, children);
      delete[] children;
    }
  }
/*
  pointrep *branch( bitoct<T> &node, deque<T*> &splitPoints, T _center[3], T _size) {
    // if bucket is too small stop building tree
    // -----------------------------------------
    if ((_size <= voxelSize)) {
      // copy points
      pointrep *points = new pointrep[POINTDIM*splitPoints.size() + 1];
      points[0].length = splitPoints.size();
      int i = 1;
      for (deque<T*>::iterator itr = splitPoints.begin(); 
          itr != splitPoints.end(); itr++) {
        for (unsigned int iterator = 0; iterator < POINTDIM; iterator++) {
          points[i++].v = (*itr)[iterator];
        }
      }
      return points; 
    }  

    // calculate new buckets
    T newcenter[8][3];
    T sizeNew;

    sizeNew = _size / 2.0;

    for (int i = 0; i < 8; i++) {
      childcenter(_center, newcenter[i], _size, i);
    }

    countPointsAndQueue(splitPoints, newcenter, sizeNew, node);
    return 0;
  }*/

  pointrep *branch( bitoct<T> &node, vector<T*> &splitPoints, T _center[3], T _size) {
    // if bucket is too small stop building tree
    // -----------------------------------------
    if ((_size <= voxelSize)) {
      // copy points
      pointrep *points = new pointrep[POINTDIM*splitPoints.size() + 1];
      points[0].length = splitPoints.size();
      int i = 1;
      for (typename vector<T *>::iterator itr = splitPoints.begin(); 
          itr != splitPoints.end(); itr++) {
        for (unsigned int iterator = 0; iterator < POINTDIM; iterator++) {
          points[i++].v = (*itr)[iterator];
        }
      }
      return points; 
    }  

    // calculate new buckets
    T newcenter[8][3];
    T sizeNew;

    sizeNew = _size / 2.0;

    for (int i = 0; i < 8; i++) {
      childcenter(_center, newcenter[i], _size, i);
    }

    countPointsAndQueue(splitPoints, newcenter, sizeNew, node);
    return 0;
  }
/*
  pointrep *branch( bitoct<T> &node, list<T*> &points, T _center[3], T _size) {
    // if bucket is too small stop building tree
    // -----------------------------------------
    if ((_size <= voxelSize)) {
      // copy points
      pointrep *points = new pointrep[POINTDIM*splitPoints.size() + 1];
      points[0].length = splitPoints.size();
      int i = 1;
      for (list<T *>::iterator itr = splitPoints.begin(); 
          itr != splitPoints.end(); itr++) {
        for (unsigned int iterator = 0; iterator < POINTDIM; iterator++) {
          points[i++].v = (*itr)[iterator];
        }
      }
      return points; 
    }  

    // calculate new buckets
    T newcenter[8][3];
    T sizeNew;

    sizeNew = _size / 2.0;

    for (int i = 0; i < 8; i++) {
      childcenter(_center, newcenter[i], _size, i);
    }

    countPointsAndQueue(splitPoints, newcenter, sizeNew, node);
    return 0;
  }*/
 /* 
  void countPointsAndQueue(list<T*> &i_points, T center[8][3], T size, bitoct<T> &parent) {
    list<T*> points[8];
    int n_children = 0;


    for (int j = 0; j < 7; j++) {
      for ( list<T *>::iterator itr = i_points.begin(); itr != i_points.end();) {
        if (fabs((*itr)[0] - center[j][0]) <= size) {
          if (fabs((*itr)[1] - center[j][1]) <= size) {
            if (fabs((*itr)[2] - center[j][2]) <= size) {
              points[j].push_back(*itr);
              itr = i_points.erase(itr);
              continue;
            }
          }
        }
        itr++;
      }
    }
    points[7] = i_points;
    i_points.clear();

    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        parent.valid = ( 1 << j ) | parent.valid;
        ++n_children;
      }
    }

    // create children
    bitunion<T> *children = new bitunion<T>[n_children];
    bitoct<T>::link(parent, children);

    int count = 0;
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        pointrep *c = branch(children[count].node, points[j], center[j], size);  // leaf node
        if (c) { 
          children[count].points = c; // set this child to set of points
          parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
        }
        points[j].clear();
        ++count;
      }
    }
  }

  void countPointsAndQueue(deque<T*> &i_points, T center[8][3], T size, bitoct<T> &parent) {
    deque<T*> points[8];
    int n_children = 0;


    for (int j = 0; j < 8; j++) {
      for ( deque<T *>::iterator itr = i_points.begin(); itr != i_points.end(); itr++) {
        if (fabs((*itr)[0] - center[j][0]) <= size) {
          if (fabs((*itr)[1] - center[j][1]) <= size) {
            if (fabs((*itr)[2] - center[j][2]) <= size) {
              points[j].push_back(*itr);
              continue;
            }
          }
        }
      }
    }
    i_points.clear();
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        parent.valid = ( 1 << j ) | parent.valid;
        ++n_children;
      }
    }

    // create children
    bitunion<T> *children = new bitunion<T>[n_children];
    bitoct<T>::link(parent, children);

    int count = 0;
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        pointrep *c = branch(children[count].node, points[j], center[j], size);  // leaf node
        if (c) { 
          children[count].points = c; // set this child to deque of points
          parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
        }
        points[j].clear();
        ++count;
      }
    }
  }*/

  void countPointsAndQueue(vector<T*> &i_points, T center[8][3], T size, bitoct<T> &parent) {
    vector<T*> points[8];
    int n_children = 0;

#ifdef _OPENMP 
#pragma omp parallel for schedule(dynamic) 
#endif
    for (int j = 0; j < 8; j++) {
      for (typename vector<T *>::iterator itr = i_points.begin(); itr != i_points.end(); itr++) {
        if (fabs((*itr)[0] - center[j][0]) <= size) {
          if (fabs((*itr)[1] - center[j][1]) <= size) {
            if (fabs((*itr)[2] - center[j][2]) <= size) {
              points[j].push_back(*itr);
              continue;
            }
          }
        }
      }
    }

    i_points.clear();
    vector<T*>().swap(i_points);
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        parent.valid = ( 1 << j ) | parent.valid;
        ++n_children;
      }
    }
    // create children
    bitunion<T> *children = new bitunion<T>[n_children];
    bitoct<T>::link(parent, children);

    int count = 0;
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        pointrep *c = branch(children[count].node, points[j], center[j], size);  // leaf node
        if (c) { 
          children[count].points = c; // set this child to deque of points
          parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
        }
        points[j].clear();
        vector<T*>().swap(points[j]);
        ++count;
      }
    }
  }

  void countPointsAndQueue(T **pts, int n,  T center[8][3], T size, bitoct<T> &parent) {
    vector<T*> points[8];
    int n_children = 0;
#ifdef _OPENMP 
#pragma omp parallel for schedule(dynamic) 
#endif
    for (int j = 0; j < 8; j++) {
      for (int i = 0; i < n; i++) {
        if (fabs(pts[i][0] - center[j][0]) <= size) {
          if (fabs(pts[i][1] - center[j][1]) <= size) {
            if (fabs(pts[i][2] - center[j][2]) <= size) {
              points[j].push_back( pts[i] );
            }
          }
        } 
      }
    }
    for (int j = 0; j < 8; j++) {
      // if non-empty set valid flag for this child
      if (!points[j].empty()) {
        parent.valid = ( 1 << j ) | parent.valid;
        ++n_children;
      }
    }

    // create children
    bitunion<T> *children = new bitunion<T>[n_children];
    bitoct<T>::link(parent, children);
    int count = 0;
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        pointrep *c = branch(children[count].node, points[j], center[j], size);  // leaf node
        if (c) { 
          children[count].points = c; // set this child to vector of points
          parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
        }
        points[j].clear();
        vector<T*>().swap(points[j]);
        ++count;
      }
    }
  }


  void childcenter(T *pcenter, T *ccenter, T size, int i) {
    switch (i) {
      case 0:
        ccenter[0] = pcenter[0] - size / 2.0;
        ccenter[1] = pcenter[1] - size / 2.0;
        ccenter[2] = pcenter[2] - size / 2.0;
        break;
      case 1:
        ccenter[0] = pcenter[0] + size / 2.0;
        ccenter[1] = pcenter[1] - size / 2.0;
        ccenter[2] = pcenter[2] - size / 2.0;
        break;
      case 2:
        ccenter[0] = pcenter[0] - size / 2.0;
        ccenter[1] = pcenter[1] + size / 2.0;
        ccenter[2] = pcenter[2] - size / 2.0;
        break;
      case 3:
        ccenter[0] = pcenter[0] - size / 2.0;
        ccenter[1] = pcenter[1] - size / 2.0;
        ccenter[2] = pcenter[2] + size / 2.0;
        break;
      case 4:
        ccenter[0] = pcenter[0] + size / 2.0;
        ccenter[1] = pcenter[1] + size / 2.0;
        ccenter[2] = pcenter[2] - size / 2.0;
        break;
      case 5:
        ccenter[0] = pcenter[0] + size / 2.0;
        ccenter[1] = pcenter[1] - size / 2.0;
        ccenter[2] = pcenter[2] + size / 2.0;
        break;
      case 6:
        ccenter[0] = pcenter[0] - size / 2.0;
        ccenter[1] = pcenter[1] + size / 2.0;
        ccenter[2] = pcenter[2] + size / 2.0;
        break;
      case 7:
        ccenter[0] = pcenter[0] + size / 2.0;
        ccenter[1] = pcenter[1] + size / 2.0;
        ccenter[2] = pcenter[2] + size / 2.0;
        break;
      default:
        break;
    }
  }


  /**
   * the root of the octree 
   */
  bitoct<T>* root;

  
  /**
   * storing the center
   */
  T center[3];
  
  /**
   * storing the dimension
   */
  T size;

  /**
   * storing the voxel size
   */
  T voxelSize;

  unsigned int POINTDIM;

};

#endif
