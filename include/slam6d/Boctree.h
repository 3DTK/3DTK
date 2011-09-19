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

#include "limits.h"
#include "globals.icc"
#include "point_type.h"

#if __GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4)
  #define POPCOUNT(mask) __builtin_popcount(mask)
#else
  #define POPCOUNT(mask) _my_popcount_3(mask)
#endif

#include "slam6d/nnparams.h"
#include "slam6d/searchTree.h"
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
class bitoct{
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
  template <class T>
  static inline void link(bitoct &parent, bitunion<T> *child) {
    parent.child_pointer = (long)((char*)child - (char*)&parent);
  }

  /**
   * Returns the children of this node (given as parent).
   */
  template <class T>
  static inline void getChildren(bitoct &parent, bitunion<T>* &children) {
    children = (bitunion<T>*)((char*)&parent + parent.child_pointer);
  }

  template <class T>
  inline bitunion<T>* getChild(unsigned char index) {
    bitunion<T> *children = (bitunion<T>*)((char*)this + this->child_pointer);
    for (unsigned char i = 0; i < index; i++) {
      if (  ( 1 << i ) & valid ) {   // if ith node exists
        children++;
      }
    }
    return children;
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
  bitoct node;

  bitunion(pointrep *p) : points(p) {};
  bitunion(bitoct b) : node(b) {};
  bitunion() : points(0) {
    node.child_pointer = 0;
    node.valid = 0;
    node.leaf = 0;
  };           // needed for new []
  
  inline T* getPoints() {
    //return (T*)&(this->points[1].v);
    return &(this->points[1].v);
  }
  inline unsigned int getLength() {
    return this->points[0].length;
  }
  
  inline bitunion<T>* getChild(unsigned char index) {
    bitunion<T> *children = (bitunion<T>*)((char*)this + this->node.child_pointer);
    for (unsigned char i = 0; i < index; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        children++;
      }
    }
    return children;
  }
  
  inline bool isValid(unsigned char index) {
    return  (  ( 1 << index ) & node.valid );
  }
 /*
  inline pointrep* getChild(unsigned char index) {
    bitunion<T> *children = (bitunion<T>*)((char*)this + this->node.child_pointer);
    return children[index].points; 
  }*/
  
  inline bool childIsLeaf(unsigned char index) {
    return (  ( 1 << index ) & node.leaf ); // if ith node is leaf get center
  }

  
};


/**
 * @brief Octree
 * 
 * A cubic bounding box is calculated
 * from the given 3D points. Then it
 * is recusivly subdivided into smaller
 * subboxes
 */
template <class T> class BOctTree : public SearchTree {

public:
  BOctTree() {
  }

  template <class P>
  BOctTree(P * const* pts, int n, T voxelSize, PointType _pointtype = PointType(), bool _earlystop = false ) : pointtype(_pointtype), earlystop(_earlystop) {
    this->voxelSize = voxelSize;

    this->POINTDIM = pointtype.getPointDim();

    mins = new T[POINTDIM];
    maxs = new T[POINTDIM];

    // initialising
    for (unsigned int i = 0; i < POINTDIM; i++) { 
      mins[i] = pts[0][i]; 
      maxs[i] = pts[0][i];
    }

    for (unsigned int i = 0; i < POINTDIM; i++) { 
      for (int j = 1; j < n; j++) {
        mins[i] = min(mins[i], (T)pts[j][i]);
        maxs[i] = max(maxs[i], (T)pts[j][i]);
      }
    }

    center[0] = 0.5 * (mins[0] + maxs[0]);
    center[1] = 0.5 * (mins[1] + maxs[1]);
    center[2] = 0.5 * (mins[2] + maxs[2]);
    size = max(max(0.5 * (maxs[0] - mins[0]), 0.5 * (maxs[1] - mins[1])), 0.5 * (maxs[2] - mins[2]));
    size += 1.0; // for numerical reasons we increase size 

    // calculate new buckets
    T newcenter[8][3];
    T sizeNew = size / 2.0;

    for (unsigned char i = 0; i < 8; i++) {
      childcenter(center, newcenter[i], size, i);
    }
    // set up values
    uroot = new bitunion<T>();
    root = &uroot->node;

    countPointsAndQueue(pts, n, newcenter, sizeNew, *root, center);
    init();
  }

  BOctTree(std::string filename) {
    deserialize(filename); 
    init();
  }

  template <class P>
  BOctTree(vector<P *> &pts, T voxelSize, PointType _pointtype = PointType(), bool _earlystop = false) : earlystop(_earlystop) {
    this->voxelSize = voxelSize;

    this->POINTDIM = pointtype.getPointDim();

    mins = new T[POINTDIM];
    maxs = new T[POINTDIM];

    // initialising
    for (unsigned int i = 0; i < POINTDIM; i++) { 
      mins[i] = pts[0][i]; 
      maxs[i] = pts[0][i];
    }

    for (unsigned int i = 0; i < POINTDIM; i++) { 
      for (unsigned int j = 1; j < pts.size(); j++) {
        mins[i] = min(mins[i], pts[j][i]);
        maxs[i] = max(maxs[i], pts[j][i]);
      }
    }

    center[0] = 0.5 * (mins[0] + maxs[0]);
    center[1] = 0.5 * (mins[1] + maxs[1]);
    center[2] = 0.5 * (mins[2] + maxs[2]);
    size = max(max(0.5 * (maxs[0] - mins[0]), 0.5 * (maxs[1] - mins[1])), 0.5 * (maxs[2] - mins[2]));
    
    size += 1.0; // for numerical reasons we increase size 

    // calculate new buckets
    T newcenter[8][3];
    T sizeNew = size / 2.0;

    for (unsigned char i = 0; i < 8; i++) {
      childcenter(center, newcenter[i], size, i);
    }
    // set up values
    uroot = new bitunion<T>();
    root = &uroot->node;

    countPointsAndQueue(pts, newcenter, sizeNew, *root, center);
  }

  virtual ~BOctTree(){
    deletetNodes(*root);
    delete root;

    delete[] mins;
    delete[] maxs;
  } 

  void GetOctTreeCenter(vector<T*>&c) { GetOctTreeCenter(c, *root, center, size); }
  void GetOctTreeRandom(vector<T*>&c) { GetOctTreeRandom(c, *root); }
  void GetOctTreeRandom(vector<T*>&c, unsigned int ptspervoxel) { GetOctTreeRandom(c, ptspervoxel, *root); }
  void AllPoints(vector<T *> &vp) { AllPoints(*BOctTree<T>::root, vp); }

  long countNodes() { return 1 + countNodes(*root); } // computes number of inner nodes
  long countLeaves() { return countLeaves(*root); }   // computes number of leaves + points
  long countOctLeaves() { return countOctLeaves(*root); } // computes number of leaves

  void  init() {
    // compute maximal depth as well as the size of the smalles leaf
    real_voxelSize = size;
    max_depth = 1;
    while (real_voxelSize > voxelSize) {
      real_voxelSize = real_voxelSize/2.0;
      max_depth++;
    }
    
    child_bit_depth = new unsigned int[max_depth];
    child_bit_depth_inv = new unsigned int[max_depth];

    for(int d=0; d < max_depth; d++) {
      child_bit_depth[d] = 1 << (max_depth - d - 1);
      child_bit_depth_inv[d] = ~child_bit_depth[d];
    }
    
    mult = 1.0/real_voxelSize;
    add[0] = -center[0] + size;
    add[1] = -center[1] + size;
    add[2] = -center[2] + size;

    for (unsigned char mask = 0; mask < 256; mask++) {
      for (unsigned char index = 0; index < 8; index++) {
        char c = 0;
        char *mimap = this->imap[index];  // maps area index to preference
        for (unsigned char i = 0; i < 8; i++) {
          if (  ( 1 << i ) & mask ) {   // if ith node exists
            sequence2ci[index][mask][ mimap[i] ] = c++;
          } else {
            sequence2ci[index][mask][ mimap[i] ] = -1;
          }
        }
      }
      if (mask == UCHAR_MAX) break;
    }
    largest_index = child_bit_depth[0] * 2 -1;
  }

  void deserialize(std::string filename ) {
    char buffer[sizeof(T) * 20];
    T *p = reinterpret_cast<T*>(buffer);

    std::ifstream file;
    file.open (filename.c_str(), std::ios::in | std::ios::binary);

    // read magic bits
    file.read(buffer, 2);
    if ( buffer[0] != 'X' || buffer[1] != 'T') {
      std::cerr << "Not an octree file!!" << endl;
      file.close();
      return;
    }

    // read header
    pointtype = PointType::deserialize(file);

    file.read(buffer, 5 * sizeof(T));
    voxelSize = p[0];
    center[0] = p[1];
    center[1] = p[2];
    center[2] = p[3];
    size = p[4];

    file.read(buffer, sizeof(int));
    int *ip = reinterpret_cast<int*>(buffer);
    POINTDIM = *ip;

    mins = new T[POINTDIM];
    maxs = new T[POINTDIM];

    file.read(reinterpret_cast<char*>(mins), POINTDIM * sizeof(T));
    file.read(reinterpret_cast<char*>(maxs), POINTDIM * sizeof(T));

    // read root node
    root = new bitoct();
    uroot = new bitunion<T>();
    deserialize(file, *root);
    file.close();
  }
  
  
  static void deserialize(std::string filename, vector<Point> &points ) {
    char buffer[sizeof(T) * 20];

    std::ifstream file;
    file.open (filename.c_str(), std::ios::in | std::ios::binary);

    // read magic bits
    file.read(buffer, 2);
    if ( buffer[0] != 'X' || buffer[1] != 'T') {
      std::cerr << "Not an octree file!!" << endl;
      file.close();
      return;
    }

    // read header
    PointType pointtype = PointType::deserialize(file);

    file.read(buffer, 5 * sizeof(T)); // read over voxelsize, center and size
    file.read(buffer, sizeof(int));

    int *ip = reinterpret_cast<int*>(buffer);
    unsigned int POINTDIM = *ip;

    file.read(buffer, POINTDIM * sizeof(T));
    file.read(buffer, POINTDIM * sizeof(T));

    // read root node
    deserialize(file, points, pointtype);
    file.close();
  }

  void serialize(std::string filename) {
    char buffer[sizeof(T) * 20];
    T *p = reinterpret_cast<T*>(buffer);

    std::ofstream file;
    file.open (filename.c_str(), std::ios::out | std::ios::binary);

    // write magic bits
    buffer[0] = 'X';
    buffer[1] = 'T';
    file.write(buffer, 2);

    // write header
    pointtype.serialize(file);

    p[0] = voxelSize;
    p[1] = center[0]; 
    p[2] = center[1]; 
    p[3] = center[2];
    p[4] = size;

    int *ip = reinterpret_cast<int*>(&(buffer[5 * sizeof(T)]));
    *ip = POINTDIM;

    file.write(buffer, 5 * sizeof(T) + sizeof(int));


    for (unsigned int i = 0; i < POINTDIM; i++) {
      p[i] = mins[i];
    }
    for (unsigned int i = 0; i < POINTDIM; i++) {
      p[i+POINTDIM] = maxs[i];
    }

    file.write(buffer, 2*POINTDIM * sizeof(T));

    // write root node
    serialize(file, *root);

    file.close();
  }

  static PointType readType(std::string filename ) {
    char buffer[sizeof(T) * 20];

    std::ifstream file;
    file.open (filename.c_str(), std::ios::in | std::ios::binary);

    // read magic bits
    file.read(buffer, 2);
    if ( buffer[0] != 'X' || buffer[1] != 'T') {
      std::cerr << "Not an octree file!!" << endl;
      file.close();
      return PointType();
    }

    // read header
    PointType pointtype = PointType::deserialize(file);

    file.close();

    return pointtype;
  }
  
  void getCenter(double _center[3]) const {
    _center[0] = center[0];
    _center[1] = center[1];
    _center[2] = center[2];
  }

protected:
  
  
  void AllPoints( bitoct &node, vector<T*> &vp) {
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
          pointrep *points = children->points;
          unsigned int length = points[0].length;
          T *point = &(points[1].v);  // first point
          for(unsigned int iterator = 0; iterator < length; iterator++ ) {
            //T *p = new T[BOctTree<T>::POINTDIM];
//            T *p = new T[3];
//            p[0] = point[0]; p[1] = point[1]; p[2] = point[2];
            T *p = new T[BOctTree<T>::POINTDIM];
            for (unsigned int k = 0; k < BOctTree<T>::POINTDIM; k++)
              p[k] = point[k];

            vp.push_back(p);

            //glVertex3f( point[0], point[1], point[2]);
            point+=BOctTree<T>::POINTDIM;
          }
        } else { // recurse
          AllPoints( children->node, vp);
        }
        ++children; // next child
      }
    }
  }
  
  
  static void deserialize(std::ifstream &f, vector<Point> &vpoints, PointType &pointtype) {
    char buffer[2];
    pointrep *point = new pointrep[pointtype.getPointDim()];
    f.read(buffer, 2);
    bitoct node;
    node.valid = buffer[0];
    node.leaf = buffer[1];

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf read points 
          pointrep first;
          f.read(reinterpret_cast<char*>(&first), sizeof(pointrep));
          unsigned int length = first.length;  // read first element, which is the length
          for (unsigned int k = 0; k < length; k++) {
            f.read(reinterpret_cast<char*>(point), sizeof(pointrep) * pointtype.getPointDim()); // read the points
            vpoints.push_back( pointtype.createPoint( &(point->v ) ) );
          }
        } else {  // write child 
          deserialize(f, vpoints, pointtype);
        }
      }
    }
	delete [] point;
  }

  void deserialize(std::ifstream &f, bitoct &node) {
    char buffer[2];
    f.read(buffer, 2);
    node.valid = buffer[0];
    node.leaf = buffer[1];

    unsigned short n_children = POPCOUNT(node.valid);

    // create children
    bitunion<T> *children = new bitunion<T>[n_children];
    bitoct::link(node, children);

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

  void serialize(std::ofstream &of, bitoct &node) {
    char buffer[2];
    buffer[0] = node.valid;
    buffer[1] = node.leaf;
    of.write(buffer, 2);


    // write children
    bitunion<T> *children;
    bitoct::getChildren(node, children);
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


  void GetOctTreeCenter(vector<T*>&c, bitoct &node, T *center, T size) {
    T ccenter[3];
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    for (unsigned char i = 0; i < 8; i++) {
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

  void GetOctTreeRandom(vector<T*>&c, bitoct &node) {
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
          pointrep *points = children->points;
          // new version to ignore leaves with less than 3 points
          /* 
          if(points[0].length > 2) { 
            for(int tmp = 0; tmp < points[0].length; tmp++) {
              T *point = &(points[POINTDIM*tmp+1].v);
              c.push_back(point);
            }
          }
          */  
          //old version
          
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
  
  

  void GetOctTreeRandom(vector<T*>&c, unsigned int ptspervoxel, bitoct &node) {
    bitunion<T> *children;
    bitoct::getChildren(node, children);

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

  long countNodes(bitoct &node) {
    long result = 0;
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
          //++result;
        } else { // recurse
          result += countNodes(children->node) + 1;
        }
        ++children; // next child
      }
    }
    return result;
  }

  long countLeaves(bitoct &node) {
    long result = 0;
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
          pointrep *points = children->points;
          long nrpts = points[0].length;
          result += POINTDIM*nrpts ;
        } else { // recurse
          result += countLeaves(children->node);
        }
        ++children; // next child
      }
    }
    return result;
  }
  long countOctLeaves(bitoct &node) {
    long result = 0;
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
          result ++;
        } else { // recurse
          result += countTrueLeaves(children->node);
        }
        ++children; // next child
      }
    }
    return result;
  }


  void deletetNodes(bitoct &node) {
    bitunion<T> *children;
    bitoct::getChildren(node, children);
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
      bitoct::getChildren(node, children);
      delete[] children;
    }
  }

  template <class P>
  void* branch( bitoct &node, vector<P*> &splitPoints, T _center[3], T _size) {
    // if bucket is too small stop building tree
    // -----------------------------------------
    if ((_size <= voxelSize) || (earlystop && splitPoints.size() <= 10) ) {
      // copy points
      pointrep *points = new pointrep[POINTDIM*splitPoints.size() + 1];
      points[0].length = splitPoints.size();
      int i = 1;
      for (typename vector<P *>::iterator itr = splitPoints.begin(); 
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

    for (unsigned char i = 0; i < 8; i++) {
      childcenter(_center, newcenter[i], _size, i);
    }

    countPointsAndQueue(splitPoints, newcenter, sizeNew, node, _center);
    return 0;
  }

  template <class P>
  void countPointsAndQueue(vector<P*> &i_points, T center[8][3], T size, bitoct &parent, T *pcenter) {
    vector<P*> points[8];
    int n_children = 0;
    for (typename vector<P *>::iterator itr = i_points.begin(); itr != i_points.end(); itr++) {
      points[childIndex<P>(pcenter, *itr)].push_back( *itr );
    }

    i_points.clear();
    vector<P*>().swap(i_points);
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        parent.valid = ( 1 << j ) | parent.valid;
        ++n_children;
      }
    }
    // create children
    bitunion<T> *children = new bitunion<T>[n_children];
    bitoct::link(parent, children);

    int count = 0;
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        pointrep *c = (pointrep*)branch(children[count].node, points[j], center[j], size);  // leaf node
        if (c) { 
          children[count].points = c; // set this child to deque of points
          parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
        }
        points[j].clear();
        vector<P*>().swap(points[j]);
        ++count;
      }
    }
  }

  template <class P>
  void countPointsAndQueue(P * const* pts, int n,  T center[8][3], T size, bitoct &parent, T pcenter[3]) {
    vector<const P*> points[8];
    int n_children = 0;
      for (int i = 0; i < n; i++) {
              points[childIndex<P>(pcenter, pts[i])].push_back( pts[i] );
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
    bitoct::link(parent, children);
    int count = 0;
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        pointrep *c = (pointrep*)branch(children[count].node, points[j], center[j], size);  // leaf node
        if (c) { 
          children[count].points = c; // set this child to vector of points
          parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
        }
        points[j].clear();
        vector<const P*>().swap(points[j]);
        ++count;
      }
    }
  }


  void getByIndex(T *point, T *&points, unsigned int &length) {
    unsigned int x,y,z;
    x = (point[0] + add[0]) * mult;
    y = (point[1] + add[1]) * mult;
    z = (point[2] + add[2]) * mult;
    
    bitunion<T> *node = uroot; 
    unsigned char child_index;
    unsigned int child_bit;
    unsigned int depth = 0;

    while (true) {
      child_bit = child_bit_depth[depth];
      child_index = ((x & child_bit )!=0)  | (((y & child_bit )!=0 )<< 1) | (((z & child_bit )!=0) << 2);
      if (node->childIsLeaf(child_index) ) {
        node = node->getChild(child_index);
        points = node->getPoints();
        length = node->getLength();
        return;
      } else {
        node = node->getChild(child_index);
      }
      depth++;
    }
  }
  NNParams params[100];

  void childcenter(T *pcenter, T *ccenter, T size, unsigned char i) {
    switch (i) {
      case 0:  // 000
        ccenter[0] = pcenter[0] - size / 2.0;
        ccenter[1] = pcenter[1] - size / 2.0;
        ccenter[2] = pcenter[2] - size / 2.0;
        break;
      case 1:  // 001
        ccenter[0] = pcenter[0] + size / 2.0;
        ccenter[1] = pcenter[1] - size / 2.0;
        ccenter[2] = pcenter[2] - size / 2.0;
        break;
      case 2:  // 010
        ccenter[0] = pcenter[0] - size / 2.0;
        ccenter[1] = pcenter[1] + size / 2.0;
        ccenter[2] = pcenter[2] - size / 2.0;
        break;
      case 3:  // 011
        ccenter[0] = pcenter[0] + size / 2.0;
        ccenter[1] = pcenter[1] + size / 2.0;
        ccenter[2] = pcenter[2] - size / 2.0;
        break;
      case 4:  // 100
        ccenter[0] = pcenter[0] - size / 2.0;
        ccenter[1] = pcenter[1] - size / 2.0;
        ccenter[2] = pcenter[2] + size / 2.0;
        break;
      case 5:  // 101
        ccenter[0] = pcenter[0] + size / 2.0;
        ccenter[1] = pcenter[1] - size / 2.0;
        ccenter[2] = pcenter[2] + size / 2.0;
        break;
      case 6:  // 110
        ccenter[0] = pcenter[0] - size / 2.0;
        ccenter[1] = pcenter[1] + size / 2.0;
        ccenter[2] = pcenter[2] + size / 2.0;
        break;
      case 7:  // 111
        ccenter[0] = pcenter[0] + size / 2.0;
        ccenter[1] = pcenter[1] + size / 2.0;
        ccenter[2] = pcenter[2] + size / 2.0;
        break;
      default:
        break;
    }
  }
void childcenter(int x, int y, int z, int &cx, int &cy, int &cz, char i, int size) {
  switch (i) {
    case 0:  // 000
      cx = x - size ;
      cy = y - size ;
      cz = z - size ;
      break;
    case 1:  // 001
      cx = x + size ;
      cy = y - size ;
      cz = z - size ;
      break;
    case 2:  // 010
      cx = x - size ;
      cy = y + size ;
      cz = z - size ;
      break;
    case 3:  // 011
      cx = x + size ;
      cy = y + size ;
      cz = z - size ;
      break;
    case 4:  // 100
      cx = x - size ;
      cy = y - size ;
      cz = z + size ;
      break;
    case 5:  // 101
      cx = x + size ;
      cy = y - size ;
      cz = z + size ;
      break;
    case 6:  // 110
      cx = x - size ;
      cy = y + size ;
      cz = z + size ;
      break;
    case 7:  // 111
      cx = x + size ;
      cy = y + size ;
      cz = z + size ;
      break;
    default:
      break;
  }
}
template <class P>
inline unsigned char childIndex(const T *center, const P *point) {
  return  (point[0] > center[0] ) | ((point[1] > center[1] ) << 1) | ((point[2] > center[2] ) << 2) ;
}


  /**
   * the root of the octree 
   */
  bitoct* root;
  bitunion<T>* uroot;


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
  /**
   * The real voxelsize of the leaves
   **/
  T real_voxelSize;

  /**
   * storing minimal and maximal values for all dimensions
   **/
  T *mins;
  T *maxs;

  T add[3];
  T mult;

  unsigned char max_depth;
  unsigned int *child_bit_depth; // octree only works to depth 32 with ints, should be plenty
  unsigned int *child_bit_depth_inv; // octree only works to depth 32 with ints, should be plenty

  unsigned int POINTDIM;

  PointType pointtype;

  bool earlystop;

  /**
   * Given a leaf node, this function looks for the closest point to params[threadNum].closest
   * in the list of points.
   */
  inline void findClosestInLeaf(bitunion<T> *node, int threadNum) {
    if (params[threadNum].count >= params[threadNum].max_count) return;
    params[threadNum].count++;
    T* points = node->getPoints();
    unsigned int length = node->getLength();
    for(unsigned int iterator = 0; iterator < length; iterator++ ) {
      double myd2 = Dist2(params[threadNum].p, points); 
      if (myd2 < params[threadNum].closest_d2) {
        params[threadNum].closest_d2 = myd2;
        params[threadNum].closest = points;
        if (myd2 <= 0.0001) {
          params[threadNum].closest_v = 0; // the search radius in units of voxelSize
        } else {
          params[threadNum].closest_v = sqrt(myd2) * mult + 1; // the search radius in units of voxelSize
        }
      }
      points+=BOctTree<T>::POINTDIM;
    }
  }
  


/** 
 * This function finds the closest point in the octree given a specified
 * radius. This implementation is quit complex, although it is already
 * simplified. The simplification incurs a significant loss in speed, as
 * several calculations have to be performed repeatedly and a high number of
 * unnecessary jumps are executed.
 */
  double *FindClosest(double *point, double maxdist2, int threadNum)
  {
    params[threadNum].closest = 0; // no point found currently
    params[threadNum].closest_d2 = maxdist2;
    params[threadNum].p = point;
    params[threadNum].x = (point[0] + add[0]) * mult;
    params[threadNum].y = (point[1] + add[1]) * mult;
    params[threadNum].z = (point[2] + add[2]) * mult;
    params[threadNum].closest_v = sqrt(maxdist2) * mult + 1; // the search radius in units of voxelSize
    params[threadNum].count = 0;
    params[threadNum].max_count = 10000; // stop looking after this many buckets

   
    // box within bounds in voxel coordinates
    int xmin, ymin, zmin, xmax, ymax, zmax;
    xmin = max(params[threadNum].x-params[threadNum].closest_v, 0); 
    ymin = max(params[threadNum].y-params[threadNum].closest_v, 0); 
    zmin = max(params[threadNum].z-params[threadNum].closest_v, 0);

//    int largest_index = child_bit_depth[0] * 2 -1;
    
    xmax = min(params[threadNum].x+params[threadNum].closest_v, largest_index);
    ymax = min(params[threadNum].y+params[threadNum].closest_v, largest_index);
    zmax = min(params[threadNum].z+params[threadNum].closest_v, largest_index);
    
    unsigned char depth = 0;
    unsigned int child_bit;
    unsigned int child_index_min;
    unsigned int child_index_max;

    bitunion<T> *node = uroot;

    int cx, cy, cz;
    
    child_bit = child_bit_depth[depth];
    cx = child_bit_depth[depth];
    cy = child_bit_depth[depth];
    cz = child_bit_depth[depth];

    while (true) { // find the first node where branching is required
      child_index_min = ((xmin & child_bit )!=0)  | (((ymin & child_bit )!=0 )<< 1) | (((zmin & child_bit )!=0) << 2);
      child_index_max = ((xmax & child_bit )!=0)  | (((ymax & child_bit )!=0 )<< 1) | (((zmax & child_bit )!=0) << 2);

      // if these are the same, go there
      // TODO: optimization: also traverse if only single child...
      if (child_index_min == child_index_max) {
        if (node->childIsLeaf(child_index_min) ) {  // luckily, no branching is required
          findClosestInLeaf(node->getChild(child_index_min), threadNum);
          return static_cast<double*>(params[threadNum].closest);
        } else {
          if (node->isValid(child_index_min) ) { // only descend when there is a child
            childcenter(cx,cy,cz, cx,cy,cz, child_index_min, child_bit/2 ); 
            node = node->getChild(child_index_min);
            child_bit /= 2;
          } else {  // there is no child containing the bounding box => no point is close enough
            return 0;
          }
        }
      } else {
        // if min and max are not in the same child we must branch
        break;
      }
    }
    
    // node contains all box-within-bounds cells, now begin best bin first search
    _FindClosest(threadNum, node->node, child_bit/2, cx, cy, cz);
    return static_cast<double*>(params[threadNum].closest);
  }
  
  /**
   * This is the heavy duty search function doing most of the (theoretically unneccesary) work. The tree is recursively searched.
   * Depending on which of the 8 child-voxels is closer to the query point, the children are examined in a special order.
   * This order is defined in map, imap is its inverse and sequence2ci is a speedup structure for faster access to the child indices. 
   */
  void _FindClosest(int threadNum, bitoct &node, int size, int x, int y, int z)
  {
    // Recursive case
   
    // compute which child is closest to the query point
    unsigned char child_index =  ((params[threadNum].x - x) >= 0) | 
                                (((params[threadNum].y - y) >= 0) << 1) | 
                                (((params[threadNum].z - z) >= 0) << 2);
    
    char *seq2ci = sequence2ci[child_index][node.valid];  // maps preference to index in children array
    char *mmap = this->map[child_index];  // maps preference to area index 

    bitunion<T> *children;
    bitoct::getChildren(node, children);
    int cx, cy, cz;
    cx = cy = cz = 0; // just to shut up the compiler warnings
    for (unsigned char i = 0; i < 8; i++) { // in order of preference
      child_index = mmap[i]; // the area index of the node 
      if (  ( 1 << child_index ) & node.valid ) {   // if ith node exists
        childcenter(x,y,z, cx,cy,cz, child_index, size); 
        if ( params[threadNum].closest_v == 0 ||  max(max(abs( cx - params[threadNum].x ), 
                 abs( cy - params[threadNum].y )),
                 abs( cz - params[threadNum].z )) - size
        >= params[threadNum].closest_v ) { 
          break;
        }
        // find the closest point in leaf seq2ci[i] 
        if (  ( 1 << child_index ) & node.leaf ) {   // if ith node is leaf
          findClosestInLeaf( &children[seq2ci[i]], threadNum);
        } else { // recurse
          _FindClosest(threadNum, children[seq2ci[i]].node, size/2, cx, cy, cz);
        }
      }
    }
  }


  /** 
   * This function shows the possible speedup that can be gained by using the
   * octree for nearest neighbour search, if a more sophisticated
   * implementation were given. Here, only the bucket in which the query point
   * falls is looked up. If doing the same thing in the kd-tree search, this
   * function is about 3-5 times as fast
   */
  double *FindClosestInBucket(double *point, double maxdist2, int threadNum) {
    params[threadNum].closest = 0;
    params[threadNum].closest_d2 = maxdist2;
    params[threadNum].p = point;
    unsigned int x,y,z;
    x = (point[0] + add[0]) * mult;
    y = (point[1] + add[1]) * mult;
    z = (point[2] + add[2]) * mult;
    T * points;
    unsigned int length;

    bitunion<T> *node = uroot; 
    unsigned char child_index;

    unsigned int  child_bit = child_bit_depth[0];

    while (true) {
      child_index = ((x & child_bit )!=0)  | (((y & child_bit )!=0 )<< 1) | (((z & child_bit )!=0) << 2);
      if (node->childIsLeaf(child_index) ) {
        node = node->getChild(child_index);
        points = node->getPoints();
        length = node->getLength();
        
        for(unsigned int iterator = 0; iterator < length; iterator++ ) {
          double myd2 = Dist2(params[threadNum].p, points); 
          if (myd2 < params[threadNum].closest_d2) {
            params[threadNum].closest_d2 = myd2;
            params[threadNum].closest = points;
          }
          points+=BOctTree<T>::POINTDIM;
        }
        return static_cast<double*>(params[threadNum].closest);
      } else {
        if (node->isValid(child_index) ) {
          node = node->getChild(child_index);
        } else {
          return 0;
        }
      }
      child_bit >>= 1;
    }
    return static_cast<double*>(params[threadNum].closest);
  }

static char map[8][8]; 
static char imap[8][8]; 
static char sequence2ci[8][256][8];  // maps preference to index in children array for every valid_mask and every case


int largest_index;
};


template <class T>
char BOctTree<T>::sequence2ci[8][256][8] = {};

template <class T>
char BOctTree<T>::map[8][8] = { 
        {0, 1, 2, 4, 3, 5, 6, 7 },
        {1, 0, 3, 5, 2, 4, 6, 7 },
        {2, 0, 3, 6, 1, 4, 5, 7 },
        {3, 1, 2, 7, 0, 5, 4, 6 },
        {4, 5, 6, 0, 7, 1, 2, 3 },
        {5, 4, 7, 1, 6, 0, 3, 2 },
        {6, 4, 7, 2, 5, 0, 3, 1 },
        {7, 5, 6, 3, 4, 1, 2, 0 } };
template <class T>
char BOctTree<T>::imap[8][8] = {
        {0, 1, 2, 4, 3, 5, 6, 7 },
        {1, 0, 4, 2, 5, 3, 6, 7 },
        {1, 4, 0, 2, 5, 6, 3, 7 },
        {4, 1, 2, 0, 6, 5, 7, 3 },
        {3, 5, 6, 7, 0, 1, 2, 4 },
        {5, 3, 7, 6, 1, 0, 4, 2 },
        {5, 7, 3, 6, 1, 4, 0, 2 },
        {7, 5, 6, 3, 4, 1, 2, 0 } };
#endif
