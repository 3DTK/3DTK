/**
 * @file 
 * @brief Efficient representation of an octree
 * @author Jan Elsberg. Automation Group, Jacobs University Bremen gGmbH, Germany. 
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef COMPACTOCTREE_H
#define COMPACTOCTREE_H

#include <stdio.h>
#include <float.h>

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

#include "slam6d/globals.icc"
#include "slam6d/point_type.h"

#include "slam6d/Boctree.h"
#include "show/colormanager.h"
#include "show/colordisplay.h"
#include "show/viewcull.h"
#include "show/scancolormanager.h"

typedef short int tshort;
typedef unsigned int lint;
//typedef unsigned char shortpointrep; 
typedef short int shortpointrep; 

class ScanColorManager;
// forward declaration
template <class T> union cbitunion;
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
class cbitoct{
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
  static inline void link(cbitoct &parent, cbitunion<T> *child) {
    parent.child_pointer = (long)((char*)child - (char*)&parent);
  }

  /**
   * Returns the children of this node (given as parent).
   */
  template <class T>
  static inline void getChildren(cbitoct &parent, cbitunion<T>* &children) {
    children = (cbitunion<T>*)((char*)&parent + parent.child_pointer);
  }
 

};

class cbitp{
  public:

#ifdef _MSC_VER
  unsigned shortpointrep pointer        : 40;
  unsigned int length                   : 24;
#else
  signed long pointer                   : 40;
  unsigned int length                   : 24;
#endif
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
template <class T> union cbitunion {
  cbitp points;
  cbitoct node;

  cbitunion(cbitp p) : points(p) {};
  cbitunion(cbitoct b) : node(b) {};
  cbitunion()  {
    node.child_pointer = 0;
    node.valid = 0;
    node.leaf = 0;
  };           // needed for new []
  
  inline shortpointrep* getPoints() {
    return (shortpointrep*)((char*)this + this->points.pointer);
  }
  inline unsigned int getLength() {
    return this->points.length;
  }
  /**
   * sets the child pointer of parent so it points to child 
   */
  inline void linkPoints(shortpointrep *child, unsigned int l) {
    this->points.length  = l;  // do this first in case of overflow
    this->points.pointer = (long)((char*)child - (char*)this);
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
class ScanColorManager;

class compactTree : public colordisplay {

public:

  template <class P>
  compactTree(P * const* pts, int n, double voxelSize, PointType _pointtype = PointType(), ScanColorManager *scm=0 ); 

  template <class P>
  compactTree(vector<P *> &pts, double voxelSize, PointType _pointtype = PointType());

  virtual ~compactTree();

  inline void GetOctTreeCenter(vector<double*>&c);
  inline void AllPoints(vector<double *> &vp);

  inline long countNodes();
  inline long countLeaves(); 
  void setColorManager(ColorManager *_cm);
  void displayOctTreeCulled(long targetpts);
  void displayOctTreeAllCulled();
  void displayOctTree(double minsize = FLT_MAX);
  template <class T>
  void selectRay(vector<T *> &points);
  template <class T>
  void selectRay(T * &point);

  unsigned long maxTargetPoints();


protected:
  
  
  void AllPoints( cbitoct &node, vector<double*> &vp, double center[3], double size);

  void GetOctTreeCenter(vector<double*>&c, cbitoct &node, double *center, double size);
  
  long countNodes(cbitoct &node); 

  long countLeaves(cbitoct &node); 

  void deletetNodes(cbitoct &node);

  template <class P>
  bool branch(union cbitunion<tshort> &node, vector<P*> &splitPoints, double _center[3], double _size);

  template <class P>
  inline void countPointsAndQueue(vector<P*> &i_points, double center[8][3], double size, cbitoct &parent);

  template <class P>
  inline void countPointsAndQueue(P * const* pts, int n,  double center[8][3], double size, cbitoct &parent);


  void childcenter(double *pcenter, double *ccenter, double size, unsigned char i); 
  
  
  unsigned long maxTargetPoints( cbitoct &node );
 
  void displayOctTreeAll( cbitoct &node, double *center, double size); 

  void displayOctTreeAllCulled( cbitoct &node, double *center, double size );

  void displayOctTreeCulledLOD(long targetpts, cbitoct &node, double *center, double size ); 

  void displayOctTreeLOD(long targetpts, cbitoct &node, double *center, double size ); 
  
  
  void displayOctTreeCAllCulled( cbitoct &node, double *center, double size, double minsize ); 
  
  void displayOctTreeCAll( cbitoct &node, double *center, double size, double minsize ); 

  void showCube(double *center, double size); 

  /**
   * the root of the octree 
   */
  cbitoct* root;


  /**
   * storing the center
   */
  double center[3];

  /**
   * storing the dimension
   */
  double size;

  /**
   * storing the voxel size
   */
  double voxelSize;

  double precision;

  /**
   * storing minimal and maximal values for all dimensions
   **/
  double *mins;
  double *maxs;

  unsigned int POINTDIM;

  PointType pointtype;

  shortpointrep* createPoints(lint length);

};
  
template <class P>
  compactTree::compactTree(vector<P *> &pts, double voxelSize, PointType _pointtype) {
    this->voxelSize = voxelSize;

    this->POINTDIM = pointtype.getPointDim();

    mins = new double[POINTDIM];
    maxs = new double[POINTDIM];

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

    // calculate new buckets
    double newcenter[8][3];
    double sizeNew = size / 2.0;

    for (unsigned char i = 0; i < 8; i++) {
      childcenter(center, newcenter[i], size, i);
    }
    // set up values
    root = new cbitoct();

    countPointsAndQueue(pts, newcenter, sizeNew, *root);
  }

template <class P>
  bool compactTree::branch( union cbitunion<tshort> &node, vector<P*> &splitPoints, double _center[3], double _size) {
    // if bucket is too small stop building tree
    // -----------------------------------------
    if ((_size <= voxelSize)) {
      // copy points
      shortpointrep *points = createPoints(splitPoints.size()); 
      node.linkPoints(points, splitPoints.size());
      int i = 0;
      for (typename vector<P *>::iterator itr = splitPoints.begin(); 
          itr != splitPoints.end(); itr++) {
        //tshort r[3];
        for (unsigned int iterator = 0; iterator < 3; iterator++) {
//          points[i++].v = (*itr)[iterator];
          points[i++] = (((*itr)[iterator] - _center[iterator])/_size ) * pow(2,15) ;
        //  r[iterator] = (((*itr)[iterator] - _center[iterator])/_size ) * pow(2,15) ; 
//         cout << (*itr)[iterator] << " at c " << _center[iterator] << " converted to " << (((*itr)[iterator] - _center[iterator])/_size ) * pow(2, 15) ;
        }
        for (unsigned int iterator = 3; iterator < POINTDIM; iterator++) {
          points[i++] = (*itr)[iterator];
//          cout << "R " << (*itr)[iterator] << " " << (tshort)(*itr)[iterator]<< endl;
        }
//      cout << "SPLIT " << i 
/*        for (unsigned int iterator = 0; iterator < 3; iterator++) 
          cout << _center[iterator] << " ";
        cout << endl;
        for (unsigned int iterator = 0; iterator < 3; iterator++) 
          cout << (*itr)[iterator] << " ";
        cout << endl;
        for (unsigned int iterator = 0; iterator < 3; iterator++) 
          cout <<  r[iterator] << " "; 
        cout << endl;
        for (unsigned int iterator = 0; iterator < 3; iterator++) 
          cout << ((double)r[iterator]) * precision + _center[iterator] << " ";
        cout << endl;
        cout << endl;
        */
      }
      return true;
    }  

    // calculate new buckets
    double newcenter[8][3];
    double sizeNew;

    sizeNew = _size / 2.0;

    for (unsigned char i = 0; i < 8; i++) {
      childcenter(_center, newcenter[i], _size, i);
    }

    countPointsAndQueue(splitPoints, newcenter, sizeNew, node.node);
    return false;
  }
  
template <class P>
  void compactTree::countPointsAndQueue(vector<P*> &i_points, double center[8][3], double size, cbitoct &parent) {
    vector<P*> points[8];
    int n_children = 0;

#ifdef _OPENMP 
#pragma omp parallel for schedule(dynamic) 
#endif
    for (int j = 0; j < 8; j++) {
      for (typename vector<P *>::iterator itr = i_points.begin(); itr != i_points.end(); itr++) {
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
    vector<P*>().swap(i_points);
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        parent.valid = ( 1 << j ) | parent.valid;
        ++n_children;
      }
    }
    // create children
    cbitunion<tshort> *children = new cbitunion<tshort>[n_children];
    cbitoct::link(parent, children);

    int count = 0;
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        if ( branch(children[count], points[j], center[j], size)) {  // leaf node
          parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
        }
        points[j].clear();
        vector<P*>().swap(points[j]);
        ++count;
      }
    }
  }

  template <class P>
  void compactTree::countPointsAndQueue(P * const* pts, int n,  double center[8][3], double size, cbitoct &parent) {
    vector<const P*> points[8];
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
    cbitunion<tshort> *children = new cbitunion<tshort>[n_children];
    cbitoct::link(parent, children);
    int count = 0;
    for (int j = 0; j < 8; j++) {
      if (!points[j].empty()) {
        if ( branch(children[count], points[j], center[j], size)) {  // leaf node
          parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
        }
        points[j].clear();
        vector<const P*>().swap(points[j]);
        ++count;
      }
    }
  }

  template <class T>
  void compactTree::selectRay(T * &point) { 
    //selectRay(point, *root, center, size, FLT_MAX); 
  }
template <class P>
    compactTree::compactTree(P * const* pts, int n, double voxelSize, PointType _pointtype , ScanColorManager *scm ) : pointtype(_pointtype) {
    
    cm = 0;
    if (scm) {
      scm->registerTree(this);
      for (int i = 1; i < n; i++) {
        scm->updateRanges(pts[i]);
      }
    }
    this->voxelSize = voxelSize;

    this->POINTDIM = pointtype.getPointDim();

    mins = new double[POINTDIM];
    maxs = new double[POINTDIM];

    // initialising
    for (unsigned int i = 0; i < POINTDIM; i++) { 
      mins[i] = pts[0][i]; 
      maxs[i] = pts[0][i];
    }

    for (unsigned int i = 0; i < POINTDIM; i++) { 
      for (int j = 1; j < n; j++) {
        mins[i] = min(mins[i], (double)pts[j][i]);
        maxs[i] = max(maxs[i], (double)pts[j][i]);
      }
    }

    center[0] = 0.5 * (mins[0] + maxs[0]);
    center[1] = 0.5 * (mins[1] + maxs[1]);
    center[2] = 0.5 * (mins[2] + maxs[2]);
    size = max(max(0.5 * (maxs[0] - mins[0]), 0.5 * (maxs[1] - mins[1])), 0.5 * (maxs[2] - mins[2]));

    size += 1.0; // some buffer for numerical problems

    double vs = size;
    while (vs > voxelSize) {
      vs = vs/2.0;
    }
//    vs = vs/2.0;
//    double precision = vs/ pow(2, sizeof(tshort)*8-1);
    precision = vs/ pow(2, 15);
    // vs is the real voxelsize
    cout << "real voxelsize is " << vs << endl;
    cout << "precision is now " << precision << endl;

    // calculate new buckets
    double newcenter[8][3];
    double sizeNew = size / 2.0;

    for (unsigned char i = 0; i < 8; i++) {
      childcenter(center, newcenter[i], size, i);
    }
    // set up values
    root = new cbitoct();

    countPointsAndQueue(pts, n, newcenter, sizeNew, *root);
  } 
#endif
