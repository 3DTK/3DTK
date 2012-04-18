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
#include "slam6d/allocator.h"


#define POINTERBITS 32
//#define WITH_8BIT_POINTS
#ifdef WITH_8BIT_POINTS
typedef signed char tshort;
#define TSHORT_MAXP1 (1 << 7); 
#define TSHORT_MAX ((1 << 7) - 1); 
typedef signed char shortpointrep; 
#else
typedef short int tshort;
#define TSHORT_MAXP1 (1 << 15); 
#define TSHORT_MAX ((1 << 15) - 1); 
typedef short int shortpointrep; 
#endif



typedef unsigned int lint;

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
  __int64 child_pointer        : POINTERBITS;
  unsigned valid              :  8;
  unsigned leaf               :  8;
#else
  signed long child_pointer   : POINTERBITS;
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
  __int64 pointer        : POINTERBITS;
  unsigned int length    : 24;
#else
  signed long pointer                   : POINTERBITS;
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
  
  compactTree(std::string filename, ScanColorManager *scm = 0) {
    alloc = new PackedChunkAllocator;
    deserialize(filename); 
    if (scm) {
      scm->registerTree(this);
      scm->updateRanges(mins);
      scm->updateRanges(maxs);
    }
    setColorManager(0);
    maxtargetpoints =  maxTargetPoints(*root);
    current_lod_mode = 0;
  }

  virtual ~compactTree();

  inline void GetOctTreeCenter(vector<double*>&c);
  inline void AllPoints(vector<double *> &vp);

  inline long countNodes();
  inline long countLeaves(); 
  void setColorManager(ColorManager *_cm);
  void drawLOD(float lod);
  void draw();
  void displayOctTree(double minsize = FLT_MAX);
  template <class T>
  void selectRay(vector<T *> &points);
  template <class T>
  void selectRay(T * &point);

  inline void getCenter(double center[3]) const;

  void serialize(std::string filename);
protected:
  
  Allocator* alloc;
  
  void AllPoints( cbitoct &node, vector<double*> &vp, double center[3], double size);

  void GetOctTreeCenter(vector<double*>&c, cbitoct &node, double *center, double size);
  
  long countNodes(cbitoct &node); 

  long countLeaves(cbitoct &node); 

  void deletetNodes(cbitoct &node);

  template <class P>
  bool branch(union cbitunion<tshort> &node, vector<P*> &splitPoints, double _center[3], double _size);

  template <class P>
  inline void countPointsAndQueue(vector<P*> &i_points, double center[8][3], double size, cbitoct &parent, double *pcenter);

  template <class P>
  inline void countPointsAndQueue(P * const* pts, int n,  double center[8][3], double size, cbitoct &parent, double *pcenter);


  void childcenter(double *pcenter, double *ccenter, double size, unsigned char i); 

  template <class P>
inline unsigned char childIndex(const double *center, const P *point);
  
  
  unsigned long maxTargetPoints( cbitoct &node );
 
  void displayOctTreeAll( cbitoct &node, double *center, double size); 

  void displayOctTreeAllCulled( cbitoct &node, double *center, double size );

  void displayOctTreeCulledLOD(long targetpts, cbitoct &node, double *center, double size ); 
  void displayOctTreeLOD(long targetpts, cbitoct &node, double *center, double size ); 

  void displayOctTreeCulledLOD2(float lod, cbitoct &node, double *center, double size ); 
  void displayOctTreeLOD2(float lod, cbitoct &node, double *center, double size ); 
  
  
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

  void deserialize(std::string filename );
  void deserialize(std::ifstream &f, cbitoct &node);
  void serialize(std::ofstream &of, cbitoct &node);
  
  
  unsigned long maxtargetpoints;
  unsigned int current_lod_mode;
  
  void cycleLOD() {
    current_lod_mode = (current_lod_mode+1)%3;
  }
};
  
template <class P>
  compactTree::compactTree(vector<P *> &pts, double voxelSize, PointType _pointtype) {
    alloc = new PackedChunkAllocator;
    
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
    root = alloc->allocate<cbitoct>();    

    countPointsAndQueue(pts, newcenter, sizeNew, *root, center);
    maxtargetpoints =  maxTargetPoints(*root);
    current_lod_mode = 0;
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
      double distance;
      for (typename vector<P *>::iterator itr = splitPoints.begin(); 
          itr != splitPoints.end(); itr++) {
        for (unsigned int iterator = 0; iterator < 3; iterator++) {
          distance = (*itr)[iterator] - _center[iterator];
          
          if (distance >= _size) {
//            points[i++] = (1 << 15) -1;
            points[i++] = TSHORT_MAX; 
          } else {
//            points[i++] = (distance/_size ) * (1 << 15);//* pow(2,15) ;
            points[i++] = (distance/_size ) * TSHORT_MAXP1;
          }
        }
        for (unsigned int iterator = 3; iterator < POINTDIM; iterator++) {
          points[i++] = (*itr)[iterator];
        }
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

    countPointsAndQueue(splitPoints, newcenter, sizeNew, node.node, _center);
    return false;
  }
  
template <class P>
  void compactTree::countPointsAndQueue(vector<P*> &i_points, double center[8][3], double size, cbitoct &parent, double *pcenter) {
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
    cbitunion<tshort> *children = alloc->allocate<cbitunion<tshort> >(n_children);    
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
  void compactTree::countPointsAndQueue(P * const* pts, int n,  double center[8][3], double size, cbitoct &parent, double *pcenter) {
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
    cbitunion<tshort> *children = alloc->allocate<cbitunion<tshort> >(n_children);    
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
    alloc = new PackedChunkAllocator;
    
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
    //precision = vs/ (1 << 15); //pow(2, 15);
    precision = vs / TSHORT_MAXP1;
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
    root = alloc->allocate<cbitoct>();    

    countPointsAndQueue(pts, n, newcenter, sizeNew, *root, center);
    maxtargetpoints =  maxTargetPoints(*root);
    current_lod_mode = 0;
  } 

template <class P>
inline unsigned char compactTree::childIndex(const double *center, const P *point) {
  return  (point[0] >= center[0] ) | ((point[1] >= center[1] ) << 1) | ((point[2] >= center[2] ) << 2) ;
}
  
  void compactTree::getCenter(double _center[3]) const {
    _center[0] = center[0];
    _center[1] = center[1];
    _center[2] = center[2];
  }
#endif
