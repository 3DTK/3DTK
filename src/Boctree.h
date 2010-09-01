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
union bitunion;

/**
 * This is our preferred representation for the leaf nodes (as it is the most compact). 
 * BOctTree makes an array of this, the first containing the number of points (not the 
 * number of coordinates) stored.
 */
union dunion {
  double v;
  unsigned int    length;
  dunion() : length(0) {};

};

// TODO create a wrapper class to implement different point representations
//typedef list<double*> pointrep;
typedef dunion pointrep;

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
  static inline void link(bitoct &parent, bitunion *child) {
    parent.child_pointer = (long)((char*)child - (char*)&parent);
  }

  /**
   * Returns the children of this node (given as parent).
   */
  static inline void getChildren(bitoct &parent, bitunion* &children) {
    children = (bitunion*)((char*)&parent + parent.child_pointer);
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
union bitunion {
  pointrep *points;
  bitoct node;

  bitunion(pointrep *p) : points(p) {};
  bitunion(bitoct b) : node(b) {};
  bitunion() : points(0) {
	node.child_pointer = 0;
	node.leaf = 0;
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
class BOctTree {

public:
  
  BOctTree(double **pts, int n, double _voxelSize, unsigned int pointdim = 3);
  BOctTree(std::string filename, double *minmax = 0) {deserialize(filename, minmax); }

  virtual ~BOctTree();
  
  void GetOctTreeCenter(vector<double*>&c);
  void GetOctTreeRandom(vector<double*>&c);
  void GetOctTreeRandom(vector<double*>&c, unsigned int ptspervoxel);
  
  long countNodes();
  long countLeaves();

  void serialize(std::string filename, double *mimax = 0);
  void deserialize(std::string filename, double *minmax = 0);
 
protected:

  void serialize(std::ofstream &of, bitoct &node);
  void deserialize(std::ifstream &of, bitoct &node);

  void GetOctTreeCenter(vector<double*>&c, bitoct &node, double *center, double size);
  void GetOctTreeRandom(vector<double*>&c, bitoct &node);
  void GetOctTreeRandom(vector<double*>&c, unsigned int ptspervoxel, bitoct &node);
  long countNodes(bitoct &node);
  long countLeaves(bitoct &node);
  void deletetNodes(bitoct &node);

  pointrep *branch( bitoct &node, deque<double*> &points, double center[3], double size);

  pointrep *branch( bitoct &node, vector<double*> &points, double center[3], double size);
  pointrep *branch( bitoct &node, list<double*> &points, double center[3], double size);
  
  void countPointsAndQueue(list<double*> &i_points,
 		double center[8][3], double size, bitoct &parent); 

  void countPointsAndQueue(deque<double*> &i_points,
 		double center[8][3], double size, bitoct &parent); 
  
  void countPointsAndQueue(vector<double*> &i_points,
 		double center[8][3], double size, bitoct &parent); 

  void countPointsAndQueue(double **pts, int n,
    double center[8][3], double size, bitoct &parent);

  void childcenter(double *pcenter, double *ccenter, double size, int i);


  /**
   * the root of the octree 
   */
  bitoct* root;

  
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

  unsigned int POINTDIM;

};

#endif
