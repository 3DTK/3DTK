/**
 * @file 
 * @brief Representation of an octree
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __D2TREE_H__
#define __D2TREE_H__

#include "searchTree.h"

/**
 * @brief D2TREE
 * 
 * An OCTREE containing the distances 
 * to data points
 */
class D2Tree : public SearchTree {
  
public:

  D2Tree(double **pts, int n, double voxelSize);
  
  /**
   * destructor
   */
  virtual ~D2Tree();

  /**
   * This Search function returns a pointer to the closest point
   * of the query point within maxdist2. If there if no such point
   * a 0-pointer might be returned. 
   *
   * @param _p Pointer to query point
   * @param maxdist2 Maximal distance for closest points
   * @param threadNum If parallel threads share the search tree the thread num must be given
   * @return Pointer to closest point 
   */
  virtual double *FindClosest(double *_p, double maxdist2, int threadNum = 0);
  double *_FindClosest(double *_p);

   void outputTree();
   int _outputTree();
    
private:

  D2Tree(double *pts, double center[3], double _x_size, double _y_size, double _z_size, bool _isleaf);
  D2Tree* setClosestPointRecursive(double *, double center[3],
							double _x_size, double _y_size, double _z_size);
  
  void updateClosestIn(double *);
  void updateClosestOut(double *);

  static inline void calcDivide(double x_size, double y_size, double z_size,
						  double center[3],
						  double &x_sizeNew, double &y_sizeNew, double &z_sizeNew,
						  double new_center[8][3]);
  
  void updateChilds(double *pts, D2Tree **child,
				double &x_sizeNew, double &y_sizeNew, double &z_sizeNew,
				double newcenter[8][3]);

  /**
   * the (maximal 8) children of a box
   */
  D2Tree **child;

  /**
   * storing a ptr to the closest point
   */
  double *closest_point;
  
  /**
   * storing the center
   */
  double center[3];
  
  /**
   * storing the dimensions
   */
  double x_size, y_size, z_size;
  
  /**
   * is it a leaf?
   */
  bool isleaf;

  /**
   * storing the voxel size
   */
  static double voxelSize; 
};

#include "d2tree.icc"
  
#endif
