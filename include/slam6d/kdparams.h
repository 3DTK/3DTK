/** 
 * @file 
 * @brief Representation of the parameter of a k-d tree
 * @author Andreas Nuechter. jacobs University Bremen, Germany.
 */

#ifndef __KDPARAMS_H__
#define __KDPARAMS_H__

#include "slam6d/point.h"

#include <vector>
using std::vector;

/**
 * @brief Contains the intermediate (static) values of a k-d tree 
 * 
 * A parameter class for the latter k-d tree.
 * Includes the padding for parallelizetion
 * to avoid cache conflicts.
 **/
class KDParams
{
public:
  /** 
   * pointer to the closest point.  size = 4 bytes of 32 bit machines 
   */
  double *closest;

  /** 
   * distance to the closest point. size = 8 bytes 
   */
  double closest_d2;

  /** 
   * pointer to the point, size = 4 bytes of 32 bit machines 
   */
  double *p;

  /**
   * pointer to direction vector, if we're using FindClosestAlongDir
   */
  double *dir;

  double dist_2;

  /**
   * vector of all neighbors within a given radius
   */
  vector<double *> range_neighbors;
  
  /**
   * pointer to k nearest neighbors
   */
  double **closest_neighbors;

  /**
   * distances to k closest neighbors
   */
  double *distances;
  
  /**
   * k - the number of neihgbors we want to find
   */
  int k;
  
  /** 
   * add some padding to avoid false sharing
   */
  int padding[28];
};

#endif
