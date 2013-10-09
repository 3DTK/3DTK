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
template<class T>
class KDParams
{
public:
  /** 
   * the closest point
   */
  T closest;

  /** 
   * distance to the closest point
   */
  double closest_d2;

  /** 
   * the point coordinate
   */
  double *p;

  /**
   * pointer to direction vector, if we're using FindClosestAlongDir
   */
  double *dir;

  double *p0;

  double dist_2;

  /**
   * vector of all neighbors within a given radius
   */
  vector<T> range_neighbors;
 
  /**
   * pointer to k nearest neighbors
   */
  T *closest_neighbors;

  /**
   * distances to k closest neighbors
   */
  double *distances;
  
  /**
   * k - the number of neighbors we want to find
   */
  int k;
};

#endif
