/** 
 * @file 
 * @brief Representation of the parameter of a k-d tree
 * @author Andreas Nuechter. jacobs University Bremen, Germany.
 */

#ifndef __KDPARAMS_H__
#define __KDPARAMS_H__

#include "slam6d/point.h"

#include <vector>

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

  /**
   * heap for KNN.
   */
  std::vector<std::pair<Point, double> > heap;
  
  /** 
   * expand to 128 bytes to avoid false-sharing, 16 bytes from above + 28*4 bytes = 128 bytes
   */
  int padding[28];
};

#endif
