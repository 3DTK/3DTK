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
   * maximum distance to search in squared and not squared
   */

  double maxdist_d2;
  double maxdist_d;

  /**
   * the point coordinate
   */
  double *p;

  /**
   * pointer to direction vector, if we're using FindClosestAlongDir
   */
  double *dir;

  double *p0;

  /**
   * distance between two points for fixedRangeSearchBetween2Points
   */
  double dist;

  /**
   * vector of all neighbors within a given radius
   */
  std::vector<T> range_neighbors;

  /**
   * Note: BKD-tree specific.
   * vector of all the collected points in the tree,
   * i.e. all points that have not been deleted while
   * using the bkd trees at the moment of kd-forest reorganization.
   */
  std::vector<T> collected_pts;

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

  /**
   * pre-calculated values for segment search functions
   */
  double *segment_dir; // vector from one point of the segment to the other
  double segment_len2; // squared length of segment_dir
  double *segment_n;   // segment_dir divided by segment_len2
  double *segment_center; // center point of the segment
  double segment_r2;   // bounding sphere of search area radius
};

#endif
