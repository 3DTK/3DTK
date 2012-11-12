/** 
 * @file 
 * @brief Representation of a general search trees
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SEARCHTREE_H__
#define __SEARCHTREE_H__

#include <vector>
using std::vector;

#include "ptpair.h"
#include "data_types.h"
#include "pairingMode.h"

/**
 * @brief The tree structure 
 * 
 * A tree holds the pointer to the data
 **/
class Tree {
public:  
  /**
   *	Destructor - deletes the tree
   * pure virtual, i.e., must be implented by a derived class
   */
  virtual inline ~Tree() {};
};


class Scan;
/**
 * @brief The search tree structure 
 * 
 * A search tree holds the pointer to the data.
 * Furthermore, search functionality must be privided
 **/
class SearchTree : public Tree {
  friend class Scan;
public:

  /**
   * Constructor (default)
   */
  inline SearchTree() {};
  
  /**
   *	Constructor - Constructs a tree from the input.
   * must be implented by a derived class
   *	@param pts 3D array of points
   *	@param n number of points
   */
  SearchTree(double **pts, int n);

  /**
   *	Destructor - deletes the tree
   * virtual, i.e., must be implented by a derived class
   */
  virtual inline ~SearchTree() {};
  
  /**
   * Aquire a lock on this tree, signaling that its resources are in use. Neccessary for cached data in the scanserver.
   */
  virtual inline void lock() {};
  
  /**
   * Release the lock on this tree, signaling that its resources are aren't in use anymore. Neccessary for cached data in the scanserver.
   */
  virtual inline void unlock() {};

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
  virtual double *FindClosest(double *_p, double maxdist2, int threadNum = 0) const = 0;

  virtual double *FindClosestAlongDir(double *_p, double *_dir, double maxdist2, int threadNum) const;

  virtual void getPtPairs(vector <PtPair> *pairs, 
				  double *source_alignxf, 
          double * const *q_points, unsigned int startindex, unsigned int endindex,
				  int thread_num,
				  int rnd, double max_dist_match2, double &sum,
				  double *centroid_m, double *centroid_d);
    
  virtual void getPtPairs(vector <PtPair> *pairs,
				  double *source_alignxf,
          const DataXYZ& xyz_r, const DataNormal& normal_r, unsigned int startindex, unsigned int endindex,
				  int thread_num,
				  int rnd, double max_dist_match2, double &sum,
          double *centroid_m, double *centroid_d, PairingMode pairing_mode = CLOSEST_POINT);
};

#endif
