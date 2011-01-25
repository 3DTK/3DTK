/** 
 * @file 
 * @brief Representation of a general search trees
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SEARCHTREE_H__
#define __SEARCHTREE_H__

#include "searchCache.h"

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


/**
 * @brief The search tree structure 
 * 
 * A search tree holds the pointer to the data.
 * Furthermore, search functionality must be privided
 **/
class SearchTree : public Tree {
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
   * This Search function returns a pointer to the closest point
   * of the query point within maxdist2. If there if no such point
   * a 0-pointer might be returned. 
   *
   * @param _p Pointer to query point
   * @param maxdist2 Maximal distance for closest points
   * @param threadNum If parallel threads share the search tree the thread num must be given
   * @return Pointer to closest point 
   */
  virtual double *FindClosest(double *_p, double maxdist2, int threadNum = 0) = 0;
};


/**
 * @brief The search tree structure 
 * 
 * A search tree holds the pointer to the data.
 * Furthermore, search functionality must be privided
 **/
class CachedSearchTree : public SearchTree {
public:  
  /**
   * Constructor (default)
   */
  inline CachedSearchTree() {};

  /**
   *	Constructor - Constructs a tree from the input.
   * pure virtual, i.e., must be implented by a derived class
   *	@param pts 3D array of points
   *	@param n number of points
   */
  CachedSearchTree(double **pts, int n, CachedSearchTree *_parent = 0);

  /**
   *	Destructor - deletes the tree
   * pure virtual, i.e., must be implented by a derived class
   */
  virtual inline ~CachedSearchTree() {};

  /**
   * This Search function returns a pointer to the closest point
   * of the query point within maxdist2. This function searches from the
   * root to the leafs.
   *
   * @param _p Pointer to query point
   * @param maxdist2 Maximal distance for closest points
   * @param threadNum If parallel threads share the search tree the thread num must be given
   * @return A new cach Item
   */
  virtual SearchTreeCacheItem* FindClosestCacheInit(double *_p, double maxdist2, int threadNum = 0) = 0;

  /**
   * This Search function returns a pointer to the closest point
   * of the query point within maxdist2. This function might be started
   * from the leafs.
   *
   * @param _p Pointer to query point
   * @param maxdist2 Maximal distance for closest points
   * @param threadNum If parallel threads share the search tree the thread num must be given
   * @return A new cach Item
   */
  virtual SearchTreeCacheItem* FindClosestCache(double *_p, double maxdist2, int threadNum = 0) = 0;
  double *FindClosest(double *_p, double maxdist2, int threadNum = 0) {
    return 0; 
  }
};


#endif
