/*
 * BruteForce Search implementation
 *
 * Copyright (C) Simon Engel, Michael Jechow
 *
 * Released under the GPL version 3.
 *
 */

#ifndef __BRUTEFORCE_H__
#define __BRUTEFORCE_H__

#include "slam6d/kdparams.h"
#include "slam6d/searchTree.h"

/**
 * @brief Brute Force Closest Point search datastructure
 *
 **/
class BruteForceNotATree : public SearchTree {

public:

  /**
   * default constructor
   */
  BruteForceNotATree();

  /**
   * Constructor using the point set pts and the num_of_pts n
   */
  BruteForceNotATree(double **_pts, size_t _n);

  /**
   * destructor
   */
  virtual ~BruteForceNotATree();


/**
 * Finds the closest point with brute force
 * wrt. the point given as first parameter.
 * @param _p point
 * @param maxdist2 maximal search distance.
 * @param threadNum Thread number, for parallelization
 * @return Pointer to the closest point
 */
  double *FindClosest(double *_p, double maxdist2, int threadNum = 0) const;

private:

  double** pts;
  double n;

};

#endif


