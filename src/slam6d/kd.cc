/*
 * kd implementation
 *
 * Copyright (C) Andreas Nuechter, Kai Lingemann, Thomas Escher
 *
 * Released under the GPL version 3.
 *
 */

/** @file 
 *  @brief An optimized k-d tree implementation
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Thomas Escher Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#endif

#include "slam6d/kd.h"
#include "slam6d/globals.icc"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <algorithm>
using std::swap;
#include <cmath>
#include <cstring>

// KDtree class static variables
template<class PointData, class AccessorData, class AccessorFunc>
KDParams KDTreeImpl<PointData, AccessorData, AccessorFunc>::params[MAX_OPENMP_NUM_THREADS];

/**
 * Constructor
 *
 * Create a KD tree from the points pointed to by the array pts
 *
 * @param pts 3D array of points
 * @param n number of points
 */
KDtree::KDtree(double **pts, int n)
{
    create(Void(), pts, n);
}

KDtree::~KDtree()
{
}

/**
 * Finds the closest point within the tree,
 * wrt. the point given as first parameter.
 * @param _p point
 * @param maxdist2 maximal search distance.
 * @param threadNum Thread number, for parallelization
 * @return Pointer to the closest point
 */
double *KDtree::FindClosest(double *_p, double maxdist2, int threadNum) const
{
  params[threadNum].closest = 0;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  _FindClosest(Void(), threadNum);
  return params[threadNum].closest;
}
