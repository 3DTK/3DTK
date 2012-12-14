/*
 * ann_kd implementation
 *
 * Copyright (C) Andreas Nuechter, Ulugbek Makhmudov 
 *
 * Released under the GPL version 3.
 *
 */

/** @file 
 *  @brief Encapsules the implementation of ANN k-d trees. 
 *  @author Ulugbek Makhmudov, Jacobs University Bremen, Bremen, Germany.
 *  @author Andreas Nuechter, Jacobs University Bremen, Bremen, Germany.
 */

#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#endif

#include "slam6d/ann_kd.h"
#include "slam6d/globals.icc"          

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <algorithm>
using std::swap;
#include <cmath>
#include <cstring>



/**
 * Constructor
 *
 * Create ANN KD tree from the points pointed to by the array pts
 *
 * @param pts 3D array of points
 * @param n number of points
 */
ANNtree::ANNtree(PointerArray<double>&_pts, int n)
{
  pts = new double*[n];
  double* tpts = new double[3*n];
  for(int i = 0, j = 0; i < n; i++) {
    pts[i] = &tpts[j];
    tpts[j++] = _pts.get()[i][0];
    tpts[j++] = _pts.get()[i][1];
    tpts[j++] = _pts.get()[i][2];
  }

  // links to the constructor of ANNkd_tree
  annkd = new ANNkd_tree(pts, n, 3, 1, ANN_KD_SUGGEST); 
  cout << "ANNkd_tree was generated with " << n << " points" << endl;
  nn = new ANNdist[1];
  nn_idx = new ANNidx[1];    
}

/**
 * Destructor
 *
 * Cleans up the instance of ANN KD tree
 *
 *
 */
ANNtree::~ANNtree()
{
  delete annkd; //links to the destructor of ANNkd_tree
  delete [] nn;
  delete [] nn_idx;
  delete [] pts[0]; 
  delete [] pts;
}


/**
 * Finds the closest point within the tree,
 * wrt. the point given as first parameter.
 * @param _p point
 * @param maxdist2 maximal search distance.
 * @param threadNum Thread number, for parallelization
 * @return Pointer to the closest point
 */  
double *ANNtree::FindClosest(double *_p, double maxdist2, int threadNum) const
{

#pragma omp critical
  annkd->annkSearch(_p, 1, nn_idx, nn, 0.0);

  int idx = nn_idx[0];
 
  if (Dist2(_p, pts[idx]) > maxdist2) return 0;

  return pts[idx];
}  

