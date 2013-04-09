/*
 * kd implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/** @file 
 *  @brief An optimized k-d tree implementation
 *  @author Remus Dumitru. Jacobs University Bremen, Germany
 *  @author Corneliu-Claudiu Prodescu. Jacobs University Bremen, Germany
 *  @author Andreas Nuechter. Jacobs University Bremen, Germany.
 *  @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Thomas Escher Inst. of CS, University of Osnabrueck, Germany.
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
#include <limits>
#include <vector>

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
double *KDtree::FindClosest(double *_p,
                            double maxdist2,
                            int threadNum) const
{
  params[threadNum].closest = 0;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  _FindClosest(Void(), threadNum);
  return params[threadNum].closest;
}

double *KDtree::FindClosestAlongDir(double *_p,
                                    double *_dir,
                                    double maxdist2,
                                    int threadNum) const
{
  params[threadNum].closest = NULL;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  params[threadNum].dir = _dir;
  _FindClosestAlongDir(Void(), threadNum);
  return params[threadNum].closest;
}

vector<Point> KDtree::kNearestNeighbors(double *_p,
                                        int _k,
                                        int threadNum) const
{
  vector<Point> result;    
  params[threadNum].closest = 0;
  params[threadNum].p = _p;
  params[threadNum].k = _k;
  // todo fix this C/C++ mixture
  params[threadNum].closest_neighbors = (double **)calloc(_k,
                                                          sizeof(double *) );
  params[threadNum].distances = (double *)calloc(_k,
                                                 sizeof(double));
  
  _KNNSearch(Void(), threadNum);
  
  free (params[threadNum].distances);

  for (int i = 0; i < _k; i++) {
#pragma omp critical
    result.push_back(Point(params[threadNum].closest_neighbors[i][0],
                           params[threadNum].closest_neighbors[i][1],
                           params[threadNum].closest_neighbors[i][2]));
  }
  
  free (params[threadNum].closest_neighbors);

  return result;
}


vector<Point> KDtree::fixedRangeSearchBetween2Points(double *_p,
                      double *_p0,
                      double maxdist2,
                      int threadNum) const {
  vector<Point> result;
  params[threadNum].closest = _p0;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  params[threadNum].dist_2 = Dist2(_p, _p0);

  double * _dir = new double[3];
  for(int i = 0; i < 3; i++) {
    _dir[i] = _p0[i] - _p[i];
  }

  Normalize3(_dir);
  
  params[threadNum].dir = _dir;
  params[threadNum].range_neighbors.clear();

  _fixedRangeSearchBetween2Points(Void(), threadNum);
  
  for (size_t i = 0; i < params[threadNum].range_neighbors.size(); i++) {
    result.push_back(Point(params[threadNum].range_neighbors[i][0],
                           params[threadNum].range_neighbors[i][1],
                           params[threadNum].range_neighbors[i][2]));
  }
  
  delete[] _dir;
  return result;
}


vector<Point> KDtree::fixedRangeSearchAlongDir(double *_p,
                      double *_dir,
                      double maxdist2,
                      int threadNum) const {
  vector<Point> result;
  params[threadNum].closest = NULL;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  params[threadNum].dir = _dir;
  params[threadNum].range_neighbors.clear();

  _fixedRangeSearchAlongDir(Void(), threadNum);
  
  for (size_t i = 0; i < params[threadNum].range_neighbors.size(); i++) {
    result.push_back(Point(params[threadNum].range_neighbors[i][0],
                           params[threadNum].range_neighbors[i][1],
                           params[threadNum].range_neighbors[i][2]));
  }
  
  return result;
}

vector<Point> KDtree::fixedRangeSearch(double *_p,
                                       double sqRad2,
                                       int threadNum) const
{
  vector<Point> result;
  params[threadNum].closest = 0;
  params[threadNum].closest_d2 = sqRad2;
  params[threadNum].p = _p;
  params[threadNum].range_neighbors.clear();
  _FixedRangeSearch(Void(), threadNum);
  
  for (size_t i = 0; i < params[threadNum].range_neighbors.size(); i++) {
#pragma omp critical    
    result.push_back(Point(params[threadNum].range_neighbors[i][0],
                           params[threadNum].range_neighbors[i][1],
                           params[threadNum].range_neighbors[i][2]));
  }
  
  return result;
}
