/** @file 
 *  @brief An optimized k-d tree implementation
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
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
KDParams KDtree::params[MAX_OPENMP_NUM_THREADS];

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
  // Find bbox
  double xmin = pts[0][0], xmax = pts[0][0];
  double ymin = pts[0][1], ymax = pts[0][1];
  double zmin = pts[0][2], zmax = pts[0][2];
  for (int i = 1; i < n; i++) {
    xmin = min(xmin, pts[i][0]);
    xmax = max(xmax, pts[i][0]);
    ymin = min(ymin, pts[i][1]);
    ymax = max(ymax, pts[i][1]);
    zmin = min(zmin, pts[i][2]);
    zmax = max(zmax, pts[i][2]);
  }

  // Leaf nodes
  if ((n > 0) && (n <= 10)) {
    leaf.p = new double*[n];
    npts = n;
    memcpy(leaf.p, pts, n * sizeof(double *));
    return;
  }

  // Else, interior nodes
  npts = 0;

  node.center[0] = 0.5 * (xmin+xmax);
  node.center[1] = 0.5 * (ymin+ymax);
  node.center[2] = 0.5 * (zmin+zmax);
  node.dx = 0.5 * (xmax-xmin);
  node.dy = 0.5 * (ymax-ymin);
  node.dz = 0.5 * (zmax-zmin);
  node.r2 = sqr(node.dx) + sqr(node.dy) + sqr(node.dz);

  // Find longest axis
  if (node.dx > node.dy) {
    if (node.dx > node.dz) {
      node.splitaxis = 0;
    } else {
      node.splitaxis = 2;
    }
  } else {
    if (node.dy > node.dz) {
      node.splitaxis = 1;
    } else {
      node.splitaxis = 2;
    }
  }

  // Partition
  double splitval = node.center[node.splitaxis];

  if ( fabs(max(max(node.dx,node.dy),node.dz)) < 0.01 ) {
    leaf.p = new double*[n];
    npts = n;
    memcpy(leaf.p, pts, n * sizeof(double *));
    return;
  }


  double **left = pts, **right = pts + n - 1;
  while (1) {
    while ((*left)[node.splitaxis] < splitval)
      left++;
    while ((*right)[node.splitaxis] >= splitval)
      right--;
    if (right < left)
      break;
    swap(*left, *right);
  }
  // Build subtrees
  int i;
#ifdef WITH_OPENMP_KD                   // does anybody know the reason why this is slower ?? --Andreas
  omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for schedule(dynamic) 
#endif
  for (i = 0; i < 2; i++) {
    if (i == 0) node.child1 = new KDtree(pts,	left-pts);
    if (i == 1) node.child2 = new KDtree(left, n-(left-pts));
  }
}

/**
 * Finds the closest point within the tree,
 * wrt. the point given as first parameter.
 * @param _p point
 * @param maxdist2 maximal search distance.
 * @param threadNum Thread number, for parallelization
 * @return Pointer to the closest point
 */
double *KDtree::FindClosest(double *_p, double maxdist2, int threadNum)
{
  params[threadNum].closest = 0;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  _FindClosest(threadNum);
  return params[threadNum].closest;
}

/**
 * Wrapped function 
 */
void KDtree::_FindClosest(int threadNum)
{
  // Leaf nodes
  if (npts) {
    for (int i = 0; i < npts; i++) {
      double myd2 = Dist2(params[threadNum].p, leaf.p[i]);
      if (myd2 < params[threadNum].closest_d2) {
	   params[threadNum].closest_d2 = myd2;
	   params[threadNum].closest = leaf.p[i];
      }
    }
    return;
  }

  // Quick check of whether to abort  
  double approx_dist_bbox = max(max(fabs(params[threadNum].p[0]-node.center[0])-node.dx,
							 fabs(params[threadNum].p[1]-node.center[1])-node.dy),
						  fabs(params[threadNum].p[2]-node.center[2])-node.dz);
  if (approx_dist_bbox >= 0 && sqr(approx_dist_bbox) >= params[threadNum].closest_d2)
    return;

  // Recursive case
  double myd = node.center[node.splitaxis] - params[threadNum].p[node.splitaxis];
  if (myd >= 0.0) {
    node.child1->_FindClosest(threadNum);
    if (sqr(myd) < params[threadNum].closest_d2) {
      node.child2->_FindClosest(threadNum);
    }
  } else {
    node.child2->_FindClosest(threadNum);
    if (sqr(myd) < params[threadNum].closest_d2) {
      node.child1->_FindClosest(threadNum);
    }
  }
}
