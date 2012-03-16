/** @file 
 *  @brief An optimized cached k-d tree implementation
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *
 * This file implements the paper (3dim2007.pdf):
 * Andreas Nuechter, Kai Lingemann, and Joachim Hertzberg. Cached k-d tree search for ICP
 * algorithms. In Proceedings of the 6th IEEE International Conference on Recent Advances
 * in 3D Digital Imaging and Modeling (3DIM '07), IEEE Computer Society Press, ISBN
 */

#include "slam6d/kdc.h"
#include "slam6d/globals.icc"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <algorithm>
using std::swap;
#include <cmath>
#include <cstring>

#define CENTROID

// KDtree_cache class static variables
KDCacheItem KDtree_cache::cacheItem[MAX_OPENMP_NUM_THREADS];

/**
 * Constructor
 *
 * Create a cached KD tree from the points pointed to by the array pts
 * 
 * A cached k-d tree has bi-directional pointers.
 * Reference: "Cached k-d tree search for ICP algorithms" by A. Nuechter
 *            et al., Proceedings IEEE 3DIM, Montreal, Canada, 2007.
 * 
 */
KDtree_cache::KDtree_cache(double **pts, int n, KDtree_cache *_parent)
{
  parent = _parent;

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

  center[0] = 0.5 * (xmin+xmax);
  center[1] = 0.5 * (ymin+ymax);
  center[2] = 0.5 * (zmin+zmax);
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
  double splitval = center[node.splitaxis];

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
#ifdef WITH_OPENMP_KD                         // does anybody know the reason why this is slower ?? --Andreas
  omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for
#endif
  for (int i = 0; i < 2; i++) {
    if (i == 0) node.child1 = new KDtree_cache(pts,	left-pts,  this);
    if (i == 1) node.child2 = new KDtree_cache(left, n-(left-pts), this);
  }
}


/**
 * Finds the closest point within the tree,
 * wrt. the point given as first parameter.
 *
 * You may start this function from a leaf node
 * or any intermediate node.
 *
 * @param _p point
 * @param maxdist2 maximal search distance.
 * @param threadNum Thread number, for parallelization
 * @return Pointer to the closest point
 */
KDCacheItem* KDtree_cache::FindClosestCache(double *_p, double maxdist2, int threadNum)
{
  cacheItem[threadNum].param.closest = 0;
  cacheItem[threadNum].param.closest_d2 = maxdist2;
  cacheItem[threadNum].param.p = _p;
  _FindClosestCache((KDtree_cache*)0, threadNum);
  return &cacheItem[threadNum];
}


/**
 * Wrapped function 
 */
void KDtree_cache::_FindClosestCache(KDtree_cache *prev, int threadNum)
{
  // backtrack
  // test until point lies within kd tree
  bool backtrack = true;
  if (((center[0] - node.dx) < cacheItem[threadNum].param.p[0]) &&
      ((center[0] + node.dx) > cacheItem[threadNum].param.p[0]) &&
      ((center[1] - node.dy) < cacheItem[threadNum].param.p[1]) &&
      ((center[1] + node.dy) > cacheItem[threadNum].param.p[1]) &&
      ((center[2] - node.dz) < cacheItem[threadNum].param.p[2]) &&
      ((center[2] + node.dz) > cacheItem[threadNum].param.p[2])) {
    backtrack = false;
  }

  if (backtrack) {
    if (parent != 0) {
	 parent->_FindClosestCache(prev, threadNum);
    } else {
	 cacheItem[threadNum].node = this;
	 return;
    }
  } else {
    _FindClosestCacheInit(threadNum);
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
KDCacheItem* KDtree_cache::FindClosestCacheInit(double *_p, double maxdist2, int threadNum)
{
  cacheItem[threadNum].param.closest = 0;
  cacheItem[threadNum].param.closest_d2 = maxdist2;
  cacheItem[threadNum].param.p = _p;
  cacheItem[threadNum].node = this;
  _FindClosestCacheInit(threadNum);
  return &cacheItem[threadNum];
}
 
/**
 * Wrapped function
 */
void KDtree_cache::_FindClosestCacheInit(int threadNum)
{
  // Leaf nodes
  if (npts) {
    for (int i = 0; i < npts; i++) {
      double myd2 = Dist2(cacheItem[threadNum].param.p, leaf.p[i]);
      if (myd2 < cacheItem[threadNum].param.closest_d2) {
	   cacheItem[threadNum].param.closest_d2 = myd2;
	   cacheItem[threadNum].param.closest = leaf.p[i];
	   cacheItem[threadNum].node = parent;
      }
    }
    return;
  }

  // Quick check of whether to abort
  double approx_dist_bbox = max(max(fabs(cacheItem[threadNum].param.p[0]-center[0])-node.dx,
				    fabs(cacheItem[threadNum].param.p[1]-center[1])-node.dy),
				fabs(cacheItem[threadNum].param.p[2]-center[2])-node.dz);
  if (approx_dist_bbox >= 0 && sqr(approx_dist_bbox) >= cacheItem[threadNum].param.closest_d2) {
    cacheItem[threadNum].node = 0;
    return;
  }
  
  // Recursive case
  double myd = center[node.splitaxis] - cacheItem[threadNum].param.p[node.splitaxis];
  if (myd >= 0.0) {
    node.child1->_FindClosestCacheInit(threadNum);
    if (sqr(myd) < cacheItem[threadNum].param.closest_d2) {
      node.child2->_FindClosestCacheInit(threadNum);
    }
  } else {
    node.child2->_FindClosestCacheInit(threadNum);
    if (sqr(myd) < cacheItem[threadNum].param.closest_d2) {
      node.child1->_FindClosestCacheInit(threadNum);
    }
  }
}

KDCacheItem* KDtree_cache::initCache(const Scan* Target)
{
  KDCacheItem *closest = 0;
  
  // determine cache
  for(unsigned int i = 0; i < closest_cache.size(); i++) {
    if (closest_cache[i]->target == Target) {
      closest = closest_cache[i]->item; 	                 // cache found
      break;
    }
  }
  // cache for this target is not initialized
  if (closest == 0) {
    closest = new KDCacheItem[Target->get_points_red_size()];
    KDCache *nc = new KDCache;
    nc->item = closest;
    nc->target = Target;
    closest_cache.push_back(nc);                              // append cache
  }

  return closest;
}


void KDtree_cache::getPtPairs(vector <PtPair> *pairs, 
    double *source_alignxf,                          // source
    double * const *q_points, unsigned int startindex, unsigned int nr_qpts,  // target
    int thread_num,
    int rnd, double max_dist_match2, double &sum,
    double *centroid_m, double *centroid_d, Scan *Target)
{
  KDCacheItem *closest;
  #pragma omp critical
  {
  closest = initCache(Target);
  }

  centroid_m[0] = 0.0;
  centroid_m[1] = 0.0;
  centroid_m[2] = 0.0;
  centroid_d[0] = 0.0;
  centroid_d[1] = 0.0;
  centroid_d[2] = 0.0;

  double local_alignxf_inv[16];
  M4inv(source_alignxf, local_alignxf_inv);

  for (unsigned int i = startindex; i < (unsigned int)nr_qpts; i++) {
    if (rnd > 1 && rand(rnd) != 0) continue;  // take about 1/rnd-th of the numbers only

    double p[3];
    transform3(local_alignxf_inv, q_points[i], p);

    if (closest[i].node) {
      closest[i] = *(closest[i].node->FindClosestCache(p, max_dist_match2, thread_num));
    } else {
      closest[i] = *(this->FindClosestCacheInit(p, max_dist_match2, thread_num));
    }
    if (closest[i].param.closest_d2 < max_dist_match2 ) {
      transform3(source_alignxf, closest[i].param.closest, p);

      centroid_d[0] += q_points[i][0];
      centroid_d[1] += q_points[i][1];
      centroid_d[2] += q_points[i][2];
      centroid_m[0] += p[0];
      centroid_m[1] += p[1];
      centroid_m[2] += p[2];	 

      PtPair myPair(p, q_points[i]);
      double p12[3] = { 
        myPair.p1.x - myPair.p2.x, 
        myPair.p1.y - myPair.p2.y,
        myPair.p1.z - myPair.p2.z };
      sum += Len2(p12);

      pairs->push_back(myPair);
    }
  }

  centroid_m[0] /= pairs->size();
  centroid_m[1] /= pairs->size();
  centroid_m[2] /= pairs->size();
  centroid_d[0] /= pairs->size();
  centroid_d[1] /= pairs->size();
  centroid_d[2] /= pairs->size();

  return;
}






