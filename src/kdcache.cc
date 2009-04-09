/** @file 
 *  @brief An optimized cached k-d tree implementation
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "kdcache.h"

// KDtree_cache class static variables
KDCache KDtree_cache::cache[4];

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
#ifdef WITH_OPENMP_KD
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
KDCache KDtree_cache::FindClosestCache(double *_p, double maxdist2, int threadNum)
{
  cache[threadNum].param.closest = 0;
  cache[threadNum].param.closest_d2 = HUGE_VAL;
  cache[threadNum].param.p = _p;
  _FindClosestCache((KDtree_cache*)0, threadNum);
  return cache[threadNum];
}


/**
 * Wrapped function 
 */
void KDtree_cache::_FindClosestCache(KDtree_cache *prev, int threadNum)
{
  // backtrack
  // test until point lies within kd tree
  bool backtrack = true;
  if (((center[0] - node.dx) < cache[threadNum].param.p[0]) &&
      ((center[0] + node.dx) > cache[threadNum].param.p[0]) &&
      ((center[1] - node.dy) < cache[threadNum].param.p[1]) &&
      ((center[1] + node.dy) > cache[threadNum].param.p[1]) &&
      ((center[2] - node.dz) < cache[threadNum].param.p[2]) &&
      ((center[2] + node.dz) > cache[threadNum].param.p[2])) {
    backtrack = false;
  }

  if (backtrack) {
    if (parent != 0) {
	 parent->_FindClosestCache(prev, threadNum);
    } else {
	 cache[threadNum].node = this;
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
KDCache KDtree_cache::FindClosestCacheInit(double *_p, double maxdist2, int threadNum)
{
  cache[threadNum].param.closest = 0;
  cache[threadNum].param.closest_d2 = maxdist2;
  cache[threadNum].param.p = _p;
  cache[threadNum].node = this;
  _FindClosestCacheInit(threadNum);
  return cache[threadNum];
}
 
/**
 * Wrapped function
 */
void KDtree_cache::_FindClosestCacheInit(int threadNum)
{
  // Leaf nodes
  if (npts) {
    for (int i = 0; i < npts; i++) {
      double myd2 = Dist2(cache[threadNum].param.p, leaf.p[i]);
      if (myd2 < cache[threadNum].param.closest_d2) {
	   cache[threadNum].param.closest_d2 = myd2;
	   cache[threadNum].param.closest = leaf.p[i];
	   cache[threadNum].node = parent;
      }
    }
    return;
  }

  // Quick check of whether to abort
  double approx_dist_bbox = max(max(fabs(cache[threadNum].param.p[0]-center[0])-node.dx,
				    fabs(cache[threadNum].param.p[1]-center[1])-node.dy),
				fabs(cache[threadNum].param.p[2]-center[2])-node.dz);
  if (approx_dist_bbox >= 0 && sqr(approx_dist_bbox) >= cache[threadNum].param.closest_d2) {
    cache[threadNum].node = 0;
    return;
  }
  
  // Recursive case
  double myd = center[node.splitaxis] - cache[threadNum].param.p[node.splitaxis];
  if (myd >= 0.0) {
    node.child1->_FindClosestCacheInit(threadNum);
    if (sqr(myd) < cache[threadNum].param.closest_d2) {
      node.child2->_FindClosestCacheInit(threadNum);
    }
  } else {
    node.child2->_FindClosestCacheInit(threadNum);
    if (sqr(myd) < cache[threadNum].param.closest_d2) {
      node.child1->_FindClosestCacheInit(threadNum);
    }
  }
}









