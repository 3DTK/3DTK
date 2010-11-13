/** 
 * @file 
 * @brief Representation of the optimized cached k-d tree.
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __KDC_H__
#define __KDC_H__

#ifdef _MSC_VER
#ifdef OPENMP
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#include "searchTree.h"
#include "kdcache.h"

/**
 * @brief The optimized k-d tree with caching. 
 * 
 * A kD tree for points, with limited
 * capabilities. The tree uses
 * forward backward pointers and
 * returns a pointer to the leaf
 **/
class KDtree_cache : public CachedSearchTree {
  
public:
  /**
   *	Constructor - Constructs an kd tree from the input.
   *	@param pts 3D array of points
   *	@param n number of points
   */
  KDtree_cache(double **pts, int n, KDtree_cache *_parent = 0);
  /**
   *	destructor
   */
  ~KDtree_cache() {
    if (!npts) {
#ifdef WITH_OPENMP_KD
      omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for
#endif
      for (int i = 0; i < 2; i++) {
	   if (i == 0 && node.child1) delete node.child1; 
        if (i == 1 && node.child2) delete node.child2; 
	 }
    } else {
      if (leaf.p) delete [] leaf.p;
    }
  }

  KDCacheItem* FindClosestCache(double *_p, double maxdist2, int threadNum = 0);
  KDCacheItem* FindClosestCacheInit(double *_p, double maxdist2, int threadNum = 0);

private:
#ifdef _OPENMP
#ifdef __INTEL_COMPILER
      __declspec (align(16)) static KDCacheItem cacheItem[MAX_OPENMP_NUM_THREADS];
#else
      static KDCacheItem cacheItem[MAX_OPENMP_NUM_THREADS];
#endif //__INTEL_COMPILER
#else
	static KDCacheItem cacheItem[MAX_OPENMP_NUM_THREADS];
#endif	

  /**
   * number of points. If this is 0: intermediate node.  If nonzero: leaf.
   */
  int npts;
  
  KDtree_cache *parent;
  double center[3];
  /**
   * Cue the standard rant about anon unions but not structs in C++
   */
  union {
    /** in case of node... */
    struct {
	 /** storing the center of the voxel (R^3) */
      double dx,  ///< defining the voxel itself
	        dy,  ///< defining the voxel itself
	        dz,  ///< defining the voxel itself
	        r2;  ///< defining the voxel itself
      int splitaxis;   ///< defining the kind of splitaxis
      KDtree_cache *child1;  ///< pointers to the childs
      KDtree_cache *child2;  ///< pointers to the childs
    } node;
    /** in case of leaf... */
    struct {
	 /** store the value itself */
      double **p;
    } leaf;
  };

  /**
   * Wrapped function
   */
  void _FindClosestCacheInit(int threadNum = 0);
  void _FindClosestCache(KDtree_cache *prev = 0, int threadNum = 0);

};

#endif


