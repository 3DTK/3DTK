/** @file 
 *  @brief Representation of the optimized k-d tree. 
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __KD_H__
#define __KD_H__

#include "slam6d/kdparams.h"
#include "slam6d/searchTree.h"

#ifdef _MSC_VER
#ifdef OPENMP
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

/**
 * @brief The optimized k-d tree. 
 * 
 * A kD tree for points, with limited
 * capabilities (find nearest point to
 * a given point, or to a ray).
 **/
class KDtree : public SearchTree {

public:

  KDtree(double **pts, int n);
  
  /**
   * destructor
   */
  virtual ~KDtree() {                
    if (!npts) {
#ifdef WITH_OPENMP_KD
      omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for schedule(dynamic)
#endif
      for (int i = 0; i < 2; i++) {
	   if (i == 0 && node.child1) delete node.child1; 
        if (i == 1 && node.child2) delete node.child2; 
	 }
    } else {
      if (leaf.p) delete [] leaf.p;
    }
  }

  double *FindClosest(double *_p, double maxdist2, int threadNum = 0);

private:
  /**
   * storing the parameters of the k-d tree, i.e., the current closest point,
   * the distance to the current closest point and the point itself.
   * These global variable are needed in this search.
   *
   * Padded in the parallel case.
   */
#ifdef _OPENMP
#ifdef __INTEL_COMPILER
  __declspec (align(16)) static KDParams params[MAX_OPENMP_NUM_THREADS];
#else
  static KDParams params[MAX_OPENMP_NUM_THREADS];
#endif //__INTEL_COMPILER
#else
  static KDParams params[MAX_OPENMP_NUM_THREADS];
#endif	

  /**
   * number of points. If this is 0: intermediate node. If nonzero: leaf.
   */
  int npts;
  
  /**
   * Cue the standard rant about anon unions but not structs in C++
   */
  union {
    /** 
     * in case of internal node... 
     */
    struct {	 
      double center[3]; ///< storing the center of the voxel (R^3)
      double dx,  ///< defining the voxel itself
	     dy,  ///< defining the voxel itself
	     dz,  ///< defining the voxel itself
	     r2;  ///< defining the voxel itself
      int splitaxis;   ///< defining the kind of splitaxis
      KDtree *child1;  ///< pointers to the childs
      KDtree *child2;  ///< pointers to the childs
    } node;
    /** 
     * in case of leaf node ... 
     */
    struct {
      /** 
       * store the value itself
       * Here we store just a pointer to the data
       */
      double **p;
    } leaf;
  };

  void _FindClosest(int threadNum);
};

#endif


