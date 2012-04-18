/** @file 
 *  @brief Representation of the optimized k-d tree. 
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Thomas Escher
 */

#ifndef __KD_MANAGED_H__
#define __KD_MANAGED_H__

#include "slam6d/kdparams.h"
#include "slam6d/searchTree.h"
#include "slam6d/data_types.h"

#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

/**
 * @brief The optimized k-d tree. 
 * 
 * A kD tree for points, with limited
 * capabilities (find nearest point to
 * a given point, or to a ray).
 **/
class KDtreeManagedBase : public SearchTree {
public:
  KDtreeManagedBase(const DataXYZ& pts, unsigned int* indices, unsigned int n);
  
  virtual ~KDtreeManagedBase();

  virtual double *FindClosest(double *_p, double maxdist2, int threadNum = 0) const { return 0; }

protected:
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
      KDtreeManagedBase *child1;  ///< pointers to the childs
      KDtreeManagedBase *child2;  ///< pointers to the childs
    } node;
    /** 
     * in case of leaf node ... 
     */
    struct {
      //! Content is an array of indices to be put into the dynamically aquired data array
      unsigned int* p;
    } leaf;
  };
  
  void _FindClosest(const DataXYZ& pts, int threadNum) const;
};

class Scan;

class KDtreeManaged : public KDtreeManagedBase {
public:
  KDtreeManaged(Scan* scan);
  virtual ~KDtreeManaged() {}
  
  virtual void lock();
  virtual void unlock();

  //! Aquires cached data first to pass on to the usual KDtree to process
  virtual double* FindClosest(double *_p, double maxdist2, int threadNum = 0) const;
private:
  Scan* m_scan;
  DataXYZ* m_data;

  //! Mutex for safely reducing points just once in a multithreaded environment
  boost::mutex m_mutex_locking;
  volatile unsigned int m_count_locking;

  // constructor initializer list hacks
  unsigned int* m_temp_indices;
  unsigned int* prepareTempIndices(unsigned int n);
};

#endif
