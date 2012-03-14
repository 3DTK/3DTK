/** @file 
 *  @brief Representation of the optimized k-d tree. 
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Thomas Escher
 */

#ifndef __KD_H__
#define __KD_H__

#include "slam6d/kdparams.h"
#include "slam6d/searchTree.h"

#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#ifdef WITH_SCANSERVER
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#endif //WITH_SCANSERVER

/**
 * @brief The optimized k-d tree. 
 * 
 * A kD tree for points, with limited
 * capabilities (find nearest point to
 * a given point, or to a ray).
 **/
class KDtree : public SearchTree {
public:
#ifndef WITH_SCANSERVER
  KDtree(double **pts, int n);
#else //WITH_SCANSERVER
  KDtree(const DataXYZ& pts, unsigned int* indices, unsigned int n);
#endif //WITH_SCANSERVER
  
  virtual ~KDtree();

#ifndef WITH_SCANSERVER
  virtual double *FindClosest(double *_p, double maxdist2, int threadNum = 0) const;
#else //WITH_SCANSERVER
  virtual double *FindClosest(double *_p, double maxdist2, int threadNum = 0) const { return 0; }
#endif //WITH_SCANSERVER

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
      KDtree *child1;  ///< pointers to the childs
      KDtree *child2;  ///< pointers to the childs
    } node;
    /** 
     * in case of leaf node ... 
     */
    struct {
#ifndef WITH_SCANSERVER
      /** 
       * store the value itself
       * Here we store just a pointer to the data
       */
      double **p;
#else //WITH_SCANSERVER
      //! Content is an array of indices to be put into the dynamically aquired data array
      unsigned int* p;
#endif //WITH_SCANSERVER
    } leaf;
  };
  
#ifndef WITH_SCANSERVER
  void _FindClosest(int threadNum) const;
#else //WITH_SCANSERVER
  void _FindClosest(const DataXYZ& pts, int threadNum) const;
#endif //WITH_SCANSERVER
};

#ifdef WITH_SCANSERVER
class Scan;

class KDtreeManaged : public KDtree {
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
#endif //WITH_SCANSERVER

#endif
