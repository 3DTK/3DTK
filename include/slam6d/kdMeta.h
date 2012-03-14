/** @file 
 *  @brief Representation of the optimized k-d tree. MetaScan variant.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Thomas Escher
 */

#ifndef __KD_META_H__
#define __KD_META_H__

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
class KDtreeMeta : public SearchTree {
protected:
  class Index {
  public:
    unsigned int s, i;
    inline void set(unsigned int _s, unsigned int _i) { s = _s; i = _i; }
  };
public:
  KDtreeMeta();
  virtual ~KDtreeMeta();

  void create(const DataXYZ* const* pts, Index* indices, unsigned int n);

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
      KDtreeMeta *child1;  ///< pointers to the childs
      KDtreeMeta *child2;  ///< pointers to the childs
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
      Index* p;
#endif //WITH_SCANSERVER
    } leaf;
  };
  
  inline double* point(const DataXYZ* const* pts, const Index& i) const {
    return (*pts[i.s])[i.i];
  }
  
  void _FindClosest(const DataXYZ* const* pts, int threadNum) const;
};

class Scan;

class KDtreeMetaManaged : public KDtreeMeta {
public:
  KDtreeMetaManaged(const vector<Scan*>& scans);
  virtual ~KDtreeMetaManaged();
  
  virtual void lock();
  virtual void unlock();

  //! Aquires cached data first to pass on to the usual KDtree to process
  virtual double* FindClosest(double *_p, double maxdist2, int threadNum = 0) const;
private:
  Scan** m_scans;
  DataXYZ** m_data;
  unsigned int m_size;

  //! Mutex for safely reducing points just once in a multithreaded environment
  boost::mutex m_mutex_locking;
  volatile unsigned int m_count_locking;

  // constructor initializer list hacks
  Index* m_temp_indices;
  Index* prepareTempIndices(const vector<Scan*>& scans);
  unsigned int getPointsSize(const vector<Scan*>& scans);
};

#endif
