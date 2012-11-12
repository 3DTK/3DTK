/** @file 
 *  @brief Representation of the optimized k-d tree. MetaScan variant.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Thomas Escher
 */

#ifndef __KD_META_H__
#define __KD_META_H__

#include "kdparams.h"
#include "searchTree.h"
#include "data_types.h"
#include "kdTreeImpl.h"

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif



class Scan;


struct Index {
  unsigned int s, i;
  inline void set(unsigned int _s, unsigned int _i) { s = _s; i = _i; }
};

struct IndexAccessor {
  inline double* operator() (const DataXYZ* const* pts, const Index& i) const {
    return (*pts[i.s])[i.i];
  }
};

/**
 * @brief The optimized k-d tree. 
 * 
 * A kD tree for points, with limited
 * capabilities (find nearest point to
 * a given point, or to a ray).
 **/
class KDtreeMetaManaged : 
    public SearchTree,
    private KDTreeImpl<const DataXYZ* const*, Index, IndexAccessor>
{
public:
  KDtreeMetaManaged(const vector<Scan*>& scans);
  virtual ~KDtreeMetaManaged();
  
  virtual void lock();
  virtual void unlock();

  //! Aquires cached data first to pass on to the usual KDtree to process
  virtual double* FindClosest(double *_p, double maxdist2, int threadNum = 0) const;

  virtual double *FindClosestAlongDir(double *_p, double *_dir, double maxdist2, int threadNum = 0) const;
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
