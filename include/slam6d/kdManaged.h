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
#include "slam6d/kdTreeImpl.h"

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

class Scan;

struct ArrayAccessor {
    inline double *operator() (const DataXYZ& pts, unsigned int i) {
        return pts[i];
    }
};

/**
 * @brief The optimized k-d tree. 
 * 
 * A kD tree for points, with limited
 * capabilities (find nearest point to
 * a given point, or to a ray).
 **/
class KDtreeManaged :
    public SearchTree,
    private KDTreeImpl<const DataXYZ&, unsigned int, ArrayAccessor>

{
public:
  KDtreeManaged(Scan* scan);
  virtual ~KDtreeManaged() {}
  
  virtual void lock();
  virtual void unlock();

  //! Aquires cached data first to pass on to the usual KDtree to process
  virtual double* FindClosest(double *_p, double maxdist2, int threadNum = 0) const;

  virtual double *FindClosestAlongDir(double *_p, double *_dir, double maxdist2, int threadNum = 0) const;
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
