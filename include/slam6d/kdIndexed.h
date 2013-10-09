/** @file 
 *  @brief Representation of the optimized k-d tree. 
 *  @author Andreas Nuechter. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Thomas Escher
 */

#ifndef __KD_INDEXED_H__
#define __KD_INDEXED_H__

#include "slam6d/kdparams.h"
#include "slam6d/searchTree.h"
#include "slam6d/kdTreeImpl.h"

#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif


struct Void { };

struct IndexAccessor {
    inline double *operator() (double** data, size_t index) {
        return data[index];
    }
};

struct ParamAccessor {
    inline size_t operator() (double** data, size_t index) {
        return index;
    }
};

/**
 * @brief The optimized k-d tree. 
 * 
 * A kD tree for points, with limited
 * capabilities (find nearest point to
 * a given point, or to a ray).
 **/
class KDtreeIndexed : private KDTreeImpl<double**, size_t, IndexAccessor, size_t, ParamAccessor>
{
public:
  KDtreeIndexed(double **pts, size_t n);
  
  virtual ~KDtreeIndexed();

  virtual size_t FindClosest(double *_p,
						double maxdist2,
						int threadNum = 0) const;

  virtual size_t FindClosestAlongDir(double *_p,
							   double *_dir,
							   double maxdist2,
							   int threadNum = 0) const;
  
  virtual vector<size_t> fixedRangeSearchAlongDir(double *_p,
                  double *_dir,
                  double maxdist2,
                  int threadNum = 0) const;
  
  virtual vector<size_t> fixedRangeSearchBetween2Points(double *_p,
                  double *_dir,
                  double maxdist2,
                  int threadNum = 0) const;

  virtual vector<size_t> kNearestNeighbors(double *_p,
								  int k,
								  int threadNum = 0) const;
  
  virtual vector<size_t> fixedRangeSearch(double *_p,
								 double sqRad2,
								 int threadNum = 0) const;

private:
  double **m_data;
  size_t m_size;

  // constructor initializer list hacks
  size_t* m_temp_indices;
  size_t* prepareTempIndices(size_t n);  
};

#endif
