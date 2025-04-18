/** @file
 *  @brief Representation of the optimized k-d tree.
 *  @author Andreas Nuechter. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Thomas Escher
 */

#ifndef __KD_H__
#define __KD_H__

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

struct PtrAccessor {
    inline double *operator() (Void, double* indices) {
        return indices;
    }
};

/**
 * @brief The optimized k-d tree.
 *
 * A kD tree for points, with limited
 * capabilities (find nearest point to
 * a given point, or to a ray).
 **/
class KDtree : public SearchTree, protected KDTreeImpl<Void, double*, PtrAccessor, double*, PtrAccessor>
{
protected:
  KDtree() {}; /* only supposed to be used by derived classes */

public:
  KDtree(double **pts,
         int n,
         int bucketSize = 20);

  virtual ~KDtree();

  virtual std::vector<double*> CollectPts(int threadNum = 0) const;

  virtual int Remove(double *p,
                     int threadNum = 0); // no const

  // FIXME this file has tab width of 5
  virtual double *FindClosest(double *_p,
						double maxdist2,
						int threadNum = 0) const;

  virtual double *FindClosestAlongDir(double *_p,
							   double *_dir,
							   double maxdist2,
							   int threadNum = 0) const;

  virtual std::vector<Point> fixedRangeSearchAlongDir(double *_p,
                  double *_dir,
                  double maxdist2,
                  int threadNum = 0) const;

  virtual std::vector<Point> fixedRangeSearchBetween2Points(double *_p,
                  double *_dir,
                  double maxdist2,
                  int threadNum = 0) const;

  virtual std::vector<Point> kNearestNeighbors(double *_p,
								  int k,
								  int threadNum = 0) const;

  virtual std::vector<Point> kNearestRangeSearch(double *_p,
								 int k,
								 double sqRad2,
								 int threadNum = 0) const;

  virtual std::vector<Point> fixedRangeSearch(double *_p,
								 double sqRad2,
								 int threadNum = 0) const;

  virtual std::vector<Point> AABBSearch(double *_p,
                                   double* _p0,
                                   int threadNum = 0) const;

  virtual double *segmentSearch_1NearestPoint(double *_p,
          double* _p0, double maxdist2, int threadNum) const;

};



#endif
