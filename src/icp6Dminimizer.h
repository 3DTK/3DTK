/** @file 
 *  @brief Implementation of the virtual functor for ICP error function minimization
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __ICP6DMINIMIZER__
#define __ICP6DMINIMIZER__

#include <vector>
using std::vector;

#include "defs.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#include "ptpair.h"

class icp6Dminimizer {

public:
  /** 
   * Constructor 
   */
  icp6Dminimizer(bool quiet = false) { this->quiet = quiet; };
  /** 
   * Destructor 
   */
  virtual ~icp6Dminimizer() {};                                

  /**
   * aligning the point pairs
   */
  virtual double Point_Point_Align(const vector<PtPair>& Pairs, double *alignxf,
							const double centroid_m[3], const double centroid_d[3]) = 0;
  virtual double Point_Point_Align_Parallel(const int openmp_num_threads, 
								    const unsigned int n[OPENMP_NUM_THREADS],
								    const double sum[OPENMP_NUM_THREADS], 
								    const double centroid_m[OPENMP_NUM_THREADS][3],
								    const double centroid_d[OPENMP_NUM_THREADS][3], 
								    const double Si[OPENMP_NUM_THREADS][9], 
								    double *alignxf) = 0;

protected:
  bool quiet; ///< determines the verbosity
};

#endif 
