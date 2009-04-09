/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Peter Schneider. Institute of Computer Science, University of Koblenz and Landau, Germany.
 */

#ifndef __ICP6DHELIX_H__
#define __ICP6DHELIX_H__

#include "icp6Dminimizer.h"
#include "newmat/newmatio.h"

/**
 * @brief Implementation of the ICP error function minimization via helix-translation
 */
class icp6D_HELIX : public icp6Dminimizer
{
public:
  /** 
   * Constructor 
   */
  icp6D_HELIX(bool quiet = false) : icp6Dminimizer(quiet) {};
  /** 
   * Destructor 
   */
  virtual ~icp6D_HELIX() {};                                  

  double Point_Point_Align(const vector<PtPair>& Pairs, double *alignxf,
					  const double centroid_m[3], const double centroid_d[3]);  
  double Point_Point_Align_Parallel(const int openmp_num_threads, 
				    const unsigned int n[OPENMP_NUM_THREADS],
				    const double sum[OPENMP_NUM_THREADS], 
				    const double centroid_m[OPENMP_NUM_THREADS][3],
				    const double centroid_d[OPENMP_NUM_THREADS][3], 
				    const double Si[OPENMP_NUM_THREADS][9], 
				    double *alignxf);

  static void computeRt(const ColumnVector* ccs, const int vectorOffset, double *alignxf);
};

#endif
