/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __ICP6DSVD_H__
#define __ICP6DSVD_H__

#include "icp6Dminimizer.h"

/**
 * @brief Implementation of the ICP error function minimization via singular value decomposition
 */
class icp6D_SVD : public icp6Dminimizer
{
public:
  /** 
   * Constructor 
   */
  icp6D_SVD(bool quiet = false) : icp6Dminimizer(quiet) {};
  /** 
   * Destructor 
   */
  virtual ~icp6D_SVD() {};                                

  double Point_Point_Align(const vector<PtPair>& Pairs, double *alignxf,
					  const double centroid_m[3], const double centroid_d[3]);  
  double Point_Point_Align_Parallel(const int openmp_num_threads, 
				    const unsigned int n[OPENMP_NUM_THREADS],
				    const double sum[OPENMP_NUM_THREADS], 
				    const double centroid_m[OPENMP_NUM_THREADS][3],
				    const double centroid_d[OPENMP_NUM_THREADS][3], 
				    const double Si[OPENMP_NUM_THREADS][9], double *alignxf);
};

#endif
