/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany.
 */

#ifndef __ICP6DQUAT_H__
#define __ICP6DQUAT_H__

#include "icp6Dminimizer.h"

/**
 * @brief Implementation of the ICP error function minimization via quaternions
 */
class icp6D_QUAT : public icp6Dminimizer
{
public:
  /** constructor */
  icp6D_QUAT(bool quiet = false) : icp6Dminimizer(quiet) {};
  
  /** destructor */
  virtual ~icp6D_QUAT() {};                                  

  double Align(const vector<PtPair>& Pairs,
			double *alignxf,
			const double centroid_m[3],
			const double centroid_d[3]);
  
  double Align_Parallel(const int openmp_num_threads, 
				    const unsigned int n[OPENMP_NUM_THREADS],
				    const double sum[OPENMP_NUM_THREADS], 
				    const double centroid_m[OPENMP_NUM_THREADS][3],
				    const double centroid_d[OPENMP_NUM_THREADS][3], 
				    const double Si[OPENMP_NUM_THREADS][9],
				    double *alignxf);
  
  inline int getAlgorithmID() { return 1; }; 

protected:
  void   quaternion2matrix(double *q, double m[3][3]);
  int    ferrari(double a, double b, double c, double d, double rts[4]);
  int    qudrtc(double b, double c, double rts[4]);
  double cubic(double p, double q, double r);
  void   maxEigenVector(double Q[4][4], double ev[4]);  
  void   characteristicPol(double Q[4][4], double c[4]);    
};

#endif
