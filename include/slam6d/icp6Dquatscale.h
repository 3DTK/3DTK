/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Flavia Grosan, Alex Tandrau. Jacobs University Bremen, Germany.
 */

#ifndef __ICP6DQUATSCALE_H__
#define __ICP6DQUATSCALE_H__

#include "icp6Dminimizer.h"

/**
 * @brief Implementation of the ICP error function minimization via quaternions
 */
class icp6D_QUAT_SCALE : public icp6Dminimizer
{
public:
  /** constructor */
  icp6D_QUAT_SCALE(bool quiet = false) : icp6Dminimizer(quiet) {};
  /** destructor */
  virtual ~icp6D_QUAT_SCALE() {};                                  

  double Point_Point_Align(const vector<PtPair>& Pairs, double *alignxf,
					  const double centroid_m[3], const double centroid_d[3]);
  inline int getAlgorithmID() { return 9; }; 

private:
  void   quaternion2matrix(double *q, double m[3][3]);
  int    ferrari(double a, double b, double c, double d, double rts[4]);
  int    qudrtc(double b, double c, double rts[4]);
  double cubic(double p, double q, double r);
  void   maxEigenVector(double Q[4][4], double ev[4]);  
  void   characteristicPol(double Q[4][4], double c[4]);    
};

#endif
