/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#ifndef __ICP6DNAPX_H__
#define __ICP6DNAPX_H__

#include "icp6Dminimizer.h"

/**
 * @brief Implementation of the ICP error function minimization via
          approximation approximation using information from point normals
 */
class icp6D_NAPX : public icp6Dminimizer
{
public:
  /** 
   * Constructor 
   */
  icp6D_NAPX(bool quiet = false) : icp6Dminimizer(quiet) {};

  /** 
   * Destructor 
   */
  virtual ~icp6D_NAPX() {};                                  

  double Align(const vector<PtPair>& Pairs,
			double *alignxf,
			const double centroid_m[3],
			const double centroid_d[3]);
  
  static void computeRt(const double *x, const double *dx, double *alignxf);

  inline int getAlgorithmID() { return 10; }; 
};

#endif
