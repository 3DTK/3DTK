/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#ifndef __ICP6DLUMEULER_H__
#define __ICP6DLUMEULER_H__

#include "icp6Dminimizer.h"

/**
 * @brief Implementation of the ICP error function minimization via LUM-style
 *        linearization
 */
class icp6D_LUMEULER : public icp6Dminimizer
{
public:
  /** 
   * Constructor 
   */
  icp6D_LUMEULER(bool quiet = false) : icp6Dminimizer(quiet) {};
  
  /** 
   * Destructor 
   */
  virtual ~icp6D_LUMEULER() {};                                

  double Align(const vector<PtPair>& Pairs,
			double *alignxf,
			const double centroid_m[3],
			const double centroid_d[3]);  

  inline int getAlgorithmID() { return 3; }; 
};

#endif
