/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 *  @author ???, Jacobs University Bremen gGmbH, Germany
 */

#ifndef __ICP6DDUALQUAT_H__
#define __ICP6DDUALQUAT_H__

#include "icp6Dminimizer.h"

/**
 * @brief Implementation of the ICP error function minimization via singular value decomposition
 */
class icp6D_DUAL : public icp6Dminimizer
{
public:
  /** 
   * Constructor 
   */
  icp6D_DUAL(bool quiet = false) : icp6Dminimizer(quiet) {};
  /** 
   * Destructor 
   */
  virtual ~icp6D_DUAL() {};                                

  double Point_Point_Align(const vector<PtPair>& Pairs, double *alignxf,
					  const double centroid_m[3], const double centroid_d[3]);  

  inline int getAlgorithmID() { return 4; }; 
};

#endif
