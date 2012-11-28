/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#ifndef __ICP6DLUMQUAT_H__
#define __ICP6DLUMQUAT_H__

#include "icp6Dminimizer.h"

/**
 * @brief Implementation of the ICP error function minimization via LUM
 *        with quaternions
 */
class icp6D_LUMQUAT : public icp6Dminimizer
{
public:
  /** 
   * Constructor 
   */
  icp6D_LUMQUAT(bool quiet = false) : icp6Dminimizer(quiet) {};

  /** 
   * Destructor 
   */
  virtual ~icp6D_LUMQUAT() {};                                

  double Align(const vector<PtPair>& Pairs,
			double *alignxf,
			const double centroid_m[3],
			const double centroid_d[3]);  

  inline int getAlgorithmID() { return 8; }; 
};

#endif
