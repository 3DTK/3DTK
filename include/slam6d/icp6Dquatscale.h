/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Flavia Grosan, Jacobs University Bremen gGmbH, Germany.
 *  @author Alex Tandrau, Jacobs University Bremen gGmbH, Germany.
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany.
 */

#ifndef __ICP6DQUATSCALE_H__
#define __ICP6DQUATSCALE_H__

#include "icp6Dquat.h"

/**
 * @brief Implementation of the ICP error function minimization via
 *        quaternions cosidering scale
 */
class icp6D_QUAT_SCALE : public icp6D_QUAT
{
public:
  /** constructor */
  icp6D_QUAT_SCALE(bool quiet = false) : icp6D_QUAT(quiet) {};
  
  /** destructor */
  virtual ~icp6D_QUAT_SCALE() {};                                  

  double Align(const vector<PtPair>& Pairs,
			double *alignxf,
			const double centroid_m[3],
			const double centroid_d[3]);
  
  inline int getAlgorithmID() { return 9; }; 

};

#endif
