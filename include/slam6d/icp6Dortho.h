/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 *  @author Christian Mueller (Knut). Inst. of CS, Univ. of Osnabrueck, Germany.
 */

#ifndef __ICP6DORTHO_H__
#define __ICP6DORTHO_H__

#include "icp6Dminimizer.h"

/**
 * @brief Implementation of the ICP error function minimization via
 *        orthonormal matrices
 */
class icp6D_ORTHO : public icp6Dminimizer
{
public:
  /** 
   * Constructor 
   */
  icp6D_ORTHO(bool quiet = false) : icp6Dminimizer(quiet) {};
  
  /** 
   * Destructor 
   */
  virtual ~icp6D_ORTHO() {};                                

  double Align(const vector<PtPair>& Pairs,
			double *alignxf,
			const double centroid_m[3],
			const double centroid_d[3]);  

  inline int getAlgorithmID() { return 3; }; 
};

#endif
