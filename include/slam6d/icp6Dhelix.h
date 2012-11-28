/** @file 
 *  @brief Definition of the ICP error function minimization
 *  @author Peter Schneider. Inst. of CS, Univ. of Koblenz and Landau, Germany.
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#ifndef __ICP6DHELIX_H__
#define __ICP6DHELIX_H__

#include "icp6Dminimizer.h"
#include "newmat/newmatio.h"
using namespace NEWMAT;

/**
 * @brief Implementation of the ICP error function minimization via
 *        helix-translation
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

  double Align(const vector<PtPair>& Pairs,
			double *alignxf,
			const double centroid_m[3],
			const double centroid_d[3]);  

  static void computeRt(const ColumnVector* ccs,
				    const int vectorOffset,
				    double *alignxf);

  inline int getAlgorithmID() { return 5; }; 
  
};

#endif
