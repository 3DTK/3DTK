/** @file 
 *  @brief Implementation of the ICP error function minimization via
 *  orthonormal matrices
 *
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 *  @author ???. Jacobs University Bremen gGmbH, Germany
 *
 */

#include "icp6Dlumeuler.h"

#include "globals.icc"
#include <iomanip>
using std::ios;
using std::resetiosflags;
using std::setiosflags;

#include "newmat/newmat.h"
#include "newmat/newmatap.h"

/**
 * computes the rotation matrix consisting
 * of a rotation and translation that
 * minimizes the root-mean-square error of the
 * point pairs using the SVD PARAMETERS
 * vector of point pairs, rotation matrix
 * @param pairs Vector of point pairs (pairs of corresponding points)
 * @param *alignfx The resulting transformation matrix
 * @return Error estimation of the matching (rms)
*/
double icp6D_LUMEULER::Point_Point_Align(const vector<PtPair>& pairs, double *alignfx,
								 const double centroid_m[3], const double centroid_d[3])
{
  // alignxf is filled with the current pose, rPos is the translation, rPosTheta are the 3 euler angles theta_x, theta_y, theta_z
  double rPos[3], rPosTheta[3];
  Matrix4ToEuler(alignfx, rPos, rPosTheta);

  double error = 0;
  double sum = 0.0;
	
  for(unsigned int i = 0; i <  pairs.size(); i++){
    sum += sqr(pairs[i].p1.x - pairs[i].p2.x)
	 + sqr(pairs[i].p1.y - pairs[i].p2.y)
	 + sqr(pairs[i].p1.z - pairs[i].p2.z) ;
  }

  error = sqrt(sum / (double)pairs.size());
  
  if (!quiet) {
    cout.setf(ios::basefield);
    cout << "LUMEULER RMS point-to-point error = "
	    << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
	    << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
	    << std::setw(10) << std::setprecision(7)
	    << error
	    << "  using " << std::setw(6) << (int)pairs.size() << " points" << endl;
  }
  
  cout << "LUMEULER is not implemented yet!!!" << endl;
  exit(-1);
  // NOTE: remember to return an incremental matrix instead of the global matrix as computed in the paper
  
  return error;
}

