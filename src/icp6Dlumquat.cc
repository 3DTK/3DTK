/** @file 
 *  @brief Implementation of the ICP error function minimization via
 *  orthonormal matrices
 *
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 *  @author ???. Jacobs University Bremen gGmbH, Germany
 *
 */

#include "icp6Dlumquat.h"

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
double icp6D_LUMQUAT::Point_Point_Align(const vector<PtPair>& pairs, double *alignfx,
								const double centroid_m[3], const double centroid_d[3])
{
  // alignxf is filled with the current pose, t is the translation, quat is the quaternion
  double t[3], quat[4];
  Matrix4ToQuat(alignfx, quat, t);


  double error = 0;
  double sum = 0.0;
	
  // Get centered PtPairs
  double** m = new double*[pairs.size()];
  double** d = new double*[pairs.size()];

  for(unsigned int i = 0; i <  pairs.size(); i++){
    m[i] = new double[3];
    d[i] = new double[3];
    m[i][0] = pairs[i].p1.x - centroid_m[0];
    m[i][1] = pairs[i].p1.y - centroid_m[1];
    m[i][2] = pairs[i].p1.z - centroid_m[2];
    d[i][0] = pairs[i].p2.x - centroid_d[0];
    d[i][1] = pairs[i].p2.y - centroid_d[1];
    d[i][2] = pairs[i].p2.z - centroid_d[2];

    sum += sqr(pairs[i].p1.x - pairs[i].p2.x)
	 + sqr(pairs[i].p1.y - pairs[i].p2.y)
	 + sqr(pairs[i].p1.z - pairs[i].p2.z) ;
    
  }

  error = sqrt(sum / (double)pairs.size());
  
  if (!quiet) {
    cout.setf(ios::basefield);
    cout << "LUMQUAT RMS point-to-point error = "
	    << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
	    << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
	    << std::setw(10) << std::setprecision(7)
	    << error
	    << "  using " << std::setw(6) << (int)pairs.size() << " points" << endl;
  }
  
  cout << "LUMQUAT is not implemented yet!!!" << endl;
  exit(-1);
  // NOTE: remember to return an incremental matrix instead of the global matrix as computed in the paper
   
  for(unsigned int i = 0; i < pairs.size(); i++){
	delete [] m[i];
	delete [] d[i];
  }
  delete [] m;
  delete [] d;

  return error;
}

