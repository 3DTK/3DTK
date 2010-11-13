/** @file 
 *  @brief Implementation of the ICP error function minimization via
 *  orthonormal matrices
 *
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 *  @author ???. Jacobs University Bremen gGmbH, Germany
 *
 */

#include "slam6d/icp6Dortho.h"

#include "slam6d/globals.icc"
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
double icp6D_ORTHO::Point_Point_Align(const vector<PtPair>& pairs, double *alignfx,
							   const double centroid_m[3], const double centroid_d[3])
{
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
    cout << "ORTHO RMS point-to-point error = "
	    << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
	    << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
	    << std::setw(10) << std::setprecision(7)
	    << error
	    << "  using " << std::setw(6) << (int)pairs.size() << " points" << endl;
  }
  
  cout << "Orthonormal matrices are not implemented yet!!!" << endl;
  exit(-1);
  
  for(unsigned int i = 0; i < pairs.size(); i++){
	delete [] m[i];
	delete [] d[i];
  }
  delete [] m;
  delete [] d;

  return error;
}

