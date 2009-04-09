/** @file 
 *  @brief Implementation of the ICP error function minimization via approximation
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "icp6Dapx.h"

#include "globals.icc"
#include <iomanip>
#include <cstring>
using std::ios;
using std::resetiosflags;
using std::setiosflags;

/**
 * computes the rotation matrix consisting
 * of a rotation and translation that
 * minimizes the root-mean-square error
 * of the point pairs, using the <b>approximation</b>
 * sin(x) = x.
 *
 * @param Pairs Vector of point pairs (pairs of corresponding points)
 * @param alignxf The resulting transformation matrix
 * @return Error estimation of the matching (rms)
 */
double icp6D_APX::Point_Point_Align(const vector<PtPair>& Pairs, double *alignxf,
							 const double centroid_m[3], const double centroid_d[3])
{
  int n = Pairs.size();

  // ?!? <= 3
  if (n <= 3) {
    M4identity(alignxf);
    return 0;
  }

  int i;

  double A[3][3];
  double B[3];
  memset(&A[0][0], 0, 9 * sizeof(double));
  memset(&B[0], 0, 3 * sizeof(double));

  double sum = 0;
  double p1[3], p2[3];

  for (i = 0; i < n; i++) {
    p1[0] = Pairs[i].p1.x;
    p1[1] = Pairs[i].p1.y;
    p1[2] = Pairs[i].p1.z;
    p2[0] = Pairs[i].p2.x;
    p2[1] = Pairs[i].p2.y;
    p2[2] = Pairs[i].p2.z;

    double p12[3] = { p1[0] - p2[0], p1[1] - p2[1], p1[2] - p2[2] };
    double p2c[3] = { p2[0] - centroid_d[0], p2[1] - centroid_d[1], p2[2] - centroid_d[2] };

    sum += Len2(p12);

    B[0] += (p12[2]*p2c[1] - p12[1]*p2c[2]);
    B[1] += (p12[0]*p2c[2] - p12[2]*p2c[0]);
    B[2] += (p12[1]*p2c[0] - p12[0]*p2c[1]);
    A[0][0] += (sqr(p2c[1]) + sqr(p2c[2]));
    A[0][1] -= p2c[0] * p2c[1];
    A[0][2] -= p2c[0] * p2c[2];
    A[1][1] += (sqr(p2c[0]) + sqr(p2c[2]));
    A[1][2] -= p2c[1] * p2c[2];
    A[2][2] += (sqr(p2c[0]) + sqr(p2c[1]));
  }

  double error = sqrt(sum / n);
  if (!quiet) {
    cout.setf(ios::basefield);
    cout << "APX RMS point-to-point error = "
	    << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
	    << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
	    << std::setw(10) << std::setprecision(7)
	    << error
	    << "  using " << std::setw(6) << (int)Pairs.size() << " points" << endl;
  }

  // Solve eqns
  double diag[3];
  if (!choldc(A, diag)) {
    printf("Couldn't find transform.\n");
    return -1.0;
  }
  double x[3];
  cholsl(A, diag, B, x);

  // Interpret results
  double sx = x[0];
  double cx = sqrt(1.0 - sx*sx);
  double sy = x[1];
  double cy = sqrt(1.0 - sy*sy);
  double sz = x[2];
  double cz = sqrt(1.0 - sz*sz);

  alignxf[0]  = cy*cz;
  alignxf[1]  = sx*sy*cz + cx*sz;
  alignxf[2]  = -cx*sy*cz + sx*sz;
  alignxf[3]  = 0;
  alignxf[4]  = -cy*sz;
  alignxf[5]  = -sx*sy*sz + cx*cz;
  alignxf[6]  = cx*sy*sz + sx*cz;
  alignxf[7]  = 0;
  alignxf[8]  = sy;
  alignxf[9]  = -sx*cy;
  alignxf[10] = cx*cy;
  alignxf[11] = 0;
  alignxf[12] = centroid_m[0] - alignxf[0]*centroid_d[0] - alignxf[4]*centroid_d[1] - alignxf[8]*centroid_d[2];
  alignxf[13] = centroid_m[1] - alignxf[1]*centroid_d[0] - alignxf[5]*centroid_d[1] - alignxf[9]*centroid_d[2];
  alignxf[14] = centroid_m[2] - alignxf[2]*centroid_d[0] - alignxf[6]*centroid_d[1] - alignxf[10]*centroid_d[2];
  alignxf[15] = 1;

  return error;
}


double icp6D_APX::Point_Point_Align_Parallel(const int openmp_num_threads, 
					     const unsigned int n[OPENMP_NUM_THREADS],
					     const double sum[OPENMP_NUM_THREADS], 
					     const double centroid_m[OPENMP_NUM_THREADS][3],
					     const double centroid_d[OPENMP_NUM_THREADS][3], 
					     const double Si[OPENMP_NUM_THREADS][9], 
					     double *alignxf)
{
  cout << "parallel APX not impelented yet" << endl;
  exit(-1);
}


void icp6D_APX::computeRt(const double *x, const double *dx, double *alignxf)
{
  double sx = x[0];
  double cx = sqrt(1.0 - sx*sx);
  double sy = x[1];
  double cy = sqrt(1.0 - sy*sy);
  double sz = x[2];
  double cz = sqrt(1.0 - sz*sz);
  
  alignxf[0]  = cy*cz;
  alignxf[1]  = sx*sy*cz + cx*sz;
  alignxf[2]  = -cx*sy*cz + sx*sz;
  alignxf[3]  = 0;
  alignxf[4]  = -cy*sz;
  alignxf[5]  = -sx*sy*sz + cx*cz;
  alignxf[6]  = cx*sy*sz + sx*cz;
  alignxf[7]  = 0;
  alignxf[8]  = sy;
  alignxf[9]  = -sx*cy;
  alignxf[10] = cx*cy;
  alignxf[11] = 0;
  alignxf[12] = dx[0];
  alignxf[13] = dx[1];
  alignxf[14] = dx[2];
  alignxf[15] = 1;
}
