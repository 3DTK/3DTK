/*
 * icp6Dnapx implementation
 *
 * Copyright (C) Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/**
 *  @file 
 *  @brief Implementation of the ICP error function minimization via
 *         approximation using information from point normals
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#include "slam6d/icp6Dnapx.h"

#include "slam6d/globals.icc"
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
double icp6D_NAPX::Align(const vector<PtPair>& Pairs,
                         double *alignxf,
                         const double centroid_m[3],
                         const double centroid_d[3])
{
  int n = Pairs.size();

  double A[6][6];
  double B[6];
  memset(&A[0][0], 0, 36 * sizeof(double));
  memset(&B[0], 0, 6 * sizeof(double));

  double sum = 0;
  double p1[3], p2[3], norm[3];

  for (int i=0; i < n; i++) {
    p1[0] = Pairs[i].p1.x;
    p1[1] = Pairs[i].p1.y;
    p1[2] = Pairs[i].p1.z;
    p2[0] = Pairs[i].p2.x;
    p2[1] = Pairs[i].p2.y;
    p2[2] = Pairs[i].p2.z;
    norm[0] = Pairs[i].p2.nx;
    norm[1] = Pairs[i].p2.ny;
    norm[2] = Pairs[i].p2.nz;

    double d = (p1[0] - p2[0]) * norm[0]
      + (p1[1] - p2[1]) * norm[1]
      + (p1[2] - p2[2]) * norm[2];
    double p2c[3] = { p2[0] - centroid_d[0], p2[1] - centroid_d[1],
                      p2[2] - centroid_d[2] };
    double c[3];
    Cross(p2c, norm, c);

    sum += d * d;
    B[0] += c[0];
    B[1] += c[1];
    B[2] += c[2];
    B[3] += norm[0];
    B[4] += norm[1];
    B[5] += norm[2];
    A[0][0] += c[0] * c[0];
    A[0][1] += c[0] * c[1];
    A[0][2] += c[0] * c[2];
    A[0][3] += c[0] * norm[0];
    A[0][4] += c[0] * norm[1];
    A[0][5] += c[0] * norm[2];
    A[1][1] += c[1] * c[1];
    A[1][2] += c[1] * c[2];
    A[1][3] += c[1] * norm[0];
    A[1][4] += c[1] * norm[1];
    A[1][5] += c[1] * norm[2];
    A[2][2] += c[2] * c[2];
    A[2][3] += c[2] * norm[0];
    A[2][4] += c[2] * norm[1];
    A[2][5] += c[2] * norm[2];
    A[3][3] += norm[0] * norm[0];
    A[3][4] += norm[0] * norm[1];
    A[3][5] += norm[0] * norm[2];
    A[4][4] += norm[1] * norm[1];
    A[4][5] += norm[1] * norm[2];
    A[5][5] += norm[2] * norm[2];
  }

  double error = sqrt(sum / n);
  if (!quiet) {
    cout.setf(ios::basefield);
    cout << "APX RMS point-to-plane error = "
         << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
         << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
         << std::setw(10) << std::setprecision(7)
         << error
         << "  using " << std::setw(6) << (int)Pairs.size()
         << " points" << endl;
  }

  // Solve eqns
  double diag[6];
  if (!choldc(A, diag)) {
    printf("Couldn't find transform.\n");
    return -1.0f;
  }

  double x[6];
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
  alignxf[12] = x[3] + centroid_d[0] - alignxf[0]*centroid_d[0] -
    alignxf[4]*centroid_d[1] - alignxf[8]*centroid_d[2];
  alignxf[13] = x[4] + centroid_d[1] - alignxf[1]*centroid_d[0] -
    alignxf[5]*centroid_d[1] - alignxf[9]*centroid_d[2];
  alignxf[14] = x[5] + centroid_d[2] - alignxf[2]*centroid_d[0] -
    alignxf[6]*centroid_d[1] - alignxf[10]*centroid_d[2];
  alignxf[15] = 1;
  
  return error;
}



void icp6D_NAPX::computeRt(const double *x, const double *dx, double *alignxf)
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
