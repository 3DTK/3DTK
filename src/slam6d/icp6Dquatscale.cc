/*
 * icp6Dquatscale implementation
 *
 * Copyright (C) Flavia Grosan, Alex Tandrau, Dorit Borrmann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/** @file 
 *  @brief Implementation of the ICP error function minimization via
 *         quaternions and scale factor
 *  @author Flavia Grosan, Jacobs University Bremen gGmbH, Germany.
 *  @author Alex Tandrau. Jacobs University Bremen gGmbH, Germany.
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany.
 *  @author Dorit borrmann. Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/icp6Dquatscale.h"

#include "slam6d/globals.icc"
#include <iomanip>
using std::ios;
using std::resetiosflags;
using std::setiosflags;
#include <cfloat>
#include <cmath>
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

/**
 * computes the rotation matrix consisting
 * of a rotation and translation that
 * minimizes the root-mean-square error of the
 * point pairs using the Quaternion method of Horn
 * PARAMETERS
 * vector of point pairs, rotation matrix
 * @param pairs Vector of point pairs (pairs of corresponding points)
 * @param *alignfx The resulting transformation matrix
 * @return Error estimation of the matching (rms)
 */
double icp6D_QUAT_SCALE::Align(const vector<PtPair>& pairs,
                               double *alignfx,
                               const double centroid_m[3],
                               const double centroid_d[3])
{
  int n = pairs.size();

  double sum = 0.0;
  double sums[2]; sums[0] = sums[1] = 0.0;
 
  // the quaternion
  double q[7];

  double S[3][3]; // Cross Covariance Matrix
  double Q[4][4];
  int i,j;

  // calculate the cross covariance matrix
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      S[i][j] = 0;

  for (i=0; i<n; i++) {
    sum += sqr(pairs[i].p1.x - pairs[i].p2.x) +
      sqr(pairs[i].p1.y - pairs[i].p2.y) +
      sqr(pairs[i].p1.z - pairs[i].p2.z);

    sums[0] += sqr(pairs[i].p1.x - centroid_m[0]) +
      sqr(pairs[i].p1.y - centroid_m[1]) +
      sqr(pairs[i].p1.z - centroid_m[2]);
          
    sums[1] += sqr(pairs[i].p2.x - centroid_d[0]) +
      sqr(pairs[i].p2.y - centroid_d[1]) +
      sqr(pairs[i].p2.z - centroid_d[2]);
                                   
    S[0][0] += pairs[i].p2.x * pairs[i].p1.x;
    S[0][1] += pairs[i].p2.x * pairs[i].p1.y;
    S[0][2] += pairs[i].p2.x * pairs[i].p1.z;
    S[1][0] += pairs[i].p2.y * pairs[i].p1.x;
    S[1][1] += pairs[i].p2.y * pairs[i].p1.y;
    S[1][2] += pairs[i].p2.y * pairs[i].p1.z;
    S[2][0] += pairs[i].p2.z * pairs[i].p1.x;
    S[2][1] += pairs[i].p2.z * pairs[i].p1.y;
    S[2][2] += pairs[i].p2.z * pairs[i].p1.z;
  }

  double error = sqrt(sum / n);
  if (!quiet) {
    cout.setf(ios::basefield);
    cout << "QUAT SCALE RMS point-to-point error = "
         << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
         << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
         << std::setw(10) << std::setprecision(7)
         << error
         << "  using " << std::setw(6) << (int)pairs.size() << " points"
         << endl;
  }

  double fact = 1 / double(n);
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      S[i][j] *= fact;
  S[0][0] -= centroid_d[0] * centroid_m[0];
  S[0][1] -= centroid_d[0] * centroid_m[1];
  S[0][2] -= centroid_d[0] * centroid_m[2];
  S[1][0] -= centroid_d[1] * centroid_m[0];
  S[1][1] -= centroid_d[1] * centroid_m[1];
  S[1][2] -= centroid_d[1] * centroid_m[2];
  S[2][0] -= centroid_d[2] * centroid_m[0];
  S[2][1] -= centroid_d[2] * centroid_m[1];
  S[2][2] -= centroid_d[2] * centroid_m[2];
  
  // calculate the 4x4 symmetric matrix Q
  double trace = S[0][0] + S[1][1] + S[2][2];
  double A23 = S[1][2] - S[2][1];
  double A31 = S[2][0] - S[0][2];
  double A12 = S[0][1] - S[1][0];

  Q[0][0] = trace;
  Q[0][1] = Q[1][0] = A23;
  Q[0][2] = Q[2][0] = A31;
  Q[0][3] = Q[3][0] = A12;
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      Q[i+1][j+1] = S[i][j] + S[j][i] - ( i==j ? trace : 0);
  
  maxEigenVector(Q, q);

  // calculate the rotation matrix
  double m[3][3]; // rot matrix
  quaternion2matrix(q, m);

  M4identity(alignfx);

  double scale_s = sqrt(sums[0] / sums[1]);

  alignfx[0] = m[0][0] * scale_s;
  alignfx[1] = m[1][0] * scale_s;
  alignfx[2] = m[2][0] * scale_s;
  alignfx[3] = 0.0;
  alignfx[4] = m[0][1] * scale_s;
  alignfx[5] = m[1][1] * scale_s;
  alignfx[6] = m[2][1] * scale_s;
  alignfx[7] = 0.0;
  alignfx[8] = m[0][2] * scale_s;
  alignfx[9] = m[1][2] * scale_s;
  alignfx[10] = m[2][2] * scale_s;
  alignfx[11] = 0.0;
  
  // calculate the translation vector, 
  alignfx[12] = centroid_m[0]
    - m[0][0]*centroid_d[0]*scale_s
    - m[0][1]*scale_s*centroid_d[1]
    - m[0][2]*centroid_d[2]*scale_s; 
  alignfx[13] = centroid_m[1]
    - m[1][0]*centroid_d[0]*scale_s
    - m[1][1]*scale_s*centroid_d[1]
    - m[1][2]*centroid_d[2]*scale_s;
  alignfx[14] = centroid_m[2]
    - m[2][0]*centroid_d[0]*scale_s
    - m[2][1]*scale_s*centroid_d[1]
    - m[2][2]*centroid_d[2]*scale_s;

  return error;
}
