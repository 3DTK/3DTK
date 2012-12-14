/*
 * icp6Dlumeuler implementation
 *
 * Copyright (C) Andreas Nuechter, Alexandru-Eugen Ichim
 *
 * Released under the GPL version 3.
 *
 */


/** @file 
 *  @brief Implementation of the ICP error function minimization via
 *  linearization using euler angles
 *
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 *  @author Alexandru-Eugen Ichim. Jacobs University Bremen gGmbH, Germany
 *
 */

#include "slam6d/icp6Dlumeuler.h"

#include "slam6d/globals.icc"
#include <iomanip>
using std::ios;
using std::resetiosflags;
using std::setiosflags;

#include "newmat/newmat.h"
#include "newmat/newmatap.h"

using namespace NEWMAT;

/**
 * computes the rotation matrix consisting
 * of a rotation and translation that
 * minimizes the root-mean-square error of the
 * point pairs using linearization with euler angles
 * @param pairs Vector of point pairs (pairs of corresponding points)
 * @param *alignfx The resulting transformation matrix
 * @return Error estimation of the matching (rms)
 */
double icp6D_LUMEULER::Align(const vector<PtPair>& pairs,
                             double *alignfx,
                             const double centroid_m[3],
                             const double centroid_d[3])
{
  // alignxf is filled with the current pose,
  // rPos is the translation,
  // rPosTheta are the 3 euler angles theta_x, theta_y, theta_z
  double rPos[3], rPosTheta[3];
  Matrix4ToEuler(alignfx, rPosTheta, rPos);

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
         << "  using " << std::setw(6) << (int)pairs.size() << " points"
         << endl;
  }

  /////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////

  double x = 0.0, y = 0.0, z = 0.0,
    dx = 0.0, dy = 0.0, dz = 0.0,
    sx = 0.0, sy = 0.0, sz = 0.0,
    xpy = 0.0, xpz = 0.0, ypz = 0.0,
    xy = 0.0, yz = 0.0, xz = 0.0;

  /// MZ = M^T * Z
  ColumnVector MZ(6); MZ = 0.0;
  /// MM = M^T * M
  SymmetricMatrix MM(6); MM = 0.0;
  for(unsigned int i = 0; i < pairs.size(); ++i) {
    /// temporary values that we shall use multiple times in
    //  the subsequent computations
    x = (pairs[i].p1.x + pairs[i].p2.x) / 2.0;
    y = (pairs[i].p1.y + pairs[i].p2.y) / 2.0;
    z = (pairs[i].p1.z + pairs[i].p2.z) / 2.0;
    dx = pairs[i].p1.x - pairs[i].p2.x;
    dy = pairs[i].p1.y - pairs[i].p2.y;
    dz = pairs[i].p1.z - pairs[i].p2.z;

    /// sums of each coordinate
    sx += x;
    sy += y;
    sz += z;

    /// sums of squares of pairs of coordinates
    xpy += x*x + y*y;
    xpz += x*x + z*z;
    ypz += y*y + z*z;

    /// sums of products of pairs of coordinates
    xy += x*y;
    xz += x*z;
    yz += y*z;

    /// incrementally construct M^T * Z
    MZ(1) += dx;
    MZ(2) += dy;
    MZ(3) += dz;
    MZ(4) += -z*dy + y*dz;
    MZ(5) += -y*dx + x*dy;
    MZ(6) +=  z*dx - x*dz;
  }

  /// construct M^T * M
  MM(1,1) = MM(2,2) = MM(3,3) = pairs.size();
  MM(4,4) = ypz;
  MM(5,5) = xpy;
  MM(6,6) = xpz;
  MM(1,5) = MM(5,1) = -sy;
  MM(1,6) = MM(6,1) = sz;
  MM(2,4) = MM(4,2) = -sz;
  MM(2,5) = MM(5,2) = sx;
  MM(3,4) = MM(4,3) = sy;
  MM(3,6) = MM(6,3) = -sx;
  MM(4,5) = MM(5,4) = -xz;
  MM(4,6) = MM(6,4) = -xy;
  MM(5,6) = MM(6,5) = -yz;

  ColumnVector Ehat(6);
  Ehat = MM.i() * MZ;

  double cosx = cos(rPosTheta[0]),
    cosy = cos(rPosTheta[1]),
    cosz = cos(rPosTheta[2]);
  double sinx = sin(rPosTheta[0]),
    siny = sin(rPosTheta[1]),
    sinz = sin(rPosTheta[2]);
  double tx = rPos[0], ty = rPos[1], tz = rPos[2];
  /// create transform matrix of the first scan
  Matrix T1(4, 4); T1 = IdentityMatrix(4);
  T1(1, 4) = tx;
  T1(2, 4) = ty;
  T1(3, 4) = tz;
  T1(1, 1) = cosy*cosz;
  T1(1, 2) = -cosy*sinz;
  T1(1, 3) = siny;
  T1(2, 1) = cosz*sinx*siny + cosx*sinz;
  T1(2, 2) = cosx*cosz-sinx*siny*sinz;
  T1(2, 3) = -cosy*sinx;
  T1(3, 1) = sinx*sinz - cosx*cosz*siny;
  T1(3, 2) = cosz*sinx + cosx*siny*sinz;
  T1(3, 3) = cosx*cosy;

  /// create matrix H
  Matrix H(6, 6); H = IdentityMatrix(6);
  H(1, 5) = -tz*cosx + ty*sinx;
  H(1, 6) = ty*cosx*cosy + tz*cosy*sinx;
  H(2, 4) = tz;
  H(2, 5) = -tx*sinx;
  H(2, 6) = -tx*cosx*cosy + tz*siny;
  H(3, 4) = -ty;
  H(3, 5) = tx*cosx;
  H(3, 6) = -tx*cosy*sinx - ty*siny;
  H(4, 6) = siny;
  H(5, 5) = sinx;
  H(5, 6) = cosx*cosy;
  H(6, 5) = cosx;
  H(6, 6) = -cosy*sinx;

  /// the vector Xhat is the pose estimation of the
  // second scan = the final pose of the first scan
  ColumnVector Xhat(6);
  Xhat << rPos[0] << rPos[1] << rPos[2]
       << rPosTheta[0] << rPosTheta[1] << rPosTheta[2];
  ColumnVector X = Xhat - H.i()*Ehat;

  cosx = cos(X(4)), cosy = cos(X(5)), cosz = cos(X(6));
  sinx = sin(X(4)), siny = sin(X(5)), sinz = sin(X(6));
  /// transform of the second scan as computed so far
  Matrix T2(4, 4); T2 = IdentityMatrix(4);
  T2(1, 4) = X(1);
  T2(2, 4) = X(2);
  T2(3, 4) = X(3);
  T2(1, 1) = cosy*cosz;
  T2(1, 2) = -cosy*sinz;
  T2(1, 3) = siny;
  T2(2, 1) = cosz*sinx*siny + cosx*sinz;
  T2(2, 2) = cosx*cosz-sinx*siny*sinz;
  T2(2, 3) = -cosy*sinx;
  T2(3, 1) = sinx*sinz - cosx*cosz*siny;
  T2(3, 2) = cosz*sinx + cosx*siny*sinz;
  T2(3, 3) = cosx*cosy;

  /// the incremental transform calculated from the absolute poses
  //  of the two scans
  Matrix T_inc = T1 * T2.i();

  /// convert our 4x4 transform to column-wise opengl form
  alignfx[0] = T_inc(1, 1);
  alignfx[1] = T_inc(2, 1);
  alignfx[2] = T_inc(3, 1);
  alignfx[3] = 0.0;
  alignfx[4] = T_inc(1, 2);
  alignfx[5] = T_inc(2, 2);
  alignfx[6] = T_inc(3, 2);
  alignfx[7] = 0.0;
  alignfx[8] = T_inc(1, 3);
  alignfx[9] = T_inc(2, 3);
  alignfx[10]= T_inc(3, 3);
  alignfx[11]= 0.0;
  alignfx[12]= T_inc(1, 4);
  alignfx[13]= T_inc(2, 4);
  alignfx[14]= T_inc(3, 4);
  alignfx[15]= 1.0;

  /////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////

  return error;
}

