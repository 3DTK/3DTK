/*
 * icp6Dlumquat implementation
 *
 * Copyright (C) Andreas Nuechter, Alexandru-Eugen Ichim
 *
 * Released under the GPL version 3.
 *
 */


/** @file 
 *  @brief Implementation of the ICP error function minimization via
 *  via linearization with quaternions
 *
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 *  @author Alexandru-Eugen Ichim. Jacobs University Bremen gGmbH, Germany
 *
 */

#include "slam6d/icp6Dlumquat.h"

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
 * point pairs using linearization with quarernions
 * @param pairs Vector of point pairs (pairs of corresponding points)
 * @param *alignfx The resulting transformation matrix
 * @return Error estimation of the matching (rms)
 */
double icp6D_LUMQUAT::Align(const vector<PtPair>& pairs,
                            double *alignfx,
                            const double centroid_m[3],
                            const double centroid_d[3])
{
  // alignfx is filled with the current pose, t is the translation,
  // quat is the quaternion
  double t[3], quat[4];
  Matrix4ToQuat(alignfx, quat, t);

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
    cout << "LUMQUAT RMS point-to-point error = "
         << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
         << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
         << std::setw(10) << std::setprecision(7)
         << error
         << "  using " << std::setw(6) << (int)pairs.size() << " points"
         << endl;
  }

  /////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////

  /// MZ = M^T * Z
  ColumnVector MZ(7); MZ = 0.0;
  /// MM = M^T * M
  Matrix MM(7, 7); MM = 0.0; 
  double x = 0.0, y = 0.0, z = 0.0,
    dx = 0.0, dy = 0.0, dz = 0.0,
    sx = 0.0, sy = 0.0, sz = 0.0,
    xpy = 0.0, xpz = 0.0, ypz = 0.0,
    xpypz = 0.0, xy = 0.0, yz = 0.0, xz = 0.0;

  for(unsigned int i = 0; i < pairs.size(); ++i) {
    /// temporary values that we shall use multiple times in the
    //  subsequent computations

    x = (pairs[i].p1.x + pairs[i].p1.x) / 2.0;
    y = (pairs[i].p1.y + pairs[i].p2.y) / 2.0;
    z = (pairs[i].p1.z + pairs[i].p2.z) / 2.0;
    dx = pairs[i].p1.x - pairs[i].p2.x;
    dy = pairs[i].p1.y - pairs[i].p2.y;
    dz = pairs[i].p1.z - pairs[i].p2.z;

    /// sums of each coordinate
    sx += x;
    sy += y;
    sz += z;

    /// sums of squares of coordinates
    xpy += x*x + y*y;
    xpz += x*x + z*z;
    ypz += y*y + z*z;
    xpypz += x*x + y*y + z*z;

    /// sums of products of pairs of coordinates
    xy += x*y;
    xz += x*z;
    yz += y*z;

    /// incrementally construct matrix M^T * Z
    MZ(1) += dx;
    MZ(2) += dy;
    MZ(3) += dz;
    MZ(4) += x*dx + y*dy + z*dz;
    MZ(5) += z*dy - y*dz;
    MZ(6) += x*dz - z*dx;
    MZ(7) += y*dx - x*dy;
  }

  /// construct M^ * M
  MM(1,1) = MM(2,2) = MM(3,3) = pairs.size();
  MM(4,4) = xpypz;
  MM(5,5) = ypz;
  MM(6,6) = xpz;
  MM(7,7) = xpy;
  MM(1,4) = MM(4,1) = sx;
  MM(1,6) = MM(6,1) = -sz;
  MM(1,7) = MM(7,1) = sy;
  MM(2,4) = MM(4,2) = sy;
  MM(2,5) = MM(5,2) = sz;
  MM(2,7) = MM(7,2) = -sx;
  MM(3,4) = MM(4,3) = sz;
  MM(3,5) = MM(5,3) = -sy;
  MM(3,6) = MM(6,3) = sx;
  MM(5,6) = MM(6,5) = -xy;
  MM(5,7) = MM(7,5) = -xz;
  MM(6,7) = MM(7,6) = -yz;

  ColumnVector Ehat = MM.i() * MZ ;

  /// construct the auxiliary matrices U and T needed to build up H
  double p = quat[0], q = quat[1], r = quat[2], s = quat[3];
  x = t[0], y = t[1], z = t[2];
  Matrix U(4, 4); 
  U << p << q << r << s
    << q <<-p << s <<-r
    << r <<-s <<-p << q
    << s << r <<-q <<-p;

  Matrix T(3, 4);
  T << p*x + s*y - r*z << q*x + r*y + s*z << r*x - q*y + p*z << s*x - p*y - q*z
    <<-s*x + p*y + q*z <<-r*x + q*y - p*z << q*x + r*y + s*z << p*x + s*y - r*z
    << r*x - q*y + p*z <<-s*x + p*y + q*z <<-p*x - s*y + r*z << q*x + r*y - s*z;
  /// now actually compose matrix H
  Matrix H(7, 7); H = 0.0;
  H.SubMatrix(1, 3, 1, 3) = IdentityMatrix(3);
  H.SubMatrix(1, 3, 4, 7) = T *(-2);
  H.SubMatrix(4, 7, 4, 7) = U * 2;


  /// Xhat is the pose estimate for the second scan - use the
  //  pose of the first scan
  ColumnVector Xhat(7); Xhat << x << y << z << p << q << r << s;

  /// create a transformation matrix for the first scan with the given values
  Matrix T1(4, 4); T1 = 0.0;
  Matrix Cq(3, 3);
  ColumnVector qvec(3); qvec << q << r << s;
  Cq << 0 << -qvec(3) << qvec(2) << qvec(3)
     << 0 << -qvec(1) << -qvec(2) << qvec(1)
     << 0;
  ColumnVector qtq(1); qtq = qvec.t() *qvec * (-1);
  T1.SubMatrix(1, 3, 1, 3) = IdentityMatrix(3)
    * (p*p + qtq(1)) + qvec*qvec.t()*2 + Cq *2*p;
  T1(1, 4) = x;
  T1(2, 4) = y;
  T1(3, 4) = z;
  T1(4, 4) = 1;


  /// the translation and quaternion for the second scan are computed
  ColumnVector X = Xhat - H.i() * Ehat;


  /// create the transformation matrix for the second scan
  x = X(1), y = X(2), z = X(3);
  p = X(4); q = X(5); r = X(6); s = X(7);
  Matrix T2(4, 4); T2 = 0.0;
  qvec = 0.0; qvec << q << r << s;
  Cq = 0.0;
  Cq << 0 << -qvec(3) << qvec(2) << qvec(3)
     << 0 << -qvec(1) << -qvec(2) << qvec(1)
	<< 0;
  qtq = qvec.t() *qvec * (-1);
  T2.SubMatrix(1, 3, 1, 3) = IdentityMatrix(3) * (p*p + qtq(1))
    + qvec*qvec.t()*2 + Cq *2*p;
  T2(1, 4) = x;
  T2(2, 4) = y;
  T2(3, 4) = z;
  T2(4, 4) = 1;

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

