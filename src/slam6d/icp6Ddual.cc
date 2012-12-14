/*
 * icp6Ddual implementation
 *
 * Copyright (C) Andreas Nuechter, Alexandru-Eugen Ichim
 *
 * Released under the GPL version 3.
 *
 */

/** @file 
 *  @brief Implementation of the ICP error function minimization
 *  via dual quaternions
 *
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 *  @author Alexandru-Eugen Ichim. Jacobs University Bremen gGmbH, Germany
 *
 */

#include "slam6d/icp6Ddual.h"
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
 * point pairs using dual quaternions
 * @param pairs Vector of point pairs (pairs of corresponding points)
 * @param *alignfx The resulting transformation matrix
 * @return Error estimation of the matching (rms)
 */
double icp6D_DUAL::Align(const vector<PtPair>& pairs,
                         double *alignfx,
                         const double centroid_m[3],
                         const double centroid_d[3])
{
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
    cout << "DUALQUAT RMS point-to-point error = "
         << resetiosflags(ios::adjustfield)
         << setiosflags(ios::internal)
         << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
         << std::setw(10) << std::setprecision(7)
         << error
         << "  using " << std::setw(6) << (int)pairs.size()
         << " points" << endl;
  }

  /////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////

  Matrix C1(4, 4); C1 = 0.0;
  Matrix C2(4, 4); C2 = 0.0;

  /// build up matrices C1 and C2
  for(unsigned int i = 0; i < pairs.size(); ++i) {
    ColumnVector m(3), d(3);
    m << pairs[i].p1.x << pairs[i].p1.y << pairs[i].p1.z;
    d << pairs[i].p2.x << pairs[i].p2.y << pairs[i].p2.z;
    Matrix Cm(3, 3);
    Cm << 0 << -m(3) << m(2) << m(3) << 0 << -m(1) << -m(2) << m(1) << 0;
    Matrix Cd(3, 3);
    Cd << 0 << -d(3) << d(2) << d(3) << 0 << -d(1) << -d(2) << d(1) << 0;

    C1.SubMatrix(1, 1, 1, 1) += m.t()*d;
    C1.SubMatrix(1, 1, 2, 4) += -m.t()*Cd;
    C1.SubMatrix(2, 4, 1, 1) += -Cm*d;
    C1.SubMatrix(2, 4, 2, 4) += m*d.t() + Cm*Cd;

    C2.SubMatrix(1, 1, 2, 4) += -d.t() + m.t();
    C2.SubMatrix(2, 4, 1, 1) += d - m;
    C2.SubMatrix(2, 4, 2, 4) += -Cd - Cm;
  }

  /// the sums need to be multiplied by a scalar
  C1 = C1 * (-2);
  C2 = C2 * 2;

  /// matrix A from C1 and C2
  Matrix A = (C2.t()*C2 * 1.0/(2*pairs.size()) - C1 - C1.t()) * 0.5;

  /// the quaternion qdot is the eigenvector of matrix A
  //  corresponding to the largest eigenvalue,
  /// which is always found in the first column of matrix U
  Matrix U(4, 4); DiagonalMatrix D(4);
  SVD(A, D, U);
  ColumnVector qdot = U.Column(1);
  ColumnVector q(3); q << qdot(2) << qdot(3) << qdot(4);
  Matrix Cq(3, 3);
  Cq << 0 << -q(3) << q(2) << q(3) << 0 << -q(1) << -q(2) << q(1) << 0;

  ColumnVector s = C2*qdot * (-1.0)/(2*pairs.size());

  Matrix Q(4, 4);
  Q(1, 1) = qdot(1);
  Q.SubMatrix(1, 1, 2, 4) = q.t();
  Q.SubMatrix(2, 4, 1, 1) = -q;
  Q.SubMatrix(2, 4, 2, 4) = IdentityMatrix(3)*qdot(1) + Cq;

  /// get the translation from the double quaternion
  ColumnVector p = Q*s;
  ColumnVector translation(3); translation << p(2) << p(3) << p(4);

  /// compute the rotation matrix from the quaternion
  ColumnVector qtqx = q.t()*q;
  Matrix R = IdentityMatrix(3) *
    (qdot(1)*qdot(1) - qtqx(1)) + q*q.t()*2 + Cq*qdot(1)*2;

  /// create the final transformation matrix (OpenGL column-wise ordering)
  alignfx[0] = R(1, 1);
  alignfx[1] = R(2, 1);
  alignfx[2] = R(3, 1);
  alignfx[3] = 0.0;
  alignfx[4] = R(1, 2);
  alignfx[5] = R(2, 2);
  alignfx[6] = R(3, 2);
  alignfx[7] = 0.0;
  alignfx[8] = R(1, 3);
  alignfx[9] = R(2, 3);
  alignfx[10]= R(3, 3);
  alignfx[11]= 0.0;
  alignfx[12]= translation(1);
  alignfx[13]= translation(2);
  alignfx[14]= translation(3);
  alignfx[15]= 1.0;

  /////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////

  return error;
}

