/*
 * icp6Dquat implementation
 *
 * Copyright (C) Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/** @file
 *  @brief Implementation of the ICP error function minimization via quaternions
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany.
 */

#include "slam6d/icp6Dquat.h"

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

#ifdef _MSC_VER
#define cbrt(x) pow(x,1/3)
#endif

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
double icp6D_QUAT::Align(const vector<PtPair>& pairs,
                         double *alignfx,
                         const double centroid_m[3],
                         const double centroid_d[3])
{
  int n = pairs.size();

  double sum = 0.0;

  // the quaternion
  double q[7];

  double S[3][3];
  double Q[4][4];
  int i,j;

  // calculate the cross covariance matrix
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      S[i][j] = 0;
  for (i=0; i<n; i++) {

    sum += sqr(pairs[i].p1.x - pairs[i].p2.x)
      + sqr(pairs[i].p1.y - pairs[i].p2.y)
      + sqr(pairs[i].p1.z - pairs[i].p2.z) ;
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
    cout << "QUAT RMS point-to-point error = "
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

  alignfx[0] = m[0][0];
  alignfx[1] = m[1][0];
  alignfx[2] = m[2][0];
  alignfx[3] = 0.0;
  alignfx[4] = m[0][1];
  alignfx[5] = m[1][1];
  alignfx[6] = m[2][1];
  alignfx[7] = 0.0;
  alignfx[8] = m[0][2];
  alignfx[9] = m[1][2];
  alignfx[10] = m[2][2];
  alignfx[11] = 0.0;

  // calculate the translation vector,
  alignfx[12] = centroid_m[0]
    - m[0][0]*centroid_d[0] - m[0][1]*centroid_d[1] - m[0][2]*centroid_d[2];
  alignfx[13] = centroid_m[1]
    - m[1][0]*centroid_d[0]
    - m[1][1]*centroid_d[1] - m[1][2]*centroid_d[2];
  alignfx[14] = centroid_m[2]
    - m[2][0]*centroid_d[0] - m[2][1]*centroid_d[1] - m[2][2]*centroid_d[2];

  return error;
}


// transform a quaternion to a rotation matrix
void icp6D_QUAT::quaternion2matrix(double *q, double m[3][3])
{
  double q00 = q[0]*q[0];
  double q11 = q[1]*q[1];
  double q22 = q[2]*q[2];
  double q33 = q[3]*q[3];
  double q03 = q[0]*q[3];
  double q13 = q[1]*q[3];
  double q23 = q[2]*q[3];
  double q02 = q[0]*q[2];
  double q12 = q[1]*q[2];
  double q01 = q[0]*q[1];
  m[0][0] = q00 + q11 - q22 - q33;
  m[1][1] = q00 - q11 + q22 - q33;
  m[2][2] = q00 - q11 - q22 + q33;
  m[0][1] = 2.0*(q12-q03);
  m[1][0] = 2.0*(q12+q03);
  m[0][2] = 2.0*(q13+q02);
  m[2][0] = 2.0*(q13-q02);
  m[1][2] = 2.0*(q23-q01);
  m[2][1] = 2.0*(q23+q01);
}

int icp6D_QUAT::ferrari(double a, double b, double c, double d,     double rts[4])
/*
  solve the quartic equation -

  x**4 + a*x**3 + b*x**2 + c*x + d = 0

  input -
  a,b,c,e - coeffs of equation.

  output -
  nquar - number of real roots.
  rts - array of root values.

  method :  Ferrari - Lagrange
  Theory of Equations, H.W. Turnbull p. 140 (1947)

  calls  cubic, qudrtc
*/
{
  int nquar,n1,n2;
  double asq,ainv2;
  double v1[4],v2[4];
  double p,q,r;
  double y;
  double e,f,esq,fsq,ef;
  double g,gg,h,hh;

  asq = a*a;

  p = b;
  q = a*c-4.0*d;
  r = (asq - 4.0*b)*d + c*c;
  y = cubic(p,q,r);

  esq = .25*asq - b - y;
  if (esq < 0.0) return(0);
  else {
    fsq = .25*y*y - d;
    if (fsq < 0.0) return(0);
    else {
      ef = -(.25*a*y + .5*c);
      if ( ((a > 0.0)&&(y > 0.0)&&(c > 0.0))
           || ((a > 0.0)&&(y < 0.0)&&(c < 0.0))
           || ((a < 0.0)&&(y > 0.0)&&(c < 0.0))
           || ((a < 0.0)&&(y < 0.0)&&(c > 0.0))
           ||  (a == 0.0)||(y == 0.0)||(c == 0.0)
           ) {
        /* use ef - */
        if ((b < 0.0)&&(y < 0.0)&&(esq > 0.0)) {
          e = sqrt(esq);
          f = ef/e;
        } else if ((d < 0.0) && (fsq > 0.0)) {
          f = sqrt(fsq);
          e = ef/f;
        } else {
          e = sqrt(esq);
          f = sqrt(fsq);
          if (ef < 0.0) f = -f;
        }
      } else {
        e = sqrt(esq);
        f = sqrt(fsq);
        if (ef < 0.0) f = -f;
      }
      /* note that e >= 0.0 */
      ainv2 = a*.5;
      g = ainv2 - e;
      gg = ainv2 + e;
      if ( ((b > 0.0)&&(y > 0.0))
           || ((b < 0.0)&&(y < 0.0)) ) {
        if (( a > 0.0) && (e != 0.0)) g = (b + y)/gg;
        else if (e != 0.0) gg = (b + y)/g;
      }
      if ((y == 0.0)&&(f == 0.0)) {
        h = 0.0;
        hh = 0.0;
      } else if ( ((f > 0.0)&&(y < 0.0))
                  || ((f < 0.0)&&(y > 0.0)) ) {
        hh = -.5*y + f;
        h = d/hh;
      } else {
        h = -.5*y - f;
        hh = d/h;
      }
      n1 = qudrtc(gg,hh,v1);
      n2 = qudrtc(g,h,v2);
      nquar = n1+n2;
      rts[0] = v1[0];
      rts[1] = v1[1];
      rts[n1+0] = v2[0];
      rts[n1+1] = v2[1];
      return nquar;
    }
  }
} /* ferrari */

int icp6D_QUAT::qudrtc(double b, double c, double rts[4])
/*
  solve the quadratic equation -

  x**2+b*x+c = 0

*/
{
  int nquad;
  double rtdis;
  double dis = b*b-4.0*c;

  if (dis >= 0.0) {
    nquad = 2;
    rtdis = sqrt(dis);
    if (b > 0.0) rts[0] = ( -b - rtdis)*.5;
    else         rts[0] = ( -b + rtdis)*.5;
    if (rts[0] == 0.0) rts[1] = -b;
    else               rts[1] = c/rts[0];
  } else {
    nquad = 0;
    rts[0] = 0.0;
    rts[1] = 0.0;
  }
  return nquad;
} /* qudrtc */


double icp6D_QUAT::cubic(double p, double q, double r)
/*
  find the lowest real root of the cubic -
  x**3 + p*x**2 + q*x + r = 0

  input parameters -
  p,q,r - coeffs of cubic equation.

  output-
  cubic - a real root.

  method -
  see D.E. Littlewood, "A University Algebra" pp.173 - 6

  Charles Prineas   April 1981

*/
{
  // int nrts;
  double po3,po3sq,qo3;
  double uo3,u2o3,uo3sq4,uo3cu4;
  double v,vsq,wsq;
  double m,mcube=0.0,n;
  double muo3,s,scube,t,cosk,sinsqk;
  double root;

  double doubmax = sqrt(DBL_MAX);

  m = 0.0;
  // nrts =0;
  if        ((p > doubmax) || (p <  -doubmax)) {
    root = -p;
  } else if ((q > doubmax) || (q <  -doubmax)) {
    if (q > 0.0) root = -r/q;
    else         root = -sqrt(-q);
  } else if ((r > doubmax)|| (r <  -doubmax)) {
    root =  -cbrt(r);
  } else {
    po3 = p/3.0;
    po3sq = po3*po3;
    if (po3sq > doubmax) root =  -p;
    else {
      v = r + po3*(po3sq + po3sq - q);
      if ((v > doubmax) || (v < -doubmax))
        root = -p;
      else {
        vsq = v*v;
        qo3 = q/3.0;
        uo3 = qo3 - po3sq;
        u2o3 = uo3 + uo3;
        if ((u2o3 > doubmax) || (u2o3 < -doubmax)) {
          if (p == 0.0) {
            if (q > 0.0) root =  -r/q;
            else         root =  -sqrt(-q);
          } else         root =  -q/p;
        }
        uo3sq4 = u2o3*u2o3;
        if (uo3sq4 > doubmax) {
          if (p == 0.0) {
            if (q > 0.0) root = -r/q;
            else         root = -sqrt(fabs(q));
          } else         root = -q/p;
        }
        uo3cu4 = uo3sq4*uo3;
        wsq = uo3cu4 + vsq;
        if (wsq >= 0.0) {
          //
          // cubic has one real root
          //
          //nrts = 1;
          if (v <= 0.0) mcube = ( -v + sqrt(wsq))*.5;
          if (v  > 0.0) mcube = ( -v - sqrt(wsq))*.5;
          m = cbrt(mcube);
          if (m != 0.0) n = -uo3/m;
          else          n = 0.0;
          root = m + n - po3;
        } else {
          // nrts = 3;
          //
          // cubic has three real roots
          //
          if (uo3 < 0.0) {
            muo3 = -uo3;
            s = sqrt(muo3);
            scube = s*muo3;
            t =  -v/(scube+scube);
            cosk = cos(acos(t)/3.0);
            if (po3 < 0.0)
              root = (s+s)*cosk - po3;
            else {
              sinsqk = 1.0 - cosk*cosk;
              if (sinsqk < 0.0) sinsqk = 0.0;
              root = s*( -cosk - sqrt(3*sinsqk)) - po3;
            }
          } else
            //
            // cubic has multiple root -
            //
            root = cbrt(v) - po3;
        }
      }
    }
  }
  return root;
} /* cubic */

// calculate the maximum eigenvector of a symmetric
// 4x4 matrix
// from B. Horn 1987 Closed-form solution of absolute
// orientation using unit quaternions (J.Opt.Soc.Am.A)
void icp6D_QUAT::maxEigenVector(double Q[4][4], double ev[4])
{
  double N[4][4];
  double rts[4];
  double c[4];
  // find the coeffs for the characteristic eqn.
  characteristicPol(Q, c);
  // find roots
  ferrari(c[0], c[1], c[2], c[3], rts);
  // find maximum root = maximum eigenvalue
  double l = rts[0];
  if (rts[1] > l) l = rts[1];
  if (rts[2] > l) l = rts[2];
  if (rts[3] > l) l = rts[3];

  // create the Q - l*I matrix
  N[0][0]=Q[0][0]-l;N[0][1]=Q[0][1] ;N[0][2]=Q[0][2]; N[0][3]=Q[0][3];
  N[1][0]=Q[1][0]; N[1][1]=Q[1][1]-l;N[1][2]=Q[1][2]; N[1][3]=Q[1][3];
  N[2][0]=Q[2][0]; N[2][1]=Q[2][1] ;N[2][2]=Q[2][2]-l;N[2][3]=Q[2][3];
  N[3][0]=Q[3][0]; N[3][1]=Q[3][1] ;N[3][2]=Q[3][2];N[3][3]=Q[3][3]-l;

  // the columns of the inverted matrix should be multiples of
  // the eigenvector, pick the largest
  int ipiv[4];
  double best[4], curr[4];
  if (LU_factor(N, ipiv)) {
    cerr << "maxEigenVector():" << endl;
    cerr << "LU_factor failed!" << endl;
    cerr << "return identity quaternion" << endl;
    ev[0] = 1.0;
    ev[1] = ev[2] = ev[3] = 0.0;
    return;
  }
  best[1] = best[2] = best[3] = 0; best[0] = 1;
  LU_solve(N, ipiv, best);
  double len =
    best[0]*best[0] + best[1]*best[1] +
    best[2]*best[2] + best[3]*best[3];
  for (int i=1; i<4; i++) {
    curr[0] = curr[1] = curr[2] = curr[3] = 0; curr[i] = 1;
    LU_solve(N, ipiv, curr);
    double tlen =
      curr[0]*curr[0] + curr[1]*curr[1] +
      curr[2]*curr[2] + curr[3]*curr[3];
    if (tlen > len) {
      len = tlen;
      best[0] = curr[0];
      best[1] = curr[1];
      best[2] = curr[2];
      best[3] = curr[3];
    }
  }
  // normalize the result
  len = 1.0/sqrt(len);
  ev[0] = best[0]*len;
  ev[1] = best[1]*len;
  ev[2] = best[2]*len;
  ev[3] = best[3]*len;
}

// find the coefficients of the characteristic eqn.
// l^4 + a l^3 + b l^2 + c l + d = 0
// for a symmetric 4x4 matrix
void icp6D_QUAT::characteristicPol(double Q[4][4], double c[4])
{
  // squares
  double q01_2 = Q[0][1] * Q[0][1];
  double q02_2 = Q[0][2] * Q[0][2];
  double q03_2 = Q[0][3] * Q[0][3];
  double q12_2 = Q[1][2] * Q[1][2];
  double q13_2 = Q[1][3] * Q[1][3];
  double q23_2 = Q[2][3] * Q[2][3];

  // other factors
  double q0011 = Q[0][0] * Q[1][1];
  double q0022 = Q[0][0] * Q[2][2];
  double q0033 = Q[0][0] * Q[3][3];
  double q0102 = Q[0][1] * Q[0][2];
  double q0103 = Q[0][1] * Q[0][3];
  double q0223 = Q[0][2] * Q[2][3];
  double q1122 = Q[1][1] * Q[2][2];
  double q1133 = Q[1][1] * Q[3][3];
  double q1223 = Q[1][2] * Q[2][3];
  double q2233 = Q[2][2] * Q[3][3];

  // a
  c[0] = -Q[0][0] - Q[1][1] - Q[2][2] - Q[3][3];

  // b
  c[1] = - q01_2 - q02_2 - q03_2 + q0011 - q12_2 -
    q13_2 + q0022 + q1122 - q23_2 + q0033 + q1133 +
    q2233;

  // c
  c[2] = (q02_2 + q03_2 + q23_2)*Q[1][1] - 2*q0102*Q[1][2] +
    (q12_2 + q13_2 + q23_2)*Q[0][0] +
    (q01_2 + q03_2 - q0011 + q13_2 - q1133)*Q[2][2] -
    2*Q[0][3]*q0223 - 2*(q0103 + q1223)*Q[1][3] +
    (q01_2 + q02_2 - q0011 + q12_2 - q0022)*Q[3][3];

  // d
  c[3] = 2*(-Q[0][2]*Q[0][3]*Q[1][2] + q0103*Q[2][2] -
            Q[0][1]*q0223 + Q[0][0]*q1223)*Q[1][3] +
    q02_2*q13_2 - q03_2*q1122 - q13_2*q0022 +
    2*Q[0][3]*Q[1][1]*q0223 - 2*q0103*q1223 + q01_2*q23_2 -
    q0011*q23_2 - q02_2*q1133 + q03_2*q12_2 +
    2*q0102*Q[1][2]*Q[3][3] - q12_2*q0033 - q01_2*q2233 +
    q0011*q2233;
}

double icp6D_QUAT::Align_Parallel(const int openmp_num_threads,
                                 const unsigned int n[OPENMP_NUM_THREADS],
                                 const double sum[OPENMP_NUM_THREADS],
                                 const double centroid_m[OPENMP_NUM_THREADS][3],
                                 const double centroid_d[OPENMP_NUM_THREADS][3],
                                 const double Si[OPENMP_NUM_THREADS][9],
                                 double *alignfx)
{
  double s = 0.0;
  double ret;
  unsigned int pairs_size = 0;
  double cm[3] = {0.0, 0.0, 0.0};  // centroid m
  double cd[3] = {0.0, 0.0, 0.0};  // centroid d

  // Implementation according to the paper
  // "The Parallel Iterative Closest Point Algorithm"
  // by Langis / Greenspan / Godin, IEEE 3DIM 2001
  // formula (4)
  for (int i = 0; i < openmp_num_threads; i++) {
    s += sum[i];
    pairs_size += n[i];
    cm[0] += n[i] * centroid_m[i][0];
    cm[1] += n[i] * centroid_m[i][1];
    cm[2] += n[i] * centroid_m[i][2];
    cd[0] += n[i] * centroid_d[i][0];
    cd[1] += n[i] * centroid_d[i][1];
    cd[2] += n[i] * centroid_d[i][2];
  }
  cm[0] /= pairs_size;
  cm[1] /= pairs_size;
  cm[2] /= pairs_size;
  cd[0] /= pairs_size;
  cd[1] /= pairs_size;
  cd[2] /= pairs_size;

  ret = sqrt(s / (double)pairs_size);
  if (!quiet) {
    cout.setf(ios::basefield);
    cout << "PQUAT RMS point-to-point error = "
         << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
         << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
         << std::setw(10) << std::setprecision(7)
         << ret
         << "  using " << std::setw(6) << pairs_size << " points" << endl;
  }

  double S[3][3];
  double Q[4][4];
  int i,j;

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      S[i][j] = 0;

  // calculate the cross covariance matrix
  // formula (5)
  for (int i = 0; i < openmp_num_threads; i++) {
    for(int j = 0; j < 3; j++){
      for(int k = 0; k < 3; k++){
        S[j][k] += Si[i][k*3+j]
          + n[i] * ((centroid_d[i][j] - cd[j]) * (centroid_m[i][k] - cm[k])) ;
      }
    }
  }

  S[0][0] -= cd[0] * cm[0];
  S[0][1] -= cd[0] * cm[1];
  S[0][2] -= cd[0] * cm[2];
  S[1][0] -= cd[1] * cm[0];
  S[1][1] -= cd[1] * cm[1];
  S[1][2] -= cd[1] * cm[2];
  S[2][0] -= cd[2] * cm[0];
  S[2][1] -= cd[2] * cm[1];
  S[2][2] -= cd[2] * cm[2];

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
      Q[i+1][j+1] = S[i][j]+S[j][i]-(i==j ? trace : 0);

  // the quaternion
  double q[7];

  maxEigenVector(Q, q);

  // calculate the rotation matrix
  double m[3][3]; // rot matrix
  quaternion2matrix(q, m);

  M4identity(alignfx);

  alignfx[0] = m[0][0];
  alignfx[1] = m[1][0];
  alignfx[2] = m[2][0];
  alignfx[3] = 0.0;
  alignfx[4] = m[0][1];
  alignfx[5] = m[1][1];
  alignfx[6] = m[2][1];
  alignfx[7] = 0.0;
  alignfx[8] = m[0][2];
  alignfx[9] = m[1][2];
  alignfx[10] = m[2][2];
  alignfx[11] = 0.0;

  // calculate the translation vector,
  alignfx[12] = cm[0] - m[0][0]*cd[0] - m[0][1]*cd[1] - m[0][2]*cd[2];
  alignfx[13] = cm[1] - m[1][0]*cd[0] - m[1][1]*cd[1] - m[1][2]*cd[2];
  alignfx[14] = cm[2] - m[2][0]*cd[0] - m[2][1]*cd[1] - m[2][2]*cd[2];

  return ret;
}
