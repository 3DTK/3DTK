/*
 * icp6Dortho implementation
 *
 * Copyright (C) Andreas Nuechter, Alexandru-Eugen Ichim
 *
 * Released under the GPL version 3.
 *
 */

/** @file
 *  @brief Implementation of the ICP error function minimization via
 *  orthonormal matrices
 *
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 *  @author Alexandru-Eugen Ichim. Jacobs University Bremen gGmbH, Germany
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
using namespace NEWMAT;

/**
 * computes the rotation matrix consisting
 * of a rotation and translation that
 * minimizes the root-mean-square error of the
 * point pairs using orthonormal matrices
 * vector of point pairs, rotation matrix
 * @param pairs Vector of point pairs (pairs of corresponding points)
 * @param *alignfx The resulting transformation matrix
 * @return Error estimation of the matching (rms)
 */
double icp6D_ORTHO::Align(const vector<PtPair>& pairs,
                          double *alignfx,
                          const double centroid_m[3],
                          const double centroid_d[3])
{
  double error = 0;
  double sum = 0.0;

  /// Get centered PtPairs
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
         << "  using " << std::setw(6) << (int)pairs.size() << " points"
         << endl;
  }

  /////////////////////////////////////////////////////////
  /////////////////////////////////////////////////////////

  /// generate matrix H
  Matrix H (3, 3); H = 0.0;
  for(unsigned int n = 0; n < pairs.size(); ++n)
    for(int i = 0; i < 3; ++i)
      for(int j = 0; j < 3; ++j)
        H(i+1, j+1) += m[n][i] * d[n][j];
  Matrix HH = H.t() * H;

  /// create a new matrix HHs equal to HH, but is of type SymmetricMatrix
  //  needed for calculating the eigenvalues
  SymmetricMatrix HHs(3);
  HHs(1, 2) = HH(2, 1);
  HHs(1, 3) = HH(3, 1);
  HHs(2, 3) = HH(3, 2);
  HHs(1, 1) = HH(1, 1);
  HHs(2, 2) = HH(2, 2);
  HHs(3, 3) = HH(3, 3);

  /// get the eigenvalues of HH
  DiagonalMatrix eigenvalues(3);
  Matrix eigenvectorsMatrix(3, 3);
  EigenValues(HHs, eigenvalues, eigenvectorsMatrix);

  /// extract the eigenvectors from the matrix to separate column vectors
  ColumnVector ev1 = eigenvectorsMatrix.Column(1);
  ColumnVector ev2 = eigenvectorsMatrix.Column(2);
  ColumnVector ev3 = eigenvectorsMatrix.Column(3);


  /// compute the rotation matrix as H * ( SUM ( 1/lambda_i * ev_i * ev_i^T ))
  Matrix R(3, 3);
  Matrix ev11 = ev1 * ev1.t() * 1/sqrt( eigenvalues(1));
  Matrix ev22 = ev2 * ev2.t() * 1/sqrt( eigenvalues(2));
  Matrix ev33 = ev3 * ev3.t() * 1/sqrt( eigenvalues(3));
  R = H * (ev11 + ev22 + ev33);

  /// calculate the translation based on the rotation matrix and the centroids
  ColumnVector cm(3); cm << centroid_m;
  ColumnVector cd(3); cd << centroid_d;
  ColumnVector translation = cm - R * cd;

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

  /// clean up memory used by centered point pairs
  for(unsigned int i = 0; i < pairs.size(); i++){
    delete [] m[i];
    delete [] d[i];
  }
  delete [] m;
  delete [] d;

  return error;
}


