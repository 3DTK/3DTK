/*
 * icp6Dsvd implementation
 *
 * Copyright (C) Kai Lingemann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/** @file 
 *  @brief Implementation of the ICP error function minimization via SVD
 *  @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Inst. of CS, University of Osnabrueck, Germany.
 */

#include "slam6d/icp6Dsvd.h"

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
 * point pairs using the SVD PARAMETERS
 * vector of point pairs, rotation matrix
 * @param pairs Vector of point pairs (pairs of corresponding points)
 * @param *alignfx The resulting transformation matrix
 * @return Error estimation of the matching (rms)
*/
double icp6D_SVD::Align(const vector<PtPair>& pairs,
                        double *alignfx,
                        const double centroid_m[3],
                        const double centroid_d[3])
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
    cout << "SVD RMS point-to-point error = "
      << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
      << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
      << std::setw(10) << std::setprecision(7)
      << error
      << "  using " << std::setw(6) << (int)pairs.size() << " points" << endl;
  }

  // Fill H matrix
  Matrix H(3,3), R(3,3);
  for(int j = 0; j < 3; j++){
    for(int k = 0; k < 3; k++){
      H(j+1, k+1) = 0.0;     
    }
  }
  
  for(unsigned int i = 0; i < pairs.size(); i++){
    for(int j = 0; j < 3; j++){
      for(int k = 0; k < 3; k++){
        H(j+1, k+1) += d[i][j]*m[i][k];
      }
    }
  }
    
  Matrix U(3,3);
  DiagonalMatrix Lamda(3);
  Matrix V(3,3);
  // Make SVD
  SVD(H, Lamda, U, V);

  // Get rotation
  R = V*(U.t());

  // Calculate translation
  double translation[3];
  ColumnVector col_vec(3);
  for(int j = 0; j < 3; j++)
    col_vec(j+1) = centroid_d[j];
  ColumnVector r_time_colVec = ColumnVector(R*col_vec);
  translation[0] = centroid_m[0] - r_time_colVec(1);
  translation[1] = centroid_m[1] - r_time_colVec(2);
  translation[2] = centroid_m[2] - r_time_colVec(3);


  // Fill result
  alignfx[0] = R(1,1);
  alignfx[1] = R(2,1);
  alignfx[2] = 0;
  alignfx[2] = R(3,1);
  alignfx[3] = 0;
  alignfx[4] = R(1,2);
  alignfx[5] = R(2,2);
  alignfx[6] = R(3,2);
  alignfx[7] = 0;
  alignfx[8] = R(1,3);
  alignfx[9] = R(2,3);
  alignfx[10] = R(3,3);
  alignfx[11] = 0;
  alignfx[12] = translation[0];
  alignfx[13] = translation[1];
  alignfx[14] = translation[2];

  alignfx[15] = 1;


  for(unsigned int i = 0; i <  pairs.size(); i++) {
    delete [] m[i];
    delete [] d[i];
  }
  delete [] m;
  delete [] d;

  return error;
}

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
double icp6D_SVD::Align_Parallel(const int openmp_num_threads, 
                                const unsigned int n[OPENMP_NUM_THREADS],
                                const double sum[OPENMP_NUM_THREADS], 
                                const double centroid_m[OPENMP_NUM_THREADS][3],
                                const double centroid_d[OPENMP_NUM_THREADS][3], 
                                const double Si[OPENMP_NUM_THREADS][9],
                                double *alignxf)
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
  //
  // The same information are given in (ecrm2007.pdf)
  // Andreas Nüchter. Parallelization of Scan Matching
  // for Robotic 3D Mapping. In Proceedings of the 3rd
  // European Conference on Mobile Robots (ECMR '07),
  // Freiburg, Germany, September 2007
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
    cout << "PSVD RMS point-to-point error = "
      << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
      << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
      << std::setw(10) << std::setprecision(7)
      << ret
      << "  using " << std::setw(6) << pairs_size << " points" << endl;
  }

  Matrix H(3,3), R(3,3);
  for(int j = 0; j < 3; j++){
    for(int k = 0; k < 3; k++){
      H(j+1, k+1) = 0.0;     
    }
  }
  // formula (5)
  for (int i = 0; i < openmp_num_threads; i++) {
    for(int j = 0; j < 3; j++){
      for(int k = 0; k < 3; k++){
        H(j+1, k+1) += Si[i][k*3+j] +
          n[i] * ((centroid_d[i][j] - cd[j]) * (centroid_m[i][k] - cm[k])) ;
      }
    }
  }

  Matrix U(3,3);
  DiagonalMatrix Lamda(3);
  Matrix V(3,3);
  // Make SVD
  SVD(H, Lamda, U, V);

  // Get rotation
  R = V*(U.t());

  // Calculate translation
  double translation[3];
  ColumnVector col_vec(3);
  for(int j = 0; j < 3; j++) {
    col_vec(j+1) = cd[j];
  }
  ColumnVector r_time_colVec = ColumnVector(R * col_vec);
  translation[0] = cm[0] - r_time_colVec(1);
  translation[1] = cm[1] - r_time_colVec(2);
  translation[2] = cm[2] - r_time_colVec(3);

  // Fill result
  alignxf[0] = R(1,1);
  alignxf[1] = R(2,1);
  alignxf[2] = 0;
  alignxf[2] = R(3,1);
  alignxf[3] = 0;
  alignxf[4] = R(1,2);
  alignxf[5] = R(2,2);
  alignxf[6] = R(3,2);
  alignxf[7] = 0;
  alignxf[8] = R(1,3);
  alignxf[9] = R(2,3);
  alignxf[10] = R(3,3);
  alignxf[11] = 0;
  alignxf[12] = translation[0];
  alignxf[13] = translation[1];
  alignxf[14] = translation[2];

  alignxf[15] = 1;

  return ret;

}
