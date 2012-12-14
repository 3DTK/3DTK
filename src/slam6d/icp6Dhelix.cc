/*
 * icp6Dhelix implementation
 *
 * Copyright (C) Peter Schneider, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */


/** @file 
 *  @brief Implementation of the ICP error function minimization via
 *         helix-translation
 *  @author Peter Schneider. Inst. CS, Univ. of Koblenz and Landau, Germany.
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#include "slam6d/icp6Dhelix.h"

#include "slam6d/globals.icc"
#include <iomanip>
using std::ios;
using std::resetiosflags;
using std::setiosflags;

#include "newmat/newmat.h"
#include "newmat/newmatap.h"
using namespace NEWMAT;
#include <cstring>

/**
 * computes a rotation matrix that rotates the points around an axis
 * and a translation vector, that translates the points along a vector
 * that is parallel to the rotation axis. Thus the result is a helical 
 * translation of the points that can be resolved thru a vector field
 * v(x) = cs + c cross x, where cs and c can be build out of an error 
 * minimization function.
 *
 * See:
 * H. Pottmann, S. Leopoldseder, and M. Hofer.  Simultaneous
 * registration of multiple views of a 3D object.
 * ISPRS Archives 34/3A (2002), 265-270.
 *
 * @param Pairs Vector of point pairs (pairs of corresponding points)
 * @param alignxf The resulting transformation matrix
 * @return Error estimation of the matching (rms)
 */
double icp6D_HELIX::Align(const vector<PtPair>& Pairs,
                          double *alignxf,
                          const double centroid_m[3],
                          const double centroid_d[3])
{
  int n = Pairs.size();

  int i;
  double sum = 0;
  double error;
  Matrix matB(6,6);
  ColumnVector bdVec(6), ccs(6), c(3), cs(3);
  matB = 0.0;
  double p2x, p2y, p2z, pDistX, pDistY, pDistZ;

  double B[6][3];
  memset(&B[0][0], 0, 18 * sizeof(double));
  double bd[6];
  memset(&bd[0], 0, 6 * sizeof(double));

  for (i = 0; i < n; i++) {
    p2x = Pairs[i].p2.x;
    p2y = Pairs[i].p2.y;
    p2z = Pairs[i].p2.z;

    B[4][0] += -p2z;
    B[3][1] += p2z;
    B[5][0] += p2y;
    B[3][2] += -p2y;
    B[4][2] += p2x;
    B[5][1] += -p2x;
    B[0][0] += p2z*p2z + p2y*p2y;
    B[1][0] += p2y*-p2x;
    B[2][0] += -p2z*p2x;
    B[1][1] += p2z*p2z + p2x*p2x;
    B[2][1] += p2z*-p2y;
    B[2][2] += p2x*p2x + p2y*p2y;


    pDistX = p2x - Pairs[i].p1.x;
    pDistY = p2y - Pairs[i].p1.y;
    pDistZ = p2z - Pairs[i].p1.z;

    bd[0] += -p2z*pDistY + p2y*pDistZ;
    bd[1] += p2z*pDistX - p2x*pDistZ;
    bd[2] += -p2y*pDistX + p2x*pDistY;
    bd[3] += pDistX;
    bd[4] += pDistY;
    bd[5] += pDistZ;

    sum += pDistX*pDistX + pDistY*pDistY + pDistZ*pDistZ;
  }

  matB(4,4) = matB(5,5) = matB(6,6) = n;

  matB(1,5) = matB(5,1) = B[4][0];
  matB(2,4) = matB(4,2) = B[3][1];
  matB(1,6) = matB(6,1) = B[5][0];
  matB(3,4) = matB(4,3) = B[3][2]; 
  matB(3,5) = matB(5,3) = B[4][2];
  matB(2,6) = matB(6,2) = B[5][1];
  matB(1,2) = matB(2,1) = B[1][0];
  matB(1,3) = matB(3,1) = B[2][0];
  matB(2,3) = matB(3,2) = B[2][1];
  matB(1,1) = B[0][0];
  matB(2,2) = B[1][1];
  matB(3,3) = B[2][2];

  bdVec(1) = bd[0];
  bdVec(2) = bd[1];
  bdVec(3) = bd[2];
  bdVec(4) = bd[3];
  bdVec(5) = bd[4];
  bdVec(6) = bd[5];

  error = sqrt( sum / (double) n );

  if (!quiet) {
    cout.setf(ios::basefield);
    cout << "HELIX RMS point-to-point error = "
         << resetiosflags(ios::adjustfield) << setiosflags(ios::internal)
         << resetiosflags(ios::floatfield) << setiosflags(ios::fixed)
         << std::setw(10) << std::setprecision(7)
         << error
         << "  using " << std::setw(6) << n << " points" << endl;
  }

  ccs = matB.i() * bdVec;

  int vectorOffset = 0;
  computeRt( &ccs, vectorOffset, alignxf);    

  return error;
}


void icp6D_HELIX::computeRt(const ColumnVector* ccs,
                            const int vectorOffset,
                            double *alignxf)
{
  ColumnVector c(3), cs(3);
  c(1) = -(*ccs)(vectorOffset + 1);
  c(2) = -(*ccs)(vectorOffset + 2);
  c(3) = -(*ccs)(vectorOffset + 3);
  cs(1) = -(*ccs)(vectorOffset + 4);
  cs(2) = -(*ccs)(vectorOffset + 5);       
  cs(3) = -(*ccs)(vectorOffset + 6); 

  double CLength = sqrt(c.SumSquare());
  double rotationCheck = c(1)*cs(1) + c(2)*cs(2) + c(3)*cs(3);  //c.t() * cs;
  Matrix R (3,3);
  double angle = atan(CLength);               // bemerkung: hier minus gesetzt
   
  ColumnVector g = c / CLength;
  double b0, b1, b2, b3;
  double sinAngle = sin(-angle/2);
  b0 = cos(-angle/2);
  b1 = g(1) * sinAngle;
  b2 = g(2) * sinAngle;
  b3 = g(3) * sinAngle;

  R(1,1) = b0*b0 + b1*b1 - b2*b2 - b3*b3;
  R(1,2) = 2*(b1*b2 + b0*b3);
  R(1,3) = 2*(b1*b3 - b0*b2);
  R(2,1) = 2*(b1*b2 - b0*b3);                
  R(2,2) = b0*b0 - b1*b1 + b2*b2 - b3*b3;
  R(2,3) = 2*(b2*b3 + b0*b1);
  R(3,1) = 2*(b1*b3 + b0*b2);
  R(3,2) = 2*(b2*b3 - b0*b1);
  R(3,3) = b0*b0 - b1*b1 - b2*b2 + b3*b3;
  R = R / (b0*b0 + b1*b1 + b2*b2 + b3*b3);

  double skewValue = rotationCheck / (CLength*CLength);
  ColumnVector gs = (cs - (c * skewValue)) / CLength;    
  ColumnVector pTemp(3);
  pTemp(1) = g(2)*gs(3) - g(3)*gs(2);
  pTemp(2) = g(3)*gs(1) - g(1)*gs(3);
  pTemp(3) = g(1)*gs(2) - g(2)*gs(1);
  ColumnVector t = R * -pTemp + g*(skewValue * angle) + pTemp;  

  alignxf[0]  = R(1,1);
  alignxf[1]  = R(2,1);
  alignxf[2]  = R(3,1);
  alignxf[3]  = 0;
  alignxf[4]  = R(1,2);
  alignxf[5]  = R(2,2);
  alignxf[6]  = R(3,2);
  alignxf[7]  = 0;
  alignxf[8]  = R(1,3);
  alignxf[9]  = R(2,3);
  alignxf[10] = R(3,3);
  alignxf[11] = 0;
  alignxf[12] = t(1);
  alignxf[13] = t(2);
  alignxf[14] = t(3);
  alignxf[15] = 1;
}


