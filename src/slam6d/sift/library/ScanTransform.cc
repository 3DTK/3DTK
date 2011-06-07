/**
 * @file ScanTransform.cc
 * @brief Implementation of ScanTransform class
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/sift/library/ScanTransform.h"

using namespace std;

ScanTransform ScanTransform::inverse()
{
  double fA0 = transform[0][0]*transform[1][1] - transform[0][1]*transform[1][0];
  double fA1 = transform[0][0]*transform[1][2] - transform[0][2]*transform[1][0];
  double fA2 = transform[0][0]*transform[1][3] - transform[0][3]*transform[1][0];
  double fA3 = transform[0][1]*transform[1][2] - transform[0][2]*transform[1][1];
  double fA4 = transform[0][1]*transform[1][3] - transform[0][3]*transform[1][1];
  double fA5 = transform[0][2]*transform[1][3] - transform[0][3]*transform[1][2];
  double fB0 = transform[2][0]*transform[3][1] - transform[2][1]*transform[3][0];
  double fB1 = transform[2][0]*transform[3][2] - transform[2][2]*transform[3][0];
  double fB2 = transform[2][0]*transform[3][3] - transform[2][3]*transform[3][0];
  double fB3 = transform[2][1]*transform[3][2] - transform[2][2]*transform[3][1];
  double fB4 = transform[2][1]*transform[3][3] - transform[2][3]*transform[3][1];
  double fB5 = transform[2][2]*transform[3][3] - transform[2][3]*transform[3][2];
  
  double fDet = fA0*fB5-fA1*fB4+fA2*fB3+fA3*fB2-fA4*fB1+fA5*fB0;
  //    if (Math<double>::FAbs(fDet) <= Math<double>::ZERO_TOLERANCE)
  //    {
  //        return Matrix4<double>::ZERO;
  //    }
  
  ScanTransform kInv(scanid);
  kInv.transform[0][0] =
    + transform[1][1]*fB5 - transform[1][2]*fB4 + transform[1][3]*fB3;
  kInv.transform[1][0] =
    - transform[1][0]*fB5 + transform[1][2]*fB2 - transform[1][3]*fB1;
  kInv.transform[2][0] =
    + transform[1][0]*fB4 - transform[1][1]*fB2 + transform[1][3]*fB0;
  kInv.transform[3][0] =
    - transform[1][0]*fB3 + transform[1][1]*fB1 - transform[1][2]*fB0;
  kInv.transform[0][1] =
    - transform[0][1]*fB5 + transform[0][2]*fB4 - transform[0][3]*fB3;
  kInv.transform[1][1] =
    + transform[0][0]*fB5 - transform[0][2]*fB2 + transform[0][3]*fB1;
  kInv.transform[2][1] =
    - transform[0][0]*fB4 + transform[0][1]*fB2 - transform[0][3]*fB0;
  kInv.transform[3][1] =
    + transform[0][0]*fB3 - transform[0][1]*fB1 + transform[0][2]*fB0;
  kInv.transform[0][2] =
    + transform[3][1]*fA5 - transform[3][2]*fA4 + transform[3][3]*fA3;
  kInv.transform[1][2] =
    - transform[3][0]*fA5 + transform[3][2]*fA2 - transform[3][3]*fA1;
  kInv.transform[2][2] =
    + transform[3][0]*fA4 - transform[3][1]*fA2 + transform[3][3]*fA0;
  kInv.transform[3][2] =
    - transform[3][0]*fA3 + transform[3][1]*fA1 - transform[3][2]*fA0;
  kInv.transform[0][3] =
    - transform[2][1]*fA5 + transform[2][2]*fA4 - transform[2][3]*fA3;
  kInv.transform[1][3] =
    + transform[2][0]*fA5 - transform[2][2]*fA2 + transform[2][3]*fA1;
  kInv.transform[2][3] =
    - transform[2][0]*fA4 + transform[2][1]*fA2 - transform[2][3]*fA0;
  kInv.transform[3][3] =
    + transform[2][0]*fA3 - transform[2][1]*fA1 + transform[2][2]*fA0;
  
  double fInvDet = ((double)1.0)/fDet;
  kInv.transform[0][0] *= fInvDet;
  kInv.transform[0][1] *= fInvDet;
  kInv.transform[0][2] *= fInvDet;
  kInv.transform[0][3] *= fInvDet;
  kInv.transform[1][0] *= fInvDet;
  kInv.transform[1][1] *= fInvDet;
  kInv.transform[1][2] *= fInvDet;
  kInv.transform[1][3] *= fInvDet;
  kInv.transform[2][0] *= fInvDet;
  kInv.transform[2][1] *= fInvDet;
  kInv.transform[2][2] *= fInvDet;
  kInv.transform[2][3] *= fInvDet;
  kInv.transform[3][0] *= fInvDet;
  kInv.transform[3][1] *= fInvDet;
  kInv.transform[3][2] *= fInvDet;
  kInv.transform[3][3] *= fInvDet;
  
  return kInv;
}

ScanTransform ScanTransform::multiply(ScanTransform m) 
{	
  ScanTransform newst(scanid);
  
  newst.transform[0][0] = transform[0][0]*m.transform[0][0] + transform[0][1]*m.transform[1][0] + transform[0][2]*m.transform[2][0] + transform[0][3]*m.transform[3][0];
  newst.transform[0][1] = transform[0][0]*m.transform[0][1] + transform[0][1]*m.transform[1][1] + transform[0][2]*m.transform[2][1] + transform[0][3]*m.transform[3][1];
  newst.transform[0][2] = transform[0][0]*m.transform[0][2] + transform[0][1]*m.transform[1][2] + transform[0][2]*m.transform[2][2] + transform[0][3]*m.transform[3][2];
  newst.transform[0][3] = transform[0][0]*m.transform[0][3] + transform[0][1]*m.transform[1][3] + transform[0][2]*m.transform[2][3] + transform[0][3]*m.transform[3][3];
  
  newst.transform[1][0] = transform[1][0]*m.transform[0][0] + transform[1][1]*m.transform[1][0] + transform[1][2]*m.transform[2][0] + transform[1][3]*m.transform[3][0];
  newst.transform[1][1] = transform[1][0]*m.transform[0][1] + transform[1][1]*m.transform[1][1] + transform[1][2]*m.transform[2][1] + transform[1][3]*m.transform[3][1];
  newst.transform[1][2] = transform[1][0]*m.transform[0][2] + transform[1][1]*m.transform[1][2] + transform[1][2]*m.transform[2][2] + transform[1][3]*m.transform[3][2];
  newst.transform[1][3] = transform[1][0]*m.transform[0][3] + transform[1][1]*m.transform[1][3] + transform[1][2]*m.transform[2][3] + transform[1][3]*m.transform[3][3];
  
  newst.transform[2][0] = transform[2][0]*m.transform[0][0] + transform[2][1]*m.transform[1][0] + transform[2][2]*m.transform[2][0] + transform[2][3]*m.transform[3][0];
  newst.transform[2][1] = transform[2][0]*m.transform[0][1] + transform[2][1]*m.transform[1][1] + transform[2][2]*m.transform[2][1] + transform[2][3]*m.transform[3][1];
  newst.transform[2][2] = transform[2][0]*m.transform[0][2] + transform[2][1]*m.transform[1][2] + transform[2][2]*m.transform[2][2] + transform[2][3]*m.transform[3][2];
  newst.transform[2][3] = transform[2][0]*m.transform[0][3] + transform[2][1]*m.transform[1][3] + transform[2][2]*m.transform[2][3] + transform[2][3]*m.transform[3][3];
  
  newst.transform[3][0] = transform[3][0]*m.transform[0][0] + transform[3][1]*m.transform[1][0] + transform[3][2]*m.transform[2][0] + transform[3][3]*m.transform[3][0];
  newst.transform[3][1] = transform[3][0]*m.transform[0][1] + transform[3][1]*m.transform[1][1] + transform[3][2]*m.transform[2][1] + transform[3][3]*m.transform[3][1];
  newst.transform[3][2] = transform[3][0]*m.transform[0][2] + transform[3][1]*m.transform[1][2] + transform[3][2]*m.transform[2][2] + transform[3][3]*m.transform[3][2];
  newst.transform[3][3] = transform[3][0]*m.transform[0][3] + transform[3][1]*m.transform[1][3] + transform[3][2]*m.transform[2][3] + transform[3][3]*m.transform[3][3];
  
  return newst;
}
