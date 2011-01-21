#ifndef __GEOM_MATH_H__
#define __GEOM_MATH_H__

#include "newmat/newmatio.h"
#include "newmat/newmatap.h"
//using namespace NEWMAT;

#include <vector>
using std::vector;

template <class T, class F>
T planeDist(const T *p, F nx, F ny, F nz, F d) { 
    return p[0]*nx + p[1]*ny + p[2]*nz + d;
}
bool SphereInAABB( float x, float y, float z, float size ) ;
void setNumber(double *plane, double *center, double _radius, double _maxDist) ;
bool PlaneInCube( float x, float y, float z, float size, float nx, float ny, float nz, float d);
bool PlaneInCube( float x, float y, float z, float size);
bool closeToPlane(double *p);
  
// given a set of points this will calculate the best fit plane
template <class T>
static T fitPlane(vector<T *> &ppoints, T plane[4], T centroid[3]) {
  NEWMAT::SymmetricMatrix A(3);
	A = 0;
  int n;
  n = ppoints.size();
  double cx, cy, cz;
  cx = 0.0;
  cy = 0.0;
  cz = 0.0;

  for (int i = 0; i < n; i++) {
    T* p = ppoints[i];
    cx += p[0];
    cy += p[1];
    cz += p[2];
  }
  cx /= n;
  cy /= n;
  cz /= n;
        
  centroid[0] = cx;
  centroid[1] = cy;
  centroid[2] = cz;

  for (int i = 0; i < n; i++) {
    T* p = ppoints[i];
    A(1, 1) += (p[0] - cx)*(p[0] - cx);
    A(2, 2) += (p[1] - cy)*(p[1] - cy);
    A(3, 3) += (p[2] - cz)*(p[2] - cz);
    A(1, 2) += (p[0] - cx)*(p[1] - cy);
    A(1, 3) += (p[0] - cx)*(p[2] - cz);
    A(2, 3) += (p[1] - cy)*(p[2] - cz);
  }

  NEWMAT::DiagonalMatrix D;
  NEWMAT::Matrix V;
  try {
    NEWMAT::Jacobi(A,D,V);
  } catch (NEWMAT::ConvergenceException) {
    cout << "couldn't find plane..." << endl;
    return 0;
  }
  /*
     cout << "A: "<< endl << A << endl;
     cout << "D: "<< endl << D << endl;
     cout << "V: "<< endl << V << endl;
     */
  int index;
  D.MinimumAbsoluteValue1(index);
     
  plane[0] = V(1,index);
  plane[1] = V(2,index);
  plane[2] = V(3,index);
  plane[3] = -planeDist(plane, cx, cy, cz, 0.0);
  //plane[3] = -(plane[0]*cx + plane[1]*cy + plane[2]*cz);
  
  double sum = 0.0;
  for(int i = 1; i < 4; i++) {
    sum += D(i);
  }
  sum = D(index)/sum;

  return D(index)/n;
}

#endif
