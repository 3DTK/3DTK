/*
 * normals implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief efficient normal computation
 *
 * @author Vaibhav Kumar Mehta. Jacobs University Bremen gGmbH, Germany
 * @author Corneliu Claudiu Prodescu. Jacobs University Bremen gGmbH, Germany
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#include <vector>
#include <ANN/ANN.h>
#include "slam6d/io_types.h"
#include "slam6d/globals.icc"
#include "slam6d/kd.h"
#include "newmat/newmat.h"
#include "newmat/newmatap.h"

#include "slam6d/normals.h"

using namespace NEWMAT;
using namespace std;

///////////////////////////////////////////////////////
/////////////NORMALS USING AKNN METHOD ////////////////
///////////////////////////////////////////////////////
void calculateNormalsApxKNN(vector<Point> &normals,
                            vector<Point> &points,
                            int k,
                            const double _rPos[3],
                            double eps)
{
  int nr_neighbors = k;

  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];

  ANNpointArray pa = annAllocPts(points.size(), 3);
  for (size_t i = 0; i < points.size(); ++i) {
    pa[i][0] = points[i].x;
    pa[i][1] = points[i].y;
    pa[i][2] = points[i].z;

  }
  ANNkd_tree t(pa, points.size(), 3);
  ANNidxArray nidx = new ANNidx[nr_neighbors];
  ANNdistArray d = new ANNdist[nr_neighbors];

  for (size_t i = 0; i < points.size(); ++i) {
    ANNpoint p = pa[i];
    // ANN search for k nearest neighbors
    // indexes of the neighbors along with the query point
    // stored in the array n
    t.annkSearch(p, nr_neighbors, nidx, d, eps);
    Point mean(0.0,0.0,0.0);
    Matrix X(nr_neighbors,3);
    SymmetricMatrix A(3);
    Matrix U(3,3);
    DiagonalMatrix D(3);
    // calculate mean for all the points
    for (int j = 0; j < nr_neighbors; ++j) {
      mean.x += points[nidx[j]].x;
      mean.y += points[nidx[j]].y;
      mean.z += points[nidx[j]].z;
    }
    mean.x /= nr_neighbors;
    mean.y /= nr_neighbors;
    mean.z /= nr_neighbors;
    // calculate covariance = A for all the points
    for (int i = 0; i < nr_neighbors; ++i) {
      X(i+1, 1) = points[nidx[i]].x - mean.x;
      X(i+1, 2) = points[nidx[i]].y - mean.y;
      X(i+1, 3) = points[nidx[i]].z - mean.z;
    }

    A << 1.0/nr_neighbors * X.t() * X;

    EigenValues(A, D, U);

    // normal = eigenvector corresponding to lowest
    // eigen value that is the 1st column of matrix U
    ColumnVector n(3);
    n(1) = U(1,1);
    n(2) = U(2,1);
    n(3) = U(3,1);
    ColumnVector point_vector(3);
    point_vector(1) = p[0] - rPos(1);
    point_vector(2) = p[1] - rPos(2);
    point_vector(3) = p[2] - rPos(3);
    point_vector = point_vector / point_vector.NormFrobenius();
    Real angle = (n.t() * point_vector).AsScalar();
    if (angle < 0) {
      n *= -1.0;
    }
    n = n / n.NormFrobenius();
    normals.push_back(Point(n(1), n(2), n(3)));
  }

  delete[] nidx;
  delete[] d;
  annDeallocPts(pa);
}

////////////////////////////////////////////////////////////////
/////////////NORMALS USING ADAPTIVE AKNN METHOD ////////////////
////////////////////////////////////////////////////////////////
void calculateNormalsAdaptiveApxKNN(vector<Point> &normals,
                                    vector<Point> &points,
                                    int kmin,
                                    int kmax,
                                    const double _rPos[3],
                                    double eps)
{
  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];

  int nr_neighbors;
  ANNpointArray pa = annAllocPts(points.size(), 3);
  for (size_t i = 0; i < points.size(); ++i) {
    pa[i][0] = points[i].x;
    pa[i][1] = points[i].y;
    pa[i][2] = points[i].z;
  }
  ANNkd_tree t(pa, points.size(), 3);

  Point mean(0.0,0.0,0.0);
  double e1,e2,e3;

  for (size_t i = 0; i < points.size(); ++i) {
    Matrix U(3,3);
    ANNpoint p = pa[i];
    for(int kidx = kmin; kidx < kmax; kidx++) {
      nr_neighbors=kidx+1;
      ANNidxArray nidx = new ANNidx[nr_neighbors];
      ANNdistArray d = new ANNdist[nr_neighbors];
      // ANN search for k nearest neighbors
      // indexes of the neighbors along with the query point
      // stored in the array n
      t.annkSearch(p, nr_neighbors, nidx, d, eps);
      mean.x=0,mean.y=0,mean.z=0;
      // calculate mean for all the points
      for (int j=0; j<nr_neighbors; ++j) {
        mean.x += points[nidx[j]].x;
        mean.y += points[nidx[j]].y;
        mean.z += points[nidx[j]].z;
      }
      mean.x /= nr_neighbors;
      mean.y /= nr_neighbors;
      mean.z /= nr_neighbors;

      Matrix X(nr_neighbors,3);
      SymmetricMatrix A(3);
      DiagonalMatrix D(3);

      // calculate covariance = A for all the points
      for (int j = 0; j < nr_neighbors; ++j) {
        X(j+1, 1) = points[nidx[j]].x - mean.x;
        X(j+1, 2) = points[nidx[j]].y - mean.y;
        X(j+1, 3) = points[nidx[j]].z - mean.z;
      }

      A << 1.0/nr_neighbors * X.t() * X;

      EigenValues(A, D, U);

      e1 = D(1);
      e2 = D(2);
      e3 = D(3);

      delete[] nidx;
      delete[] d;

      // We take the particular k if the second maximum eigen value
      // is at least 25 percent of the maximum eigen value
      if ((e1 > 0.25 * e2) && (fabs(1.0 - (double)e2/(double)e3) < 0.25))
        break;
    }

    // normal = eigenvector corresponding to lowest
    // eigen value that is the 1rd column of matrix U
    ColumnVector n(3);
    n(1) = U(1,1);
    n(2) = U(2,1);
    n(3) = U(3,1);
    ColumnVector point_vector(3);
    point_vector(1) = p[0] - rPos(1);
    point_vector(2) = p[1] - rPos(2);
    point_vector(3) = p[2] - rPos(3);
    point_vector = point_vector / point_vector.NormFrobenius();
    Real angle = (n.t() * point_vector).AsScalar();
    if (angle < 0) {
      n *= -1.0;
    }
    n = n / n.NormFrobenius();
    normals.push_back(Point(n(1), n(2), n(3)));
  }
  
  annDeallocPts(pa);
}



///////////////////////////////////////////////////////
/////////////NORMALS USING AKNN METHOD ////////////////
///////////////////////////////////////////////////////
void calculateNormalsKNN(vector<Point> &normals,
                         vector<Point> &points,
                         int k,
                         const double _rPos[3] )
{
  int nr_neighbors = k;

  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];
  
  double** pa = new double*[points.size()];
  for (size_t i = 0; i < points.size(); ++i) {
    pa[i] = new double[3];
    pa[i][0] = points[i].x;
    pa[i][1] = points[i].y;
    pa[i][2] = points[i].z;
  }

  KDtree t(pa, points.size());

#ifdef _OPENMP
  omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel 
  {
    int thread_num = omp_get_thread_num();
#else
  {
    int thread_num = 0;
#endif
	 
    for (size_t i = 0; i < points.size(); ++i) {
	 
	 double p[3] = { pa[i][0], pa[i][1], pa[i][2] };
	 
	 vector<Point> temp = t.kNearestNeighbors(p,
									  nr_neighbors,
									  thread_num);

	 nr_neighbors = temp.size();
    
	 Point mean(0.0,0.0,0.0);
	 Matrix X(nr_neighbors,3);
	 SymmetricMatrix A(3);
	 Matrix U(3,3);
	 DiagonalMatrix D(3);

	 // calculate mean for all the points              
	 for (int j = 0; j < nr_neighbors; ++j) {
	   mean.x += temp[j].x;
	   mean.y += temp[j].y;
	   mean.z += temp[j].z;
	 }
	 mean.x /= nr_neighbors;
	 mean.y /= nr_neighbors;
	 mean.z /= nr_neighbors;

	 // calculate covariance = A for all the points
	 for (int i = 0; i < nr_neighbors; ++i) {
	   X(i+1, 1) = temp[i].x - mean.x;
	   X(i+1, 2) = temp[i].y - mean.y;
	   X(i+1, 3) = temp[i].z - mean.z;
	 }

	 A << 1.0/nr_neighbors * X.t() * X;
    
	 EigenValues(A, D, U);

	 // normal = eigenvector corresponding to lowest 
	 // eigen value that is the 1st column of matrix U
	 ColumnVector n(3);
	 n(1) = U(1,1);
	 n(2) = U(2,1);
	 n(3) = U(3,1);
    
	 ColumnVector point_vector(3);
	 point_vector(1) = p[0] - rPos(1);
	 point_vector(2) = p[1] - rPos(2);
	 point_vector(3) = p[2] - rPos(3);
	 point_vector = point_vector / point_vector.NormFrobenius();
	 Real angle = (n.t() * point_vector).AsScalar();
	 if (angle < 0) {
	   n *= -1.0;
	 }
	 n = n / n.NormFrobenius();
#pragma omp critical
	 normals.push_back(Point(n(1), n(2), n(3)));       
    }
  }
    
  for (size_t i = 0; i < points.size(); ++i) {
    delete[] pa[i];
  }
  delete[] pa;
}


////////////////////////////////////////////////////////////////
/////////////NORMALS USING ADAPTIVE AKNN METHOD ////////////////
////////////////////////////////////////////////////////////////
void calculateNormalsAdaptiveKNN(vector<Point> &normals,
                                 vector<Point> &points,
                                 int kmin,
                                 int kmax,
                                 const double _rPos[3])
{
  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];
  
  double** pa = new double*[points.size()];
  for (size_t i = 0; i < points.size(); ++i) {
    pa[i] = new double[3];
    pa[i][0] = points[i].x;
    pa[i][1] = points[i].y;
    pa[i][2] = points[i].z;
  }

  KDtree t(pa, points.size());

#ifdef _OPENMP
  omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel
  {
    int thread_num = omp_get_thread_num();
#else
  {
    int thread_num = 0;
#endif

    for (size_t i = 0; i < points.size(); i++) {

	 double p[3] = { pa[i][0], pa[i][1], pa[i][2] };
	 Matrix U(3,3);
	 Point mean(0.0,0.0,0.0);
	 int nr_neighbors;
	 
	 for(int kidx = kmin; kidx < kmax; kidx++) {
	   nr_neighbors = kidx + 1;
	   vector<Point> temp = t.kNearestNeighbors(p,
									    nr_neighbors,
									    thread_num);
	   
	   nr_neighbors = temp.size();
	   	   
	   mean.x = mean.y = mean.z = 0.0;
	   // calculate mean for all the points              
	   for (int j = 0; j < nr_neighbors; j++) {
		mean.x += temp[j].x;
		mean.y += temp[j].y;
		mean.z += temp[j].z;
	   }
	   mean.x /= nr_neighbors;
	   mean.y /= nr_neighbors;
	   mean.z /= nr_neighbors;
	   	   
	   double e1, e2, e3;     
	   Matrix X(nr_neighbors,3);
	   SymmetricMatrix A(3);
	   DiagonalMatrix D(3);
	   
	   // calculate covariance = A for all the points
	   for (int j = 0; j < nr_neighbors; ++j) {
		X(j+1, 1) = temp[j].x - mean.x;
		X(j+1, 2) = temp[j].y - mean.y;
		X(j+1, 3) = temp[j].z - mean.z;
	   }
	   
	   A << 1.0/nr_neighbors * X.t() * X;
	   EigenValues(A, D, U);
	   
	   e1 = D(1);
	   e2 = D(2);
	   e3 = D(3);
          
	   // We take the particular k if the second maximum eigen value 
	   // is at least 25 percent of the maximum eigen value
	   if ((e1 > 0.25 * e2) && (fabs(1.0 - (double)e2/(double)e3) < 0.25)) 
		break;
	 }
      
	 // normal = eigenvector corresponding to lowest 
	 // eigen value that is the 1rd column of matrix U
	 ColumnVector n(3);
	 n(1) = U(1,1);
	 n(2) = U(2,1);
	 n(3) = U(3,1);
	 ColumnVector point_vector(3);
	 point_vector(1) = p[0] - rPos(1);
	 point_vector(2) = p[1] - rPos(2);
	 point_vector(3) = p[2] - rPos(3);
	 point_vector = point_vector / point_vector.NormFrobenius();
	 Real angle = (n.t() * point_vector).AsScalar();
	 if (angle < 0) {
	   n *= -1.0;
	 }
	 n = n / n.NormFrobenius();
#pragma omp critical	 
	 normals.push_back(Point(n(1), n(2), n(3)));  
    }
  }
  
  for (size_t i = 0; i < points.size(); ++i) {
    delete[] pa[i];
  }
  delete[] pa;

}
