/**
 *
 * Copyright (C) Jacobs University Bremen
 *
 * @author Vaibhav Kumar Mehta
 * @author Corneliu Claudiu Prodescu
 * @file normals.cc
 */

#include <vector>


#include <ANN/ANN.h>
#include <slam6d/io_types.h>
#include <slam6d/globals.icc>
#include <slam6d/scan.h>
#include "slam6d/fbr/panorama.h"
#include <scanserver/clientInterface.h>
#include "newmat/newmat.h"
#include "newmat/newmatap.h"

#include "normals/normals.h"

using namespace NEWMAT;
using namespace std;

///////////////////////////////////////////////////////
/////////////NORMALS USING AKNN METHOD ////////////////
///////////////////////////////////////////////////////
void calculateNormalsAKNN(vector<Point> &normals,vector<Point> &points, int k, const double _rPos[3] )
{
  cout<<"Total number of points: "<<points.size()<<endl;
  int nr_neighbors = k;

  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];

  ANNpointArray pa = annAllocPts(points.size(), 3);
  for (size_t i=0; i<points.size(); ++i)
  {
    pa[i][0] = points[i].x;
    pa[i][1] = points[i].y;
    pa[i][2] = points[i].z;
  }
  ANNkd_tree t(pa, points.size(), 3);
  ANNidxArray nidx = new ANNidx[nr_neighbors];
  ANNdistArray d = new ANNdist[nr_neighbors];

  for (size_t i=0; i<points.size(); ++i)
  {
    ANNpoint p = pa[i];
    //ANN search for k nearest neighbors
    //indexes of the neighbors along with the query point
    //stored in the array n
    t.annkSearch(p, nr_neighbors, nidx, d, 0.0);
    Point mean(0.0,0.0,0.0);
    Matrix X(nr_neighbors,3);
    SymmetricMatrix A(3);
    Matrix U(3,3);
    DiagonalMatrix D(3);
    //calculate mean for all the points
    for (int j=0; j<nr_neighbors; ++j)
    {
      mean.x += points[nidx[j]].x;
      mean.y += points[nidx[j]].y;
      mean.z += points[nidx[j]].z;
    }
    mean.x /= nr_neighbors;
    mean.y /= nr_neighbors;
    mean.z /= nr_neighbors;
    //calculate covariance = A for all the points
    for (int i = 0; i < nr_neighbors; ++i) {
      X(i+1, 1) = points[nidx[i]].x - mean.x;
      X(i+1, 2) = points[nidx[i]].y - mean.y;
      X(i+1, 3) = points[nidx[i]].z - mean.z;
    }

    A << 1.0/nr_neighbors * X.t() * X;

    EigenValues(A, D, U);

    //normal = eigenvector corresponding to lowest
    //eigen value that is the 1st column of matrix U
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
void calculateNormalsAdaptiveAKNN(vector<Point> &normals,vector<Point> &points,
                                  int kmin, int kmax, const double _rPos[3])
{
  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];

  cout<<"Total number of points: "<<points.size()<<endl;
  int nr_neighbors;
  ANNpointArray pa = annAllocPts(points.size(), 3);
  for (size_t i=0; i<points.size(); ++i)
  {
    pa[i][0] = points[i].x;
    pa[i][1] = points[i].y;
    pa[i][2] = points[i].z;
  }
  ANNkd_tree t(pa, points.size(), 3);

  Point mean(0.0,0.0,0.0);
  double e1,e2,e3;

  for (size_t i=0; i<points.size(); ++i)
  {
    Matrix U(3,3);
    ANNpoint p = pa[i];
    for(int kidx = kmin; kidx < kmax; kidx++)
    {
      nr_neighbors=kidx+1;
      ANNidxArray nidx = new ANNidx[nr_neighbors];
      ANNdistArray d = new ANNdist[nr_neighbors];
      //ANN search for k nearest neighbors
      //indexes of the neighbors along with the query point
      //stored in the array n
      t.annkSearch(p, nr_neighbors, nidx, d, 0.0);
      mean.x=0,mean.y=0,mean.z=0;
      //calculate mean for all the points
      for (int j=0; j<nr_neighbors; ++j)
      {
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

      //calculate covariance = A for all the points
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

      //We take the particular k if the second maximum eigen value
      //is at least 25 percent of the maximum eigen value
      if ((e1 > 0.25 * e2) && (fabs(1.0 - (double)e2/(double)e3) < 0.25))
        break;
    }

    //normal = eigenvector corresponding to lowest
    //eigen value that is the 1rd column of matrix U
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
/////////////NORMALS USING IMAGE NEIGHBORS ////////////
///////////////////////////////////////////////////////
void calculateNormalsPANORAMA(vector<Point> &normals,
                              vector<Point> &points,
                              vector< vector< vector< cv::Vec3f > > > extendedMap,
                              const double _rPos[3])
{
  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];

  cout<<"Total number of points: "<<points.size()<<endl;
  points.clear();
  int nr_neighbors = 0;
  cout << "height of Image: "<<extendedMap.size()<<endl;
  cout << "width of Image: "<<extendedMap[0].size()<<endl;

  // as the nearest neighbors and then the same PCA method as done in AKNN
  //temporary dynamic array for all the neighbors of a given point
  vector<cv::Vec3f> neighbors;
  for (size_t i=0; i< extendedMap.size(); i++)
  {
    for (size_t j=0; j<extendedMap[i].size(); j++)
    {
      if (extendedMap[i][j].size() == 0) continue;
      neighbors.clear();
      Point mean(0.0,0.0,0.0);

      // Offset for neighbor computation
      int offset[2][5] = {{-1,0,1,0,0},{0,-1,0,1,0}};

      // Traversing all the cells in the extended map
      for (int n = 0; n < 5; ++n) {
        int x = i + offset[0][n];
        int y = j + offset[1][n];

        // Copy the neighboring buckets into the vector
        if (x >= 0 && x < (int)extendedMap.size() &&
            y >= 0 && y < (int)extendedMap[x].size()) {
          for (unsigned int k = 0; k < extendedMap[x][y].size(); k++) {
            neighbors.push_back(extendedMap[x][y][k]);
          }
        }
      }

      nr_neighbors = neighbors.size();
      cv::Vec3f p = extendedMap[i][j][0];

      //if no neighbor point is found in the 4-neighboring pixels then normal is set to zero
      if (nr_neighbors < 3)
      {
        points.push_back(Point(p[0], p[1], p[2]));
        normals.push_back(Point(0.0,0.0,0.0));
        continue;
      }

      //calculate mean for all the points
      Matrix X(nr_neighbors,3);
      SymmetricMatrix A(3);
      Matrix U(3,3);
      DiagonalMatrix D(3);

      //calculate mean for all the points
      for(int k = 0; k < nr_neighbors; k++)
      {
        cv::Vec3f pp = neighbors[k];
        mean.x += pp[0];
        mean.y += pp[1];
        mean.z += pp[2];
      }

      mean.x /= nr_neighbors;
      mean.y /= nr_neighbors;
      mean.z /= nr_neighbors;
      //calculate covariance = A for all the points
      for (int n = 0; n < nr_neighbors; ++n) {
        cv::Vec3f pp = neighbors[n];
        X(n+1, 1) = pp[0] - mean.x;
        X(n+1, 2) = pp[1] - mean.y;
        X(n+1, 3) = pp[2] - mean.z;
      }

      A << 1.0/nr_neighbors * X.t() * X;

      EigenValues(A, D, U);
      //normal = eigenvector corresponding to lowest
      //eigen value that is the 1st column of matrix U
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

      for (unsigned int k = 0; k < extendedMap[i][j].size(); k++) {
        cv::Vec3f p = extendedMap[i][j][k];
        points.push_back(Point(p[0], p[1], p[2]));
        normals.push_back(Point(n(1), n(2), n(3)));
      }
    }
  }
}
//////////////////////////////////////////////////////////////////////////////////////////////
///////////FAST NORMALS USING PANORAMA EQUIRECTANGULAR RANGE IMAGE //////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////
/*
  void calculateNormalsFAST(vector<Point> &normals,vector<Point> &points,cv::Mat &img,vector<vector<vector<cv::Vec3f>>> extendedMap)
  {
  cout<<"Total number of points: "<<points.size()<<endl;
  points.clear();
  int nr_points = 0;
  //int nr_neighbors = 0,nr_neighbors_center = 0;
  cout << "height of Image: "<<extendedMap.size()<<endl;
  cout << "width of Image: "<<extendedMap[0].size()<<endl;
  for (size_t i=0; i< extendedMap.size(); ++i)
  {
  for (size_t j=0; j<extendedMap[0].size(); j++)
  {
  double theta,phi,rho;
  double x,y,z;
  double dRdTheta,dRdPhi;
  double n[3],m;
  nr_points = extendedMap[i][j].size();
  if (nr_points == 0 ) continue;

  for (int k = 0; k< nr_points; k++)
  {
  cv::Vec3f p = extendedMap[i][j][k];
  x = p[0];
  y = p[1];
  z = p[2];
  rho = sqrt(x*x + y*y + z*z);
  theta = atan(y/x);
  phi = atan(z/x);

  //Sobel Filter for the derivative
  dRdTheta = dRdPhi = 0.0;

  if (i == 0 || i == extendedMap.size()-1 || j == 0 || j == extendedMap[0].size()-1)
  {
  points.push_back(Point(x, y, z));
  normals.push_back(Point(0.0,0.0,0.0));
  continue;
  }
  dRdPhi += 10*img.at<uchar>(i-1,j);
  dRdPhi += 3 *img.at<uchar>(i-1,j-1);
  dRdPhi += 3 *img.at<uchar>(i-1,j+1);
  dRdPhi -= 10*img.at<uchar>(i+1,j);
  dRdPhi -= 3 *img.at<uchar>(i+1,j-1);
  dRdPhi -= 3 *img.at<uchar>(i+1,j+1);

  dRdTheta += 10*img.at<uchar>(i,j-1);
  dRdTheta += 3 *img.at<uchar>(i-1,j-1);
  dRdTheta += 3 *img.at<uchar>(i+1,j-1);
  dRdTheta -= 10*img.at<uchar>(i,j+1);
  dRdTheta -= 3 *img.at<uchar>(i-1,j+1);
  dRdTheta -= 3 *img.at<uchar>(i+1,j+1);

  n[0] = cos(theta) * sin(phi) - sin(theta) * dRdTheta / rho / sin(phi) +
  cos(theta) * cos(phi) * dRdPhi / rho;

  n[1] = sin(theta) * sin(phi) + cos(theta) * dRdTheta / rho / sin(phi) +
  sin(theta) * cos(phi) * dRdPhi / rho;

  n[2] =  cos(phi) - sin(phi) * dRdPhi / rho;

  //n[2] = -n[2];

  m = sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2]);
  n[0] /= m; n[1] /= m; n[2] /= m;

  points.push_back(Point(x, y, z));
  normals.push_back(Point(n[0],n[1],n[2]));
  }
  }
  }
  }
*/
