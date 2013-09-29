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
 * @author Vladimir Komsiyski. Jacobs University Bremen gGmbH, Germany
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#include <vector>
#include "slam6d/io_types.h"
#include "slam6d/globals.icc"
#include "slam6d/scan.h"
#include "newmat/newmat.h"
#include "newmat/newmatap.h"

#include "slam6d/normals.h"

using namespace NEWMAT;
using namespace std;


///////////////////////////////////////////////////////
/////////////NORMALS USING IMAGE NEIGHBORS ////////////
///////////////////////////////////////////////////////
void calculateNormalsPANORAMA(vector<Point> &normals,
                              vector<Point> &points,
                              const vector< vector< vector< cv::Vec3f > > >
                                extendedMap,
                              const double _rPos[3])
{
  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];

  points.clear();
  int nr_neighbors = 0;

  // as the nearest neighbors and then the same PCA method as done in AKNN
  // temporary dynamic array for all the neighbors of a given point
  vector<cv::Vec3f> neighbors;
  for (size_t i = 0; i < extendedMap.size(); i++) {
    for (size_t j=0; j<extendedMap[i].size(); j++) {
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

      // if no or too few neighbors point is found in the 4-neighboring pixels
      // then normal is set to zero
      if (nr_neighbors < 3)
      {
        points.push_back(Point(p[0], p[1], p[2]));
        normals.push_back(Point(0.0,0.0,0.0));
        continue;
      }

      // calculate mean for all the points
      Matrix X(nr_neighbors,3);
      SymmetricMatrix A(3);
      Matrix U(3,3);
      DiagonalMatrix D(3);

      // calculate mean for all the points
      for(int k = 0; k < nr_neighbors; k++) {
        cv::Vec3f pp = neighbors[k];
        mean.x += pp[0];
        mean.y += pp[1];
        mean.z += pp[2];
      }

      mean.x /= nr_neighbors;
      mean.y /= nr_neighbors;
      mean.z /= nr_neighbors;
      // calculate covariance = A for all the points
      for (int n = 0; n < nr_neighbors; ++n) {
        cv::Vec3f pp = neighbors[n];
        X(n+1, 1) = pp[0] - mean.x;
        X(n+1, 2) = pp[1] - mean.y;
        X(n+1, 3) = pp[2] - mean.z;
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

      for (unsigned int k = 0; k < extendedMap[i][j].size(); k++) {
        cv::Vec3f p = extendedMap[i][j][k];
        points.push_back(Point(p[0], p[1], p[2]));
        normals.push_back(Point(n(1), n(2), n(3)));
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////
//////FAST NORMALS USING PANORAMA EQUIRECTANGULAR RANGE IMAGE ///////////
/////////////////////////////////////////////////////////////////////////
void calculateNormalsFAST(vector<Point> &normals,
                          vector<Point> &points,
                          const cv::Mat &img,
                          const float max,
                          const double _rPos[3],
                          const vector< vector< vector <cv::Vec3f> > >
                          &extendedMap)
{

  ColumnVector rPos(3);
  for (int i = 0; i < 3; ++i)
    rPos(i+1) = _rPos[i];
  
  //!!!!!!!!!!

  int height = extendedMap.size();
  int width  = extendedMap[0].size();
  
  ofstream human_pgm("image.range1", ios::out);

  human_pgm << "P2\n" << width << " " << height
            << "\n" << (int)(max*100) << "\n";
  
  for (int i = 0; i < height; i++) {
    for (int j = width-1; j >= 0; j--) {
      human_pgm << (float)(img.at<float>(i,j)*100) << " ";
    }
    human_pgm << endl;
  }
  human_pgm.close();

  //!!!!!!!!!!!!!
  
  points.clear();
  int nr_points = 0;
  for (size_t i = 0; i < extendedMap.size(); i++) {
    for (size_t j = 0; j < extendedMap[0].size(); j++) {
      double theta, phi, rho;
      double dRdTheta, dRdPhi;
      double n[3]; //, m;
      nr_points = extendedMap[i][j].size();
      if (nr_points == 0 ) continue;
      
      for (int k = 0; k < nr_points; k++) {
        cv::Vec3f p = extendedMap[i][j][k];

        swap(p[1],p[2]);
        p[1]*=-1;
        
        double p_cart[3];//, p_polar[3];
        p_cart[0] = (double)(p[0]);
        p_cart[1] = (double)(p[1]);
        p_cart[2] = (double)(p[2]);

        static const double deg2rad = 3.1415 / 180.0;
        rho = sqrt(sqr(p_cart[0]) + sqr(p_cart[1]) + sqr(p_cart[2]));
        phi = acos(p_cart[2]/rho) - 30.0 * deg2rad;
        theta = atan2(p_cart[1], p_cart[0]) + 180.0 * deg2rad;
                
        /*
          toPolar(p_cart, p_polar);

     theta = p_polar[0] * 180 / M_PI;
     phi = p_polar[1] * 180 / M_PI;
     rho = p_polar[2];
     //horizantal angle of view of [0:360] and vertical of [-40:60]
     phi = 360.0 - phi;
     //@
     phi += 90; if (phi > 360) phi -= 360;
     //@ 
     phi = phi * 2.0 * M_PI / 360.0;
     theta -= 90;
     theta *= -1;
     theta *= 2.0 * M_PI / 360.0;

     swap(phi,theta);
     //        phi -= rad(30.0);

     */
     
        // Sobel Filter for the derivative
        dRdTheta = dRdPhi = 0.0;
        
        if (i == 0 || i == extendedMap.size()-1 ||
            j == 0 || j == extendedMap[0].size()-1) {
          points.push_back(Point(p_cart[0], p_cart[1], p_cart[2]));
          normals.push_back(Point(0.0, 0.0, 0.0));
          continue;
        }

        int x = i; // extendedMap.size() - i;
        int y = j; // extendedMap[0].size() - j;

        dRdPhi += 100 * 10*img.at<float>(x-1,y);
        dRdPhi += 100 * 3 *img.at<float>(x-1,y-1);
        dRdPhi += 100 * 3 *img.at<float>(x-1,y+1);
        dRdPhi -= 100 * 10*img.at<float>(x+1,y);
        dRdPhi -= 100 * 3 *img.at<float>(x+1,y-1);
        dRdPhi -= 100 * 3 *img.at<float>(x+1,y+1);
        
        dRdTheta += 100 * 10*img.at<float>(x,y-1);
        dRdTheta += 100 * 3 *img.at<float>(x-1,y-1);
        dRdTheta += 100 * 3 *img.at<float>(x+1,y-1);
        dRdTheta -= 100 * 10*img.at<float>(x,y+1);
        dRdTheta -= 100 * 3 *img.at<float>(x-1,y+1);
        dRdTheta -= 100 * 3 *img.at<float>(x+1,y+1);

        // add more weight to derivatives
        dRdTheta *= 20;
        dRdPhi *= 20;

	   phi += rad(30.0);
        
        n[0] = cos(theta) * sin(phi) - sin(theta) * dRdTheta / rho / sin(phi) +
          cos(theta) * cos(phi) * dRdPhi / rho;
        
        n[1] = sin(theta) * sin(phi) + cos(theta) * dRdTheta / rho / sin(phi) +
          sin(theta) * cos(phi) * dRdPhi / rho;
        
        n[2] =  cos(phi) - sin(phi) * dRdPhi / rho;

        n[2] = -n[2];
        
        p_cart[1]*=-1;
        swap(p_cart[1],p_cart[2]);

        ColumnVector point_vector(3);
        ColumnVector norm_vector(3);
        norm_vector(1) = n[0];
        norm_vector(2) = n[2];
        norm_vector(3) = -1.0*n[1];
        point_vector(1) = p_cart[0] - rPos(1);
        point_vector(2) = p_cart[1] - rPos(2);
        point_vector(3) = p_cart[2] - rPos(3);
        point_vector = point_vector / point_vector.NormFrobenius();
        Real angle = (norm_vector.t() * point_vector).AsScalar();
        if (angle < 0) {
          norm_vector *= -1.0;
        }
        norm_vector = norm_vector / norm_vector.NormFrobenius();
        
        points.push_back(Point(p_cart[0], p_cart[1], p_cart[2]));
	   normals.push_back(Point(norm_vector(1),norm_vector(2),norm_vector(3)));
      }
    }
  }
}
