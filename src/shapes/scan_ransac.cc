/*
 * scan_ransac implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#define WANT_STREAM ///< define the WANT stream :)
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <errno.h>
#include <iostream>

#include "slam6d/scan_io.h"
#include "slam6d/globals.icc"
#include <float.h>

#include "shapes/geom_math.h"
#include "shapes/integralimg.h"
#include "shapes/scan_ransac.h"
#include "shapes/NumberRecOctree.h"

#ifdef _OPENMP
#include <omp.h>
#endif


#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#include <windows.h>
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <strings.h>
#include <dlfcn.h>
#endif


void *dummy(void *nd) {
  NumberDetector *ND  = (NumberDetector *)nd;
  ND->RANSAC(*(ND->scan_points));
  return 0;
}


///////////////// constants for RANSAC and number recognition
void NumberDetector::printNumbers() {
  for (int i = 0; i < 10; i++)
    cout << detectedNumbers[i].getConsensus() << endl; 
  cout << endl;
}

// comment the following line if you want greyscale image
#define BINARY_IMG
#include "shapes/config_sickday.h"
#include "shapes/numberrec.h"

int image_counter = 0;


bool NumberDetector::FindNumber(vector<double *> &points, double plane[4]) {
  bool res = false;
  double maxx = -DBL_MAX;
  double minx = DBL_MAX;
  double maxz = -DBL_MAX;
  double minz = DBL_MAX;

  // rotate onto the floor plane
  double t[3]; t[0] = t[1] = t[2] = 0.0;
  double alignxf[16];
  double aa[4];
  aa[0] = -1.0 * acos(plane[1]);
  aa[1] = plane[2] / sqrt( plane[2]*plane[2] + plane[0]*plane[0] );
  aa[2] = 0;
  aa[3] = -plane[0] / sqrt( plane[2]*plane[2] + plane[0]*plane[0] );

  AAToMatrix(aa, t, alignxf);

  /////// acount for rotation around y axis
  double up[3];
  double rPT[3];
  double rmat[16];
  double tmp[16];
  Cross(plane, aa+1, up);
  Normalize3(up);

  // check wether up goes up or down, and flip if it goes down
  if (up[1] < 0) {
    up[0] = -up[0];
    up[1] = -up[1];
    up[2] = -up[2];
  }

  double upx = up[0] * alignxf[0] + up[1] * alignxf[4] + up[2] * alignxf[8];
//  double upy = up[0] * alignxf[1] + up[1] * alignxf[5] + up[2] * alignxf[9]; // should be redundant
  double upz = up[0] * alignxf[2] + up[1] * alignxf[6] + up[2] * alignxf[10];

  rPT[0] = rPT[2] = 0.0;
  rPT[1] = atan2(upz, upx) + M_PI/2.0;
  EulerToMatrix4(t, rPT, rmat);

  // combine rotation onto plane with rotaion around y
  MMult(rmat, alignxf, tmp);
  for (int i = 0; i < 16; i++) {
    alignxf[i] = tmp[i];
  }
  // done

  // compute 2d projection of the points, and scale reflectivity
  double **npoints = new double*[points.size()];
  for (unsigned int i = 0; i < points.size(); i++) {
    double *p = points[i];
    npoints[i] = new double[4];
    
    npoints[i][0] = -(p[0] * alignxf[0] + p[1] * alignxf[4] + p[2] * alignxf[8]);
    npoints[i][1] = p[0] * alignxf[1] + p[1] * alignxf[5] + p[2] * alignxf[9]; // this should be redundant
    npoints[i][2] = p[0] * alignxf[2] + p[1] * alignxf[6] + p[2] * alignxf[10];
    
    npoints[i][3] = (p[3] - MINREFL ) / (MAXREFL - MINREFL) ;      // scale between 0 and 1
    if (npoints[i][3] > 1.0) npoints[i][3] = 1.0;                  // 0 and 1 _are_ min and max
    if (npoints[i][3] < 0.0) npoints[i][3] = 0.0;

    if (npoints[i][0] >  maxx) maxx = npoints[i][0];
    if (npoints[i][0] <  minx) minx = npoints[i][0];
    if (npoints[i][2] >  maxz) maxz = npoints[i][2];
    if (npoints[i][2] <  minz) minz = npoints[i][2];
  }


  // create a binary image which is used to find a rectangular shape using integral images
  double lx = maxx - minx; 
  double lz = maxz - minz; 
  int maxppm = 0;
  if (lx > 50.0 && lz > 70.0) {  // only if point cloud has the necessary extent  TODO change magic numbers
    int px = lx / IMG_RES +1;
    int pz = lz / IMG_RES +1;

    int **image = new int*[px];      // allocate mem
    for (int i = 0; i < px; i++) {
      image[i] = new int[pz];
    }

    for (int i = 0; i < px; i++) {    
      for (int j = 0; j < pz; j++) {
        image[i][j] = 0;
      }
    }
    for (unsigned int i = 0; i < points.size(); i++) {
      double *p = npoints[i];
      int x = (double)(px-1) * ( p[0]-minx ) / lx;
      int z = (double)(pz-1) * ( p[2]-minz ) / lz;
      image[x][z] = 1;
    }

    // compute integral image and compute best haar feature to detect a board
    int XX, ZZ;
    double SCORE;
    integral_img iimg(image, px, pz);
    SCORE = iimg.getBest( XX, ZZ);

    // with some certainty there is a rectangular part in the image that is number-sized
    if (SCORE > MIN_SCORE) {
    #ifndef BINARY_IMG
      maxppm = 4096;     // greyscale
    #else 
      maxppm = 1;        // binary
    #endif
      // create number part of the image with reflectance values...
      int maxx = XX + BOARD_SIZE_X < px ? XX + BOARD_SIZE_X : px;
      int maxz = ZZ + BOARD_SIZE_Z < pz ? ZZ + BOARD_SIZE_Z : pz;
      for (int i = XX; i < maxx ; i++) {
        for (int j = ZZ; j < maxz; j++) {
            image[i][j] = -1;          // remember pixel where no points were
        }
      }
      
      int NRPTS = 0;
      double center[3]; center[0] = center[1] = center[2] = 0.0;

      for (unsigned int i = 0; i < points.size(); i++) {
        double *p = npoints[i];
        double x = (double)(px-1) * ( p[0]-minx ) / lx;
        double z = (double)(pz-1) * ( p[2]-minz ) / lz;
        double xi, zi;
        x = modf(x, &xi);
        z = modf(z, &zi);

        if (x < 0.5){
          xi--;
          x += 1.0;
        }
        if (z < 0.5) {
          zi--;
          z += 1.0;
        }

        // points not on the board need not to be used
        if (xi < XX || zi < ZZ || xi > maxx || zi > maxz) continue;
        
        // count number of points for normalizing 
        NRPTS++; 
        center[0] += points[i][0];
        center[1] += points[i][1];
        center[2] += points[i][2];

        // Use smooth binning with 4096 gray values
        if ( (int)xi < px && (int)zi < pz  ) {
          image[(int)xi][(int)zi] += ( (1.5 - x) * (1.5 - z) * p[3]) * 4096; 
          if (image[(int)xi][(int)zi] > maxppm) maxppm = image[(int)xi][(int)zi];
        }

        if ( (int)xi + 1 < px && (int)zi < pz ) {
          image[(int)xi + 1][(int)zi] += ((x - 0.5) * (1.5 - z) * p[3]) * 4096; 
          if (image[(int)xi + 1][(int)zi] > maxppm) maxppm = image[(int)xi+1][(int)zi];
        }

        if ( (int)xi < px && (int)zi + 1 < pz  ) {
          image[(int)xi][(int)zi + 1] += ((1.5 - x) * (z - 0.5) * p[3]) * 4096; 
          if (image[(int)xi][(int)zi + 1] > maxppm) maxppm = image[(int)xi][(int)zi+1];
        }

        if ( (int)xi + 1 < px && (int)zi + 1 < pz  ) {
          image[(int)xi + 1][(int)zi + 1] += ((x - 0.5) * (z - 0.5) * p[3]) * 4096; 
          if (image[(int)xi + 1][(int)zi + 1] > maxppm) maxppm = image[(int)xi+1][(int)zi+1];
        }
      }
      ///////////////////////////////////
#ifndef BINARY_IMG
      int MAX = 0;
      int MIN = 1000000000;
      int AVG = 0;
#endif
      // color pixels on the border of the image white ( to remove the black border on the board and the inevitable part where no points are)
      for (int i = XX; i < maxx ; i++) {
        for (int j = ZZ; j < maxz; j++) {
//          if ( (j < ZZ + WHITE_BORDER - 1 || j > maxz - WHITE_BORDER - 2 ) || ( i < XX + WHITE_BORDER + 1 || i > maxx - WHITE_BORDER  ) ) { 
          if ( (j < ZZ + WHITE_BORDER  || j > maxz - WHITE_BORDER ) || ( i < XX + WHITE_BORDER  || i > maxx - WHITE_BORDER  ) ) { 
            image[i][j] = maxppm;
          }
#ifndef BINARY_IMG
          if (image[i][j] > 0 && image[i][j] < MIN) MIN = image[i][j];
          if (image[i][j] > MAX) MAX = image[i][j];
          if (image[i][j] > 0) AVG += image[i][j];
#endif
        }
      }

      string filename = "patch" + to_string(image_counter++, 5) + ".pgm";
      ofstream of(filename.c_str(), std::ios::out);

#ifndef BINARY_IMG
      of << "P2" << endl << (maxx - XX) << " " << (maxz - ZZ) << endl << 4096 << endl;    // greyscale
#else
      unsigned char ibuffer[(maxx - XX)*(maxz - ZZ)];   // binary
      of << "P5" << endl << (maxx - XX) << " " << (maxz - ZZ) << endl << 1 << endl;       // binary
#endif

      unsigned int l = 0;
      double val;             // temporary value that contains the calibrated reflectivity of a pixel
      double factor = 4096.0/maxppm;
      for (int j = ZZ; j < maxz; j++) {
        for (int i = XX; i < maxx; i++) {
#ifndef BINARY_IMG
          //        of << image[i][j] << " ";
          if (image[i][j] == -1) {
            of << 0 << " ";
          } else {
            of << (int)(factor*image[i][j]) << " ";
          }
#else 
          if (image[i][j] == -1) {
            ibuffer[l++] = (unsigned char)0;
          } else {
            //val = ((double)image[i][j]-MIN)/((double)(MAX-MIN));
            val = ((double)image[i][j])/((double)(NRPTS));
            if (val > REFL_THRESHOLD) {
            //if (image[i][j] > 0.25*maxppm) 
            //if (image[i][j] > 4000) 
              ibuffer[l++] = (unsigned char)1;
            } else {
              ibuffer[l++] = (unsigned char)0;
            }
          }
#endif
        }
      }

#ifdef BINARY_IMG
      of.write((const char*)ibuffer, sizeof(unsigned char) * (maxx - XX) * (maxz - ZZ));
#endif

      // done writing board
      of.close();

      double probability;
      int number;
      // attempt to use OCR
      if ( gocr_recognizeNumber(filename, number, probability) ) {
        cout << filename << ": Best number is " << number << " with p = " << probability << endl;

        center[0] = center[0] / NRPTS;
        center[1] = center[1] / NRPTS;
        center[2] = center[2] / NRPTS;

        // remember hypothesis, scale SCORE to a more useful value
        SCORE = (SCORE - SCORE_SCALE) / (1.0 - SCORE_SCALE);

        // decide where to put the number
        if (number == 1 && detectedNumbers[7].isClose( center) ) {  // a 7 can be seen as a 1
          detectedNumbers[7].addObservation(7, probability, SCORE, center, plane);
        } else if (number == 0 && detectedNumbers[8].isClose( center) ) {   // an 8 can be mistaken for a 0
          detectedNumbers[8].addObservation(8, probability, SCORE, center, plane);
        } else if (number == 9 && detectedNumbers[8].isClose( center) ) {   // an 8 can be mistaken for a 9
          detectedNumbers[8].addObservation(8, probability, SCORE, center, plane);
        } else {      // usual case
          detectedNumbers[number].addObservation(number, probability, SCORE, center, plane);
        }
        
        res = true; 
      }
    }


    // remove image
    for (int i = 0; i < px; i++) {
      delete[] image[i];
    }
    delete[] image;
  }

  // remove projected points
  for (unsigned int i = 0; i < points.size(); i++) {
    delete[] npoints[i];
  }
  delete[] npoints;
  return res;
}


void NumberDetector::RANSAC(vector<const double *> &scan_points) {
  // stores 3 sample points    
  vector<double *> ps;
  double a[3], b[3], c[3], plane[4];
  // create octree from the points
  NumberRecOctTree<double> *oct = new NumberRecOctTree<double>(scan_points, 50.0, PointType<double>::USE_REFLECTANCE );
 
  scan_points.clear();
  while(true) {
    ps.clear();
    oct->DrawPoints(ps, 3);

    if ( !ps.empty()) {                                // if we have random points
      for (int j = 0; j < 3;j++) {                     // compute plane
        a[j] = ps[0][j] - ps[1][j];
        b[j] = ps[0][j] - ps[2][j];
        c[j] = ps[0][j] + ps[1][j] + ps[2][j];
        c[j] /= 3.0;
      }
      Cross(a,b, plane);
      if (fabs(Len2(plane)) < 0.0001 ) continue;       // points are collinear
      Normalize3(plane);
      if (fabs(plane[1]) > 0.8) continue;              // plane is floor or ceiling
      
      plane[3] = -1.0 * planeDist(c, plane[0], plane[1], plane[2], 0);    // compute distance from origin
      if (plane[3] < 0.0) {                            // flip normal if necessary
        for (int j = 0; j < 4;j++) {
          plane[j] = -plane[j];
        }
      }
      // count number of points on the plane
      int r =  oct->PointsOnNumber(plane, MAX_DIST_TO_PLANE, c, 105.0);
      
      if (r > MIN_NR_PTS) {
        vector<double * > points;
        oct->PointsOnNumber(plane, MAX_DIST_TO_PLANE, c, 105.0, points);
        if ( FindNumber(points, plane) ) {
          printNumbers();
        }
      }

    }
  }

  delete oct;
}
