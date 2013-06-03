/*
 * normal definition
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

#ifndef __NORMALS_PANORAMA_H__
#define __NORMALS_PANORAMA_H__

#include <vector>
using std::vector;
#include <slam6d/scan.h>
#include <slam6d/fbr/panorama.h>
#if (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#include <opencv/highgui.h>
#else
#include <opencv2/opencv.hpp>
#endif

void calculateNormalsPANORAMA(vector<Point> &normals,
                              vector<Point> &points,
                              const vector< vector< vector< cv::Vec3f > > >
                              extendedMap,
                              const double _rPos[3]);

// see paper
// Fast and Accurate Computation of Surface Normals from Range Images
// by H. Badino, D. Huber, Y. Park and T. Kanade
void calculateNormalsFAST(vector<Point> &normals,
                          vector<Point> &points,
                          const cv::Mat &img,
                          const float max,
					 const double _rPos[3],
                          const vector< vector< vector <cv::Vec3f> > >
                          &extendedMap);

// TODO should be exported to separate library
/*
 * retrieve a cv::Mat with x,y,z,r from a scan object
 * functionality borrowed from scan_cv::convertScanToMat but this function
 * does not allow a scanserver to be used, prints to stdout and can only
 * handle a single scan
 */
static inline cv::Mat scan2mat(Scan *source)
{
  DataXYZ xyz = source->get("xyz");

  DataReflectance xyz_reflectance = source->get("reflectance");

  unsigned int nPoints = xyz.size();
  cv::Mat scan(nPoints,1,CV_32FC(4));
  scan = cv::Scalar::all(0);

  cv::MatIterator_<cv::Vec4f> it;

  it = scan.begin<cv::Vec4f>();
  for(unsigned int i = 0; i < nPoints; i++){
    float x, y, z, reflectance;
    x = xyz[i][0];
    y = xyz[i][1];
    z = xyz[i][2];
    if(xyz_reflectance.size() != 0)
      {
        reflectance = xyz_reflectance[i];

        // normalize the reflectance
        reflectance += 32;
        reflectance /= 64;
        reflectance -= 0.2;
        reflectance /= 0.3;
        if (reflectance < 0) reflectance = 0;
        if (reflectance > 1) reflectance = 1;
      }

    (*it)[0] = x;
    (*it)[1] = y;
    (*it)[2] = z;
    if(xyz_reflectance.size() != 0)
      (*it)[3] = reflectance;
    else
      (*it)[3] = 0;

    ++it;
  }
  return scan;
}

#endif // __NORMALS_PANORAMA_H__
