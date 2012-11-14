#ifndef NORMALS_H
#define NORMALS_H

#include <vector>
#include <slam6d/scan.h>
#if (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#include <opencv/highgui.h>
#else
#include <opencv2/opencv.hpp>
#endif

void calculateNormalsApxKNN(std::vector<Point> &normals,
					   vector<Point> &points,
					   int k,
					   const double _rPos[3],
					   double eps = 0.0);

void calculateNormalsAdaptiveApxKNN(std::vector<Point> &normals,
							 vector<Point> &points,
							 int kmin,
							 int kmax,
							 const double _rPos[3],
							 double eps = 0.0);

void calculateNormalsKNN(std::vector<Point> &normals,
					vector<Point> &points,
					int k,
					const double _rPos[3] );


void calculateNormalsAdaptiveKNN(vector<Point> &normals,
						   vector<Point> &points,
						   int kmin, int kmax,
						   const double _rPos[3]);

void calculateNormalsPANORAMA(vector<Point> &normals,
                              vector<Point> &points,
                              vector< vector< vector< cv::Vec3f > > > extendedMap,
                              const double _rPos[3]);

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

      //normalize the reflectance
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
// TODO should be exported to separate library
/*
 * convert a matrix of float values (range image) to a matrix of unsigned
 * eight bit characters using different techniques
 */
static inline cv::Mat float2uchar(cv::Mat &source, bool logarithm, float cutoff)
{
  cv::Mat result(source.size(), CV_8U, cv::Scalar::all(0));
  float max = 0;
  // find maximum value
  if (cutoff == 0.0) {
    // without cutoff, just iterate through all values to find the largest
    for (cv::MatIterator_<float> it = source.begin<float>();
         it != source.end<float>(); ++it) {
      float val = *it;
      if (val > max) {
        max = val;
      }
    }
  } else {
    // when a cutoff is specified, sort all the points by value and then
    // specify the max so that <cutoff> values are larger than it
    vector<float> sorted(source.cols*source.rows);
    int i = 0;
    for (cv::MatIterator_<float> it = source.begin<float>();
         it != source.end<float>(); ++it, ++i) {
      sorted[i] = *it;
    }
    std::sort(sorted.begin(), sorted.end());
    max = sorted[(int)(source.cols*source.rows*(1.0-cutoff))];
    cout << "A cutoff of " << cutoff << " resulted in a max value of " << max << endl;
  }

  cv::MatIterator_<float> src = source.begin<float>();
  cv::MatIterator_<uchar> dst = result.begin<uchar>();
  cv::MatIterator_<float> end = source.end<float>();
  if (logarithm) {
    // stretch values from 0 to max logarithmically over 0 to 255
    // using the logarithm allows to represent smaller values with more
    // precision and larger values with less
    max = log(max+1);
    for (; src != end; ++src, ++dst) {
      float val = (log(*src+1)*255.0)/max;
      if (val > 255)
        *dst = 255;
      else
        *dst = (uchar)val;
    }
  } else {
    // stretch values from 0 to max linearly over 0 to 255
    for (; src != end; ++src, ++dst) {
      float val = (*src*255.0)/max;
      if (val > 255)
        *dst = 255;
      else
        *dst = (uchar)val;
    }
  }
  return result;
}



#endif // NORMALS_H
