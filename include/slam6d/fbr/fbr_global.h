/**
 * @file fbr_global.h
 * @brief Globally used headers, functions, structures
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @date 2012/05/9 14:00
 */

#ifndef FBR_GLOBAL_H_
#define FBR_GLOBAL_H_

#include <iostream>
#include <vector>
#include <fstream>
#if (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif
//for opencv 2.4
#if (CV_MAJOR_VERSION >= 2) && (CV_MINOR_VERSION >= 4)
#include <opencv2/nonfree/nonfree.hpp>
#endif
#include <math.h>
#include <string>
#include "slam6d/io_types.h"
#include "slam6d/globals.icc"

using namespace std;

namespace fbr{
  
  //Vertical angle of view of scanner
#define MAX_ANGLE 60.0
#define MIN_ANGLE -40.0
  
  /**
   * @enum projection_method
   */
  enum projection_method{
    EQUIRECTANGULAR,
    CYLINDRICAL,
    MERCATOR,
    RECTILINEAR,
    PANNINI,
    STEREOGRAPHIC,
    ZAXIS,
    CONIC
  };
  /**
   * @enum panorama_map_method
   */
  enum panorama_map_method{
    FARTHEST,
    EXTENDED,
  };
  /**
   * @enum feature_method
   */
  enum feature_detector_method{
    SIFT_DET,
    SURF_DET,
    ORB_DET,
    FAST_DET,
    STAR_DET,
  };
  enum feature_descriptor_method{
    SIFT_DES,
    SURF_DES,
    ORB_DES,
  };
  /**
   * @enum matching_method
   */
  enum matcher_method{
    BRUTEFORCE,
    FLANN,
    KNN,
    RADIUS,
    RATIO,
  };
  /**
   * @enum registration_method
   */
  enum registration_method{
    ALL,
    RANSAC,
    DISABLE,
  };
  /**
   * @enum feature_filtration_method
   */
  enum feature_filtration_method{
    OCCLUSION,
    STANDARD_DEVIATION,
    DISABLE_FILTER,
  };
  /**
   * @enum matching_filtration_method
   */
  enum matching_filtration_method{
    FUNDEMENTAL_MATRIX,
    DISABLE_MATCHING_FILTER,
  };
  //RANSAC iteration       
#define RANSACITR 20000
  //Inlier influence                                                     
#define iInfluence 0.5

  string scanFormatToString(IOType format);
  IOType stringToScanFormat(string format);
  string projectionMethodToString(projection_method method);
  projection_method stringToProjectionMethod(string method);
  string panoramaMapMethodToString(panorama_map_method method);
  panorama_map_method stringToPanoramaMapMethod(string method);
  string featureDetectorMethodToString(feature_detector_method method);
  feature_detector_method stringToFeatureDetectorMethod(string method);
  string featureDescriptorMethodToString(feature_descriptor_method method);
  feature_descriptor_method stringToFeatureDescriptorMethod(string method);
  string matcherMethodToString(matcher_method method);
  matcher_method stringToMatcherMethod(string method);
  string registrationMethodToString(registration_method method);
  registration_method stringToRegistrationMethod(string method);
  string featureFiltrationMethodToString(feature_filtration_method method);
  feature_filtration_method stringToFeatureFiltrationMethod(string method);
  string matchingFiltrationMethodToString(matching_filtration_method method);
  matching_filtration_method stringToMatchingFiltrationMethod(string method);
}
#endif /* FBR_GLOBAL_H_ */
