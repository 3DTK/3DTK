/**
 * @file fbr_global.h
 * @brief Globally used headers, functions, structures
 * @author Hamidreza Houshiar. Jacobs University Bremen gGmbH, Germany.
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
#if (CV_MAJOR_VERSION >= 2) && (CV_MINOR_VERSION >= 4) && WITH_OPENCV_NONFREE
#include <opencv2/nonfree/nonfree.hpp>
#endif
#include <math.h>
#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif
#include "slam6d/io_types.h"
#include "slam6d/globals.icc"
#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif
using namespace std;

namespace fbr{
  
  //Vertical angle of view of scanner
  //#define MAX_ANGLE 60.0
  //#define MIN_ANGLE -40.0
  //#define MAX_ANGLE 90.0
  //#define MIN_ANGLE -65.0

  typedef tuple<double, double, double, unsigned int> Color_Value_Tuple;
  
  /**
   * @enum scanner_type
   */
  enum scanner_type{
    NONE,
    RIEGL,
    FARO,
    MANUAL
  };
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
    CONIC,
    EQUALAREACYLINDRICAL,
    AZIMUTHAL
  };
  /**
   * @enum panorama_map_method
   */
  enum panorama_map_method{
    FARTHEST,
    EXTENDED,
    FULL,
    NON,
  };
  /**
   * @enum panorama_format
   */
  enum panorama_format{
    PNG,
    JPEG,
    JPEG2000,
    TIFF,
    WebP,
  };
  /**
   * @enum panorama_type
   */
  enum panorama_type{
    ThreeChannel24BitRange,
    ThreeGrayscaleRange,
  };
  /**
   * @enum feature_image
   */
  enum feature_detector_image_method{
    REFLECTANCE,
    COLOR,
  };
  /**
   * @enum feature_method
   */
  enum feature_detector_method{
#ifdef WITH_OPENCV_NONFREE
    SIFT_DET,
    SURF_DET,
#endif
    ORB_DET,
    FAST_DET,
    STAR_DET,
  };
  enum feature_descriptor_method{
#ifdef WITH_OPENCV_NONFREE
    SIFT_DES,
    SURF_DES,
#endif
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
  /**
   * @enum recovered_range_filteration_method
   */
  enum recovered_range_filteration_method{
    INTERQUARTILE,
    INTERQUARTILE_AVERAGEDIFF,
    DISABLE_RECOVERED_RANGE_FILTERATION,
  };
  //RANSAC iteration       
#define RANSACITR 20000
  //Inlier influence                                                     
#define iInfluence 0.5

  string scanFormatToString(IOType format);
  IOType stringToScanFormat(string format);
  string scannerTypeToString(scanner_type type);
  scanner_type stringToScannerType(string type);
  string projectionMethodToString(projection_method method);
  projection_method stringToProjectionMethod(string method);
  string panoramaMapMethodToString(panorama_map_method method);
  panorama_map_method stringToPanoramaMapMethod(string method);
  string panoramaFormatToString(panorama_format format);
  string panoramaFormatToFileFormatString(panorama_format format);
  panorama_format stringToPanoramaFormat(string format);
  string panoramaTypeToString(panorama_type type);
  panorama_type stringToPanoramaType(string type);
  string featureDetectorMethodToString(feature_detector_method method);
  feature_detector_image_method stringToFeatureDetectorImageMethod(string method);
  string featureDetectorImageMethodToString(feature_detector_image_method method);
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
  string recoveredRangeFilterationMethodToString(recovered_range_filteration_method method);
  recovered_range_filteration_method stringToRecoveredRangeFilterationMethod(string method);
  
  //reflecrtance normaliation
  float normalizeTheReflectance(float reflecrtance, scanner_type sType, double minReflectance, double maxReflectance);

  //color compare to sort color_value_Tuples in first three color order
  bool colorCompare (const Color_Value_Tuple &lTuple, const Color_Value_Tuple &rTuple);
  //get all rgb colors sorted based on hsv
  vector<unsigned int> getAllRGBSortedByHSL(unsigned int size = 4096);
  //get all rgb colors sorted based on hsl
  vector<unsigned int> getAllRGBSortedByHSV(unsigned int size = 4096);
  //get all rgb colors sorted based on rgb
  vector<unsigned int> getAllRGBSortedByRGB(unsigned int size = 4096);

  //get all rgb colors sorted based on RBGB
  vector<unsigned int> getAllRGBSortedByRBGB();

  //convert color value to rgb 1 bit by 1 bit
  void colorToRGB1BitBy1Bit(unsigned int color, unsigned int &R, unsigned int &G, unsigned int &B);
  //convert color value to rgb 2 bit by 2 bit
  void colorToRGB2BitBy2Bit(unsigned int color, unsigned int &R, unsigned int &G, unsigned int &B);
  //convert color value to rgb 4 bit by 4 bit
  void colorToRGB4BitBy4Bit(unsigned int color, unsigned int &R, unsigned int &G, unsigned int &B);
  //convert color value to rgb 8 bit by 8 bit
  void colorToRGB8BitBy8Bit(unsigned int color, unsigned int &R, unsigned int &G, unsigned int &B);
  
}
#endif /* FBR_GLOBAL_H_ */
