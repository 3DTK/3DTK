/*
 * fbr_global implementation
 *
 * Copyright (C) HamidReza Houshiar
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/fbr/fbr_global.h"
#include <stdexcept>

namespace fbr{
  
  string scanFormatToString(IOType format){
    return io_type_to_libname(format);
  }
  
  IOType stringToScanFormat(string format){
    return formatname_to_io_type(format.c_str());
  }  
  string scannerTypeToString(scanner_type type){
    string sType;
    switch(type){
    case NONE:
      sType = "NONE";
      break;
    case RIEGL:
      sType = "RIEGL";
      break;
    case FARO:
      sType = "FARO";
      break;
    default:
      throw std::runtime_error(std::string("scanner type") + to_string(type) + std::string(" could not be matched to a scanner type"));
    }
    return sType;
  }
  scanner_type stringToScannerType(string type){
    if(strcasecmp(type.c_str(), "NONE") == 0) return NONE;
    else if(strcasecmp(type.c_str(), "RIEGL") == 0) return RIEGL;
    else if(strcasecmp(type.c_str(), "FARO") == 0) return FARO;
    else throw std::runtime_error(std::string("scanner type ") + type + std::string(" is unknown"));
  }
  string projectionMethodToString(projection_method method){
    string sMethod;
    switch(method){
    case EQUIRECTANGULAR:
      sMethod = "EQUIRECTANGULAR";
      break;
    case CYLINDRICAL:
      sMethod = "CYLINDRICAL";
      break;
    case MERCATOR:
      sMethod = "MERCATOR";
      break;
    case RECTILINEAR:
      sMethod = "RECTILINEAR";
      break;
    case PANNINI:
      sMethod = "PANNINI";
      break;
    case STEREOGRAPHIC:
      sMethod = "STEREOGRAPHIC";
      break;
    case ZAXIS:
      sMethod = "ZAXIS";
      break;
    case CONIC:
      sMethod = "CONIC";
      break;
    case EQUALAREACYLINDRICAL:
      sMethod = "EQUALAREACYLINDRICAL";
      break;
    default:
      throw std::runtime_error(std::string("projection method ") + to_string(method) + std::string(" could not be matched to a projection method"));
    }
    return sMethod;
  }
  
  projection_method stringToProjectionMethod(string method){
    if(strcasecmp(method.c_str(), "EQUIRECTANGULAR") == 0) return EQUIRECTANGULAR;
    else if(strcasecmp(method.c_str(), "CYLINDRICAL") == 0) return CYLINDRICAL;
    else if(strcasecmp(method.c_str(), "MERCATOR") == 0) return MERCATOR;
    else if(strcasecmp(method.c_str(), "RECTILINEAR") == 0) return RECTILINEAR;
    else if(strcasecmp(method.c_str(), "PANNINI") == 0) return PANNINI;
    else if(strcasecmp(method.c_str(), "STEREOGRAPHIC") == 0) return STEREOGRAPHIC;
    else if(strcasecmp(method.c_str(), "ZAXIS") == 0) return ZAXIS;
    else if(strcasecmp(method.c_str(), "CONIC") == 0) return CONIC;
    else if(strcasecmp(method.c_str(), "EQUALAREACYLINDRICAL") == 0) return EQUALAREACYLINDRICAL;
    else throw std::runtime_error(std::string("projection method ") + method + std::string(" is unknown"));
  }
  
  string panoramaMapMethodToString(panorama_map_method method){
    string sMethod;
    switch(method){
    case FARTHEST:
      sMethod = "FARTHEST";
      break;
    case EXTENDED:
      sMethod = "EXTENDED";
      break;
    default:
      throw std::runtime_error(std::string("panorama map method ") + to_string(method) + std::string(" could not be matched to a panorama map method"));
    }
    return sMethod;
  }
  
  panorama_map_method stringToPanoramaMapMethod(string method){
    if(strcasecmp(method.c_str(), "FARTHEST") == 0) return FARTHEST;
    else if(strcasecmp(method.c_str(), "EXTENDED") == 0) return EXTENDED;
    else throw std::runtime_error(std::string("panorama map method ") + method + std::string(" is unknown"));
  }

  string featureDetectorMethodToString(feature_detector_method method){
    string sMethod;
    switch(method){
    case SIFT_DET:
      sMethod = "SIFT_DET";
      break;
    case SURF_DET:
      sMethod = "SURF_DET";
      break;
    case ORB_DET:
      sMethod = "ORB_DET";
      break;
    case FAST_DET:
      sMethod = "FAST_DET";
      break;
    case STAR_DET:
      sMethod = "STAR_DET";
      break;
    default:
      throw std::runtime_error(std::string("feature detector method ") + to_string(method) + std::string(" could not be matched to a feature detector method"));
    }
    return sMethod;
  }
  
  feature_detector_method stringToFeatureDetectorMethod(string method){
    if(strcasecmp(method.c_str(), "SIFT") == 0) return SIFT_DET;
    else if(strcasecmp(method.c_str(), "SURF") == 0) return SURF_DET;
    else if(strcasecmp(method.c_str(), "ORB") == 0) return ORB_DET;
    else if(strcasecmp(method.c_str(), "FAST") == 0) return FAST_DET;
    else if(strcasecmp(method.c_str(), "STAR") == 0) return STAR_DET;
    else throw std::runtime_error(std::string("feature detector method ") + method + std::string(" is unknown"));
  }
  
  string featureDescriptorMethodToString(feature_descriptor_method method){
    string sMethod;
    switch(method){
    case SIFT_DES:
      sMethod = "SIFT_DES";
      break;
    case SURF_DES:
      sMethod = "SURF_DES";
      break;
    case ORB_DES:
      sMethod = "ORB_DES";
      break;
    default:
      throw std::runtime_error(std::string("feature descriptor method ") + to_string(method) + std::string(" could not be matched to a feature descriptor method"));
    }
    return sMethod;
  }
  
  feature_descriptor_method stringToFeatureDescriptorMethod(string method){
    if(strcasecmp(method.c_str(), "SIFT") == 0) return SIFT_DES;
    else if(strcasecmp(method.c_str(), "SURF") == 0) return SURF_DES;
    else if(strcasecmp(method.c_str(), "ORB") == 0) return ORB_DES;
    else throw std::runtime_error(std::string("feature descriptor method ") + method + std::string(" is unknown"));
  }

  string matcherMethodToString(matcher_method method){
    string sMethod;
    switch(method){
    case BRUTEFORCE:
      sMethod = "BRUTEFORCE";
      break;
    case FLANN:
      sMethod = "FLANN";
      break;
    case KNN:
      sMethod = "KNN";
      break;
    case RADIUS:
      sMethod = "RADIUS";
      break;
    case RATIO:
      sMethod = "RATIO";
      break;
    default:
      throw std::runtime_error(std::string("matcher method ") + to_string(method) + std::string(" could not be matched to a matcher method"));
    }
    return sMethod;
  }
  
  matcher_method stringToMatcherMethod(string method){
    if(strcasecmp(method.c_str(), "BRUTEFORCE") == 0) return BRUTEFORCE;
    else if(strcasecmp(method.c_str(), "FLANN") == 0) return FLANN;
    else if(strcasecmp(method.c_str(), "KNN") == 0) return KNN;
    else if(strcasecmp(method.c_str(), "RADIUS") == 0) return RADIUS;
    else if(strcasecmp(method.c_str(), "RATIO") == 0) return RATIO;
    else throw std::runtime_error(std::string("matcher method ") + method + std::string(" is unknown"));
  }
  
  string registrationMethodToString(registration_method method){
    string sMethod;
    switch(method){
    case ALL:
      sMethod = "ALL";
      break;
    case RANSAC:
      sMethod = "RANSAC";
      break;
    case DISABLE:
      sMethod = "DISABLE";
      break;
    default:
      throw std::runtime_error(std::string("registration method ") + to_string(method) + std::string(" could not be matched to a registration method"));
    }
    return sMethod;
  }

  registration_method stringToRegistrationMethod(string method){
    if(strcasecmp(method.c_str(), "ALL") == 0) return ALL;
    else if(strcasecmp(method.c_str(), "RANSAC") == 0) return RANSAC;
    else if(strcasecmp(method.c_str(), "DISABLE") == 0) return DISABLE;
    else throw std::runtime_error(std::string("registration method ") + method + std::string(" is unknown"));
  }

  string featureFiltrationMethodToString(feature_filtration_method method){
    string fFiltrationMethod;
    switch(method){
    case OCCLUSION:
      fFiltrationMethod = "OCCLUSION";
      break;
    case STANDARD_DEVIATION:
      fFiltrationMethod = "STANDARD_DEVIATION";
      break;
    case DISABLE_FILTER:
      fFiltrationMethod = "DISABLE_FILTER";
      break;
    default:
      throw std::runtime_error(std::string("feature filtration method ") + to_string(method) + std::string(" could not be matched to a feature filtration method"));
    }
    return fFiltrationMethod;
  }

  feature_filtration_method stringToFeatureFiltrationMethod(string method){
    if(strcasecmp(method.c_str(), "OCCLUSION") == 0) return OCCLUSION;
    else if(strcasecmp(method.c_str(), "STANDARD_DEVIATION") == 0) return STANDARD_DEVIATION;
    else if(strcasecmp(method.c_str(), "DISABLE_FILTER") == 0) return DISABLE_FILTER;
    else throw std::runtime_error(std::string("feature filtration method ") + method + std::string(" is unknown"));
  }

  string matchingFiltrationMethodToString(matching_filtration_method method){
    string mFiltrationMethod;
    switch(method){
    case FUNDEMENTAL_MATRIX:
      mFiltrationMethod = "FUNDEMENTAL_MATRIX";
      break;
    case DISABLE_MATCHING_FILTER:
      mFiltrationMethod = "DISABLE_MATCHING_FILTER";
      break;
    default:
      throw std::runtime_error(std::string("matching filtration method ") + to_string(method) + std::string(" could not be matched to a matching filtration method"));
    }
    return mFiltrationMethod;
  }

  matching_filtration_method stringToMatchingFiltrationMethod(string method){
    if(strcasecmp(method.c_str(), "FUNDEMENTAL_MATRIX") == 0) return FUNDEMENTAL_MATRIX;
    else if(strcasecmp(method.c_str(), "DISABLE_MATCHING_FILTER") == 0) return DISABLE_MATCHING_FILTER;
    else throw std::runtime_error(std::string("matching filtration method ") + method + std::string(" is unknown"));
  }
}
