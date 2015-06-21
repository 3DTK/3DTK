/*
 * fbr_global implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file 
 * @brief Implementation of global functions for feature based registration 
 * @author Hamidreza Houshiar, Jacobs University Bremen, Germany
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
    case MANUAL:
      sType = "MANUAL";
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
    else if(strcasecmp(type.c_str(), "MANUAL") == 0) return MANUAL;
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
    case AZIMUTHAL:
      sMethod = "AZIMUTHAL";
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
    else if(strcasecmp(method.c_str(), "AZIMUTHAL") == 0) return AZIMUTHAL;
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
    case FULL:
      sMethod = "FULL";
      break;
    case NON:
      sMethod = "NON";
      break;
    default:
      throw std::runtime_error(std::string("panorama map method ") + to_string(method) + std::string(" could not be matched to a panorama map method"));
    }
    return sMethod;
  }
  
  panorama_map_method stringToPanoramaMapMethod(string method){
    if(strcasecmp(method.c_str(), "FARTHEST") == 0) return FARTHEST;
    else if(strcasecmp(method.c_str(), "EXTENDED") == 0) return EXTENDED;
    else if(strcasecmp(method.c_str(), "FULL") == 0) return FULL;
    else if(strcasecmp(method.c_str(), "NON") == 0) return NON;
    else throw std::runtime_error(std::string("panorama map method ") + method + std::string(" is unknown"));
  }

  string panoramaFormatToString(panorama_format format){
    string sFormat;
    switch(format){
    case PNG:
      sFormat = "PNG";
      break;
    case JPEG:
      sFormat = "JPEG";
      break;
    case JPEG2000:
      sFormat = "JPEG2000";
      break;
    case TIFF:
      sFormat = "TIFF";
      break;
    case WebP:
      sFormat = "WebP";
      break;
    default:
      throw std::runtime_error(std::string("panorama fromat ") + to_string(format) + std::string(" could not be matched to a panorama format"));
    }
    return sFormat;
  }

  string panoramaFormatToFileFormatString(panorama_format format){
    string sFormat;
    switch(format){
    case PNG:
      sFormat = "png";
      break;
    case JPEG:
      sFormat = "jpg";
      break;
    case JPEG2000:
      sFormat = "jp2";
      break;
    case TIFF:
      sFormat = "tiff";
      break;
    case WebP:
      sFormat = "webp";
      break;
    default:
      throw std::runtime_error(std::string("panorama fromat ") + to_string(format) + std::string(" could not be matched to a panorama format"));
    }
    return sFormat;
  }
 
  panorama_format stringToPanoramaFormat(string format){
    if(strcasecmp(format.c_str(), "PNG") == 0) return PNG;
    else if(strcasecmp(format.c_str(), "JPEG") == 0) return JPEG;
    else if(strcasecmp(format.c_str(), "JPEG2000") == 0) return JPEG2000;
    else if(strcasecmp(format.c_str(), "TIFF") == 0) return TIFF;
    else if(strcasecmp(format.c_str(), "WebP") == 0) return WebP;
    else throw std::runtime_error(std::string("panorama format ") + format + std::string(" is unknown"));
  }
  
  string panoramaTypeToString(panorama_type type){
    string sType;
    switch(type){
    case ThreeChannel24BitRange:
      sType = "ThreeChannel24BitRange";
      break;
    case ThreeGrayscaleRange:
      sType = "ThreeGrayscaleRange";
      break;
    default:
      throw std::runtime_error(std::string("panorama type ") + to_string(type) + std::string(" could not be matched to a panorama type"));
    }
    return sType;
  }
 
  panorama_type stringToPanoramaType(string type){
    if(strcasecmp(type.c_str(), "ThreeChannel24BitRange") == 0) return ThreeChannel24BitRange;
    else if(strcasecmp(type.c_str(), "ThreeGrayscaleRange") == 0) return ThreeGrayscaleRange;
    else throw std::runtime_error(std::string("panorama type ") + type + std::string(" is unknown"));
  }

  string featureDetectorImageMethodToString(feature_detector_image_method method)
  {
    string sMethod;
    switch(method){
    case REFLECTANCE:
      sMethod = "REFLECTANCE";
      break;
    case COLOR:
      sMethod = "COLOR";
      break;
    default:
      throw std::runtime_error(std::string("feature detector image ") + to_string(method) + std::string(" could not be matched to a feature detector image"));

    }
    return sMethod;
  }

  feature_detector_image_method stringToFeatureDetectorImageMethod(string method){
    if(strcasecmp(method.c_str(), "REFLECTANCE") == 0) return REFLECTANCE;
    else if(strcasecmp(method.c_str(), "COLOR") == 0) return COLOR;
    else throw std::runtime_error(std::string("feature detector image ") + method + std::string(" is unknown"));
  }

  string featureDetectorMethodToString(feature_detector_method method){
    string sMethod;
    switch(method){
#ifdef WITH_OPENCV_NONFREE
    case SIFT_DET:
      sMethod = "SIFT_DET";
      break;
    case SURF_DET:
      sMethod = "SURF_DET";
      break;
#endif
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
#ifdef WITH_OPENCV_NONFREE
    if(strcasecmp(method.c_str(), "SIFT") == 0) return SIFT_DET;
    else if(strcasecmp(method.c_str(), "SURF") == 0) return SURF_DET;
    else
#endif
    if(strcasecmp(method.c_str(), "ORB") == 0) return ORB_DET;
    else if(strcasecmp(method.c_str(), "FAST") == 0) return FAST_DET;
    else if(strcasecmp(method.c_str(), "STAR") == 0) return STAR_DET;
    else throw std::runtime_error(std::string("feature detector method ") + method + std::string(" is unknown"));
  }
  
  string featureDescriptorMethodToString(feature_descriptor_method method){
    string sMethod;
    switch(method){
#ifdef WITH_OPENCV_NONFREE
    case SIFT_DES:
      sMethod = "SIFT_DES";
      break;
    case SURF_DES:
      sMethod = "SURF_DES";
      break;
#endif
    case ORB_DES:
      sMethod = "ORB_DES";
      break;
    default:
      throw std::runtime_error(std::string("feature descriptor method ") + to_string(method) + std::string(" could not be matched to a feature descriptor method"));
    }
    return sMethod;
  }
  
  feature_descriptor_method stringToFeatureDescriptorMethod(string method){
#ifdef WITH_OPENCV_NONFREE
    if(strcasecmp(method.c_str(), "SIFT") == 0) return SIFT_DES;
    else if(strcasecmp(method.c_str(), "SURF") == 0) return SURF_DES;
    else
#endif
    if(strcasecmp(method.c_str(), "ORB") == 0) return ORB_DES;
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

  string recoveredRangeFilterationMethodToString(recovered_range_filteration_method method)
  {
    string recoveredRangeFilterationMethod;
    switch(method){
    case INTERQUARTILE:
      recoveredRangeFilterationMethod = "INTERQUARTILE";
      break;
    case INTERQUARTILE_AVERAGEDIFF:
      recoveredRangeFilterationMethod = "INTERQUARTILE_AVERAGEDIFF";
	break;
    case DISABLE_RECOVERED_RANGE_FILTERATION:
      recoveredRangeFilterationMethod = "DISABLE_RECOVERED_RANGE_FILTERATION";
	break;
    default:
      throw std::runtime_error(std::string("recovered range filtration method ") + to_string(method) + std::string(" could not be matched to a recovered range filtration method"));
    }
    return recoveredRangeFilterationMethod;
  }
  
  recovered_range_filteration_method stringToRecoveredRangeFilterationMethod(string method)
  {
    if(strcasecmp(method.c_str(), "INTERQUARTILE") == 0) return INTERQUARTILE;
    else if(strcasecmp(method.c_str(), "INTERQUARTILE_AVERAGEDIFF") == 0) return INTERQUARTILE_AVERAGEDIFF;
    else if(strcasecmp(method.c_str(), "DISABLE_RECOVERED_RANGE_FILTERATION") == 0) return DISABLE_RECOVERED_RANGE_FILTERATION;
    else throw std::runtime_error(std::string("recovered range filtration method ") + method + std::string(" is unknown"));
  }

  //reflectance normalization
  float normalizeTheReflectance(float reflectance, scanner_type sType, double minReflectance, double maxReflectance)
  {
    
    //normalize the reflectance
    if(sType == fbr::RIEGL)
      {
	reflectance += 32;
	reflectance /= 64;
	reflectance -= 0.2;
	reflectance /= 0.3;
      }
    if(sType == fbr::FARO)
      {
	reflectance -= 1250;
	reflectance /= 800;
      }
    if(sType == MANUAL)
      {                                                                                                            
	reflectance = (reflectance - (minReflectance))/(maxReflectance-minReflectance);
      }
    if(sType != fbr::NONE)
      {
	if (reflectance < 0) reflectance = 0;
	if (reflectance > 1) reflectance = 1;
      }
    
    return reflectance;
  }

  //color compare to sort color_value_Tuples in first three color order
  bool colorCompare (const Color_Value_Tuple &lTuple, const Color_Value_Tuple &rTuple)
  {
    
    if(get<0>(lTuple) == get<0>(rTuple))
      {
	if(get<1>(lTuple) == get<1>(rTuple))
	  {
	    return get<2>(lTuple) < get<2>(rTuple);
	  }
	else
	  return get<1>(lTuple) < get<1>(rTuple);
      }
    else
      return get<0>(lTuple) < get<0>(rTuple);
  }
  
  //get all rgb colors sorted based on hsv
  vector<unsigned int> getAllRGBSortedByHSL(unsigned int size)
  {
    vector<unsigned int> colorMap;
    vector<Color_Value_Tuple> colors;

    for(int C = 0; C <size*size; C++)
      {
	unsigned int R,G,B;
	R = (C>>16) & 0xFF;
	G = (C>>8) & 0xFF;
	B = (C>>0) & 0xFF;

	double dR = R/255.0;
	double dG = G/255.0;
	double dB = B/255.0;
	
	double dMax = std::max(dR, std::max(dG, dB));
	double dMin = std::min(dR, std::min(dG, dB));
	
	double H = 0.0, S, L = (dMax + dMin) / 2;
	
	if(dMax == dMin)
	  {
	    H = S = 0; // achromatic
	  }
	else
	  {
	    double delta = dMax - dMin;
	    S = L > 0.5 ? delta / (2 - dMax - dMin) : delta / (dMax + dMin);
	    
	    if(dMax == dR)
	      H = (dG - dB) / delta + (dG < dB ? 6 : 0);
	    else if(dMax == dG)
	      H = (dB - dR) / delta + 2;
	    else if(dMax == dB)
	      H = (dR - dG) / delta + 4;
	    
	    H /= 6;
	  }
	colors.push_back(make_tuple(H, S, L, C));
      }
    
    sort(colors.begin(), colors.end(), colorCompare);

    for(vector<Color_Value_Tuple>::iterator itr = colors.begin(); itr != colors.end(); itr++)
    {
      unsigned int  colorVal = get<3>(*itr);
      colorMap.push_back(colorVal);
    }
    return colorMap;
  }

  //get all rgb colors sorted based on hsl
  vector<unsigned int> getAllRGBSortedByHSV(unsigned int size)
  {
    vector<unsigned int> colorMap;
    vector<Color_Value_Tuple> colors;

    for(int C = 0; C <size*size; C++)
      {
	unsigned int R,G,B;
	R = (C>>16) & 0xFF;
	G = (C>>8) & 0xFF;
	B = (C>>0) & 0xFF;

	double dR = R/255.0;
	double dG = G/255.0;
	double dB = B/255.0;
	
	double dMax = std::max(dR, std::max(dG, dB));
	double dMin = std::min(dR, std::min(dG, dB));
	
	double H = 0.0, S, V = dMax;
	double delta = dMax - dMin;
	S = dMax == 0 ? 0 : delta / dMax;
	
	if(dMax == dMin)
	  {
	    H = 0; // achromatic
	  }
	else
	  {
	    if(dMax == dR)
	      H = (dG - dB) / delta + (dG < dB ? 6 : 0);
	    else if(dMax == dG)
	      H = (dB - dR) / delta + 2;
	    else if(dMax == dB)
	      H = (dR - dG) / delta + 4;
	    
	    H /= 6;
	  }
	colors.push_back(make_tuple(H, S, V, C));
      }
    
    sort(colors.begin(), colors.end(), colorCompare);

    for(vector<Color_Value_Tuple>::iterator itr = colors.begin(); itr != colors.end(); itr++)
    {
      unsigned int  colorVal = get<3>(*itr);
      colorMap.push_back(colorVal);
    }
    return colorMap;
  }
  
  //get all rgb colors sorted based on rgb
  vector<unsigned int> getAllRGBSortedByRGB(unsigned int size)
  {
    vector<unsigned int> colorMap;
    vector<Color_Value_Tuple> colors;

    for(int C = 0; C <size*size; C++)
      {
	unsigned int R,G,B;
	R = (C>>16) & 0xFF;
	G = (C>>8) & 0xFF;
	B = (C>>0) & 0xFF;

	colors.push_back(make_tuple(R, G, B, C));
      }
    
    sort(colors.begin(), colors.end(), colorCompare);

    for(vector<Color_Value_Tuple>::iterator itr = colors.begin(); itr != colors.end(); itr++)
    {
      unsigned int  colorVal = get<3>(*itr);
      colorMap.push_back(colorVal);
    }
    return colorMap;
  }


  //get all rgb colors sorted based on rgb
  vector<unsigned int> getAllRGBSortedByRBGB()
  {
    vector<unsigned int> colorMap;
    
    for (int j = 0; j < 256; j++)//red //255=interesting quirk
      {
	for (int k = 0; k < 16; k++)//blue_MSBs
	  {
	    for (int l = 0; l < 256; l++)//green 255=interesting quirk
	      {
		for (int m = 0; m < 16; m++)//blue_lsbs
		  {
		    //unsigned int  val = (j<<16)+(k<<12)+(l)+(m<<8);
		    unsigned int  val = (j<<16) | (k<<12) | (l) | (m<<8);
		    colorMap.push_back(val);
		  }
	      }
	  }
      }
    
    return colorMap;
  }

  //convert color value to rgb 1 bit by 1 bit
  void colorToRGB1BitBy1Bit(unsigned int color, unsigned int &R, unsigned int &G, unsigned int &B)
  {
    unsigned char temp;
    
    //1 bit
    temp = (color >> 23) & 0x01;
    R = (temp << 7);
    temp = (color >> 20) & 0x01;
    R += (temp << 6);
    temp = (color >> 17) & 0x01;
    R += (temp << 5);
    temp = (color >> 14) & 0x01;
    R += (temp << 4);
    temp = (color >> 11) & 0x01;
    R += (temp << 3);
    temp = (color >> 8) & 0x01;
    R += (temp << 2);
    temp = (color >> 5) & 0x01;
    R += (temp << 1);
    temp = (color >> 2) & 0x01;
    R += (temp << 0);
    
    temp = (color >> 22) & 0x01;
    G = (temp << 7);
    temp = (color >> 19) & 0x01;
    G += (temp << 6);
    temp = (color >> 16) & 0x01;
    G += (temp << 5);
    temp = (color >> 13) & 0x01;
    G += (temp << 4);
    temp = (color >> 10) & 0x01;
    G += (temp << 3);
    temp = (color >> 7) & 0x01;
    G += (temp << 2);
    temp = (color >> 4) & 0x01;
    G += (temp << 1);
    temp = (color >> 1) & 0x01;
    G += (temp << 0);
    
    temp = (color >> 21) & 0x01;
    B = (temp << 7);
    temp = (color >> 18) & 0x01;
    B += (temp << 6);
    temp = (color >> 15) & 0x01;
    B += (temp << 5);
    temp = (color >> 12) & 0x01;
    B += (temp << 4);
    temp = (color >> 9) & 0x01;
    B += (temp << 3);
    temp = (color >> 6) & 0x01;
    B += (temp << 2);
    temp = (color >> 3) & 0x01;
    B += (temp << 1);
    temp = (color >> 0) & 0x01;
    B += (temp << 0);
  }

  //convert color value to rgb 2 bit by 2 bit
  void colorToRGB2BitBy2Bit(unsigned int color, unsigned int &R, unsigned int &G, unsigned int &B)
  {
    unsigned char temp;

    //2 bits
    temp = (color >> 22) & 0x03;
    R = (temp << 6);
    temp = (color >> 16) & 0x03;
    R += (temp << 4);
    temp = (color >> 10) & 0x03;
    R += (temp << 2);
    temp = (color >> 4) & 0x03;
    R += (temp << 0);
    
    temp = (color >> 20) & 0x03;
    G = (temp << 6);
    temp = (color >> 14) & 0x03;
    G += (temp << 4);
    temp = (color >> 8) & 0x03;
    G += (temp << 2);
    temp = (color >> 2) & 0x03;
    G += (temp << 0);
    
    temp = (color >> 18) & 0x03;
    B = (temp << 6);
    temp = (color >> 12) & 0x03;
    B += (temp << 4);
    temp = (color >> 6) & 0x03;
    B += (temp << 2);
    temp = (color >> 0) & 0x03;
    B += (temp << 0);
  }

  //convert color value to rgb 4 bit by 4 bit
  void colorToRGB4BitBy4Bit(unsigned int color, unsigned int &R, unsigned int &G, unsigned int &B)
  {
    unsigned char temp;

    //4bits
    temp = (color >> 20) & 0x0F;
    R = (temp << 4);
    temp = (color >> 8) & 0x0F;
    R += (temp << 0);
    
    temp = (color >> 16) & 0x0F;
    G = (temp << 4);
    temp = (color >> 4) & 0x0F;
    G += (temp << 0);
    
    temp = (color >> 12) & 0x0F;
    B = (temp << 4);
    temp = (color >> 0) & 0x0F;
    B += (temp << 0);
  }

  //convert color value to rgb 8 bit by 8 bit
  void colorToRGB8BitBy8Bit(unsigned int color, unsigned int &R, unsigned int &G, unsigned int &B)
  {
    //8 bits
    unsigned int temp;
    temp = (color >> 16) & 0xFF;
    R = (temp << 0);
    
    temp = (color >> 8) & 0xFF;
    G = (temp << 0);
    
    temp = (color >> 0) & 0xFF;
    B = (temp << 0);
  }

}
