/*
 * feature implementation
 *
 * Copyright (C) HamidReza Houshiar
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/fbr/feature.h"

using namespace std;

namespace fbr{

  feature::feature(){
    fDetectorMethod = SIFT_DET;
    fDescriptorMethod = SIFT_DES;
    fFiltrationMethod = DISABLE_FILTER;
  }
  feature::feature(feature_detector_method detector, feature_descriptor_method descriptor, feature_filtration_method filtration){
    fDetectorMethod = detector;
    fDescriptorMethod = descriptor;
    fFiltrationMethod = filtration;
  }
  void feature::featureDetection(cv::Mat pImage, feature_detector_method method, cv::Mat rImage, feature_filtration_method fMethod){
    fDetectorMethod = method;
    fFiltrationMethod = fMethod;
    switch(fDetectorMethod){
      //Detect the keypoints using SURF Detector
    case SURF_DET:{
      double minHessian = 400;
      cv::SurfFeatureDetector detector(minHessian);
      detector.detect(pImage, keypoints);
      break;
    }
      //Detect the keypoints using SIFT Detector
    case SIFT_DET:{
      cv::SiftFeatureDetector detector;
      detector.detect(pImage, keypoints);
      break;
    }
      //Detect the keypoints using ORB Detector
    case ORB_DET:{
      cv::OrbFeatureDetector detector;
      detector.detect(pImage, keypoints);
      break;
    }
      //Detect the keypoints using FAST Detector
    case FAST_DET:{
      cv::FastFeatureDetector detector;
      detector.detect(pImage, keypoints);
      break;
    }
      //Detect the keypoints using STAR Detector
    case STAR_DET:{
      cv::StarFeatureDetector detector;
      detector.detect(pImage, keypoints);
      break;
    }
    }
    featureFiltration(pImage, rImage);
  }
  
  void feature::featureDetection(cv::Mat pImage, feature_detector_method method){
    cv::Mat rImage;
    featureDetection(pImage, method, rImage, fFiltrationMethod);
  }
  
  void feature::featureDetection(cv::Mat pImage){
    featureDetection(pImage, fDetectorMethod);
  }

  void feature::featureDescription(cv::Mat pImage, feature_descriptor_method method){
    
    fDescriptorMethod = method;
    if(keypoints.size() == 0)
      featureDetection(pImage);
    switch(fDescriptorMethod){
    case SURF_DES:{
      //Create descriptor using SURF
      cv::SurfDescriptorExtractor extractor;
      extractor.compute(pImage, keypoints, descriptors);
      break;
    }
    case SIFT_DES:{
      //Create descriptor using SIFT
      cv::SiftDescriptorExtractor extractor;
      extractor.compute(pImage, keypoints, descriptors);
      break;
    }
    case ORB_DES:{
      //Create descriptor using ORB
      cv::OrbDescriptorExtractor extractor;
      extractor.compute(pImage, keypoints, descriptors);
      break;
    }
    } 
  }   

  void feature::featureDescription(cv::Mat pImage){
    featureDescription(pImage, fDescriptorMethod);
  }
  
  feature_detector_method feature::getDetectorMethod(){
    return fDetectorMethod;
  }
  
  feature_descriptor_method feature::getDescriptorMethod(){
    return fDescriptorMethod;
  }

  feature_filtration_method feature::getFeatureFiltrationMethod(){
    return fFiltrationMethod;
  }

  //check for the keypoints vector not to be empty
  vector<cv::KeyPoint> feature::getFeatures(){
    return keypoints;
  }
  
  //check for the descriptor Mat not to be empty
  cv::Mat feature::getDescriptors(){
    return descriptors;
  }
  
  void feature::setFeatures(vector<cv::KeyPoint> keypoint){
    keypoints = keypoint;
  }
  
  void feature::setDescriptors(cv::Mat descriptor){
    descriptors = descriptor;
  }

  void feature::getDescription(description_method method){   
    if(method == FEATURE_DESCRIPTION)
      cout<<"fDetectorMethod: "<<featureDetectorMethodToString(fDetectorMethod)<<", number of detected features: "<<keypoints.size()<<", feature filtration method: "<<featureFiltrationMethodToString(fFiltrationMethod)<<"."<<endl;
    else if(method == DESCRIPTOR_DESCRIPTION)
      cout<<"fDescriptorMethod: "<<featureDescriptorMethodToString(fDescriptorMethod)<<"."<<endl;
    else
      cout<<"fDetectorMethod: "<<featureDetectorMethodToString(fDetectorMethod)<<", number of detected features: "<<keypoints.size()<<", feature filtration method: "<<featureFiltrationMethodToString(fFiltrationMethod)<<", fDescriptorMethod: "<<featureDescriptorMethodToString(fDescriptorMethod)<<"."<<endl;
    cout<<endl;
  } 

  unsigned int feature::getNumberOfFeatures(){
    return keypoints.size();
  }
  
  void feature::featureFiltration(cv::Mat pImage, cv::Mat rImage){
    vector<cv::KeyPoint> filteredKeypoints;
    if(fFiltrationMethod == OCCLUSION){
      for(unsigned int i = 0; i < keypoints.size(); i++){
	int x, y;
	x = keypoints[i].pt.x;
	y = keypoints[i].pt.y;
	float range[8];
	if(rImage.at<float>(y,x) != 0){
	  range[0] = rImage.at<float>(y,x)-rImage.at<float>(y+1,x+1);
	  range[1] = rImage.at<float>(y,x)-rImage.at<float>(y,x+1);
	  range[2] = rImage.at<float>(y,x)-rImage.at<float>(y-1,x+1);
	  range[3] = rImage.at<float>(y,x)-rImage.at<float>(y-1,x);
	  range[4] = rImage.at<float>(y,x)-rImage.at<float>(y-1,x-1);
	  range[5] = rImage.at<float>(y,x)-rImage.at<float>(y,x-1);
	  range[6] = rImage.at<float>(y,x)-rImage.at<float>(y+1,x-1);
	  range[7] = rImage.at<float>(y,x)-rImage.at<float>(y+1,x);

	  int count=0;
	  for(unsigned int j = 0; j < 8; j++){
	    if(range[j] < 20)
	      count++;
	  }
	  if(count == 8)
	    filteredKeypoints.push_back(keypoints[i]);
	}
      }
    }else if(fFiltrationMethod == STANDARD_DEVIATION){
      for(unsigned int i = 0; i < keypoints.size(); i++){
	int x, y;
	x = keypoints[i].pt.x;
	y = keypoints[i].pt.y;
	float range[9];
	if(rImage.at<float>(y,x) != 0){
	  range[0] = rImage.at<float>(y,x)-rImage.at<float>(y+1,x+1);
	  range[1] = rImage.at<float>(y,x)-rImage.at<float>(y,x+1);
	  range[2] = rImage.at<float>(y,x)-rImage.at<float>(y-1,x+1);
	  range[3] = rImage.at<float>(y,x)-rImage.at<float>(y-1,x);
	  range[4] = rImage.at<float>(y,x)-rImage.at<float>(y-1,x-1);
	  range[5] = rImage.at<float>(y,x)-rImage.at<float>(y,x-1);
	  range[6] = rImage.at<float>(y,x)-rImage.at<float>(y+1,x-1);
	  range[7] = rImage.at<float>(y,x)-rImage.at<float>(y+1,x);
	  range[8] = rImage.at<float>(y,x)-rImage.at<float>(y,x);

	  double t_std, r=0, temp=0;
	  for(unsigned int j = 0; j < 9; j++)
	    r += range[j];
	  r /= 9;
	  for(unsigned int j = 0; j < 9; j++)
	    temp += ((range[j] - r) * (range[j] - r));
	  t_std = sqrt(temp/9);
	  if(t_std < 0.1)
	    filteredKeypoints.push_back(keypoints[i]);
	}
      }
    }
    if(fFiltrationMethod != DISABLE_FILTER)
      keypoints = filteredKeypoints;
  }
}

