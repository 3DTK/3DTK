#include "slam6d/fbr/feature.h"

using namespace std;

namespace fbr{

  feature::feature(){
    fDetectorMethod = SIFT_DET;
    fDescriptorMethod = SIFT_DES;
  }
  
  void feature::featureDetection(cv::Mat pImage, feature_detector_method method){
    //Detect the keypoints using SURF Detector
    if(fDetectorMethod == SURF_DET){
      double minHessian = 400;
      cv::SurfFeatureDetector detector(minHessian);
      detector.detect(pImage, keypoints);
    }
    
    //Detect the keypoints using SIFT Detector
    if(fDetectorMethod == SIFT_DET){
      cv::SiftFeatureDetector detector;
      detector.detect(pImage, keypoints);
    }
    
    //Detect the keypoints using ORB Detector
    if(fDetectorMethod == ORB_DET){
      cv::OrbFeatureDetector detector;
      detector.detect(pImage, keypoints);
    }
    
    //Detect the keypoints using FAST Detector
    if(fDetectorMethod == FAST_DET){
      cv::FastFeatureDetector detector;
      detector.detect(pImage, keypoints);
    }
  
    //Detect the keypoints using STAR Detector
    if(fDetectorMethod == STAR_DET){
      cv::StarFeatureDetector detector;
      detector.detect(pImage, keypoints);
    }
  }
  
  void feature::featureDetection(cv::Mat pImage){
    featureDetection(pImage, fDetectorMethod);
  }

  void feature::featureDescription(cv::Mat pImage, feature_descriptor_method method){
    
    if(keypoints.size() == 0)
      featureDetection(pImage);
    
    //Create descriptor using SURF
    if(fDescriptorMethod == SURF_DES){
      cv::SurfDescriptorExtractor extractor;
      extractor.compute(pImage, keypoints, descriptors);
    }
    
    //Create descriptor using SIFT
    if(fDescriptorMethod == SIFT_DES){
      cv::SiftDescriptorExtractor extractor;
      extractor.compute(pImage, keypoints, descriptors);
    }
    
    //Create descriptor using ORB
    if(fDescriptorMethod == ORB_DES){
      cv::OrbDescriptorExtractor extractor;
      extractor.compute(pImage, keypoints, descriptors);
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
  
  //check for the keypoints vector not to be empty
  vector<cv::KeyPoint> feature::getFeatures(){
    return keypoints;
  }
  
  //check for the descriptor Mat not to be empty
  cv::Mat feature::getDescriptors(){
    return descriptors;
  }

  void feature::getDescription(description_method method){   
    if(method == FEATURE_DESCRIPTION)
      cout<<"fDetectorMethod: "<<featureDetectorMethodToString(fDetectorMethod)<<", number of detected features: "<<keypoints.size()<<"."<<endl;
    else if(method == DESCRIPTOR_DESCRIPTION)
      cout<<"fDescriptorMethod: "<<featureDescriptorMethodToString(fDescriptorMethod)<<"."<<endl;
    else
      cout<<"fDetectorMethod: "<<featureDetectorMethodToString(fDetectorMethod)<<", number of detected features: "<<keypoints.size()<<", fDescriptorMethod: "<<featureDescriptorMethodToString(fDescriptorMethod)<<"."<<endl;
    cout<<endl;
  } 

  unsigned int feature::getNumberOfFeatures(){
    return keypoints.size();
  }
}
