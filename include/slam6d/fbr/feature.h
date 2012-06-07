/**
 * @file feature.h
 * @brief detects features on images and create the descriptor for them
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @date Date: 2012/5/24/ 2:00
 */

#ifndef FEATURE_H_
#define FEATURE_H_

#include "fbr_global.h"

using namespace std;

namespace fbr{
  /**
   * @class feature
   * @brief class to detect and describe features 
   * @param fDetectorMethod feature_detector_method
   * @param fDescriptorMethod feature_descriptor_method
   * @param keypoints vector containing the detected features
   * @param descriptors cv::Mat containig the descriptor of features
   */
  class feature{
    feature_detector_method fDetectorMethod;
    feature_descriptor_method fDescriptorMethod;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    enum description_method{
      FEATURE_DESCRIPTION,
      DESCRIPTOR_DESCRIPTION,
      ALL,
    };
  public:
    /**
     * constructor of feature class
     */
    feature();
    void featureDetection(cv::Mat pImage, feature_detector_method method);
    void featureDetection(cv::Mat pImage);
    void featureDescription(cv::Mat pImage, feature_descriptor_method method);
    void featureDescription(cv::Mat pImage);
    feature_detector_method getDetectorMethod();
    feature_descriptor_method getDescriptorMethod();
    vector<cv::KeyPoint> getFeatures();
    cv::Mat getDescriptors();
    void getDescription(description_method method=ALL);
    unsigned int getNumberOfFeatures();
  };
}
#endif /* FEATURE_H_ */ 
