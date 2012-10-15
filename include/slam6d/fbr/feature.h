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
    feature_filtration_method fFiltrationMethod;
    vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    enum description_method{
      FEATURE_DESCRIPTION,
      DESCRIPTOR_DESCRIPTION,
      ALL,
    };
    void featureFiltration(cv::Mat pImage, cv::Mat rImage);
  public:
    /**
     * constructor of feature class
     */
    feature();
    feature(feature_detector_method detector, feature_descriptor_method descriptor, feature_filtration_method filtration);
    /**
     * featureDetection : creates the features
     * @param pImage cv::Mat refletance panorama image
     * @param method feature_detector_method
     * @param rImage cv::Mat range image for filtration 
     * @param fMethod feature_filtration_method
     **/
    void featureDetection(cv::Mat pImage, feature_detector_method method, cv::Mat rImage, feature_filtration_method fMethod);
    void featureDetection(cv::Mat pImage, feature_detector_method method);
    void featureDetection(cv::Mat pImage);
    /**
     * featureDescription : creates descriptor of detected features
     * @param pImage cv::Mat reflectance panorama image
     * @param method feature_descriptor_method
     **/
    void featureDescription(cv::Mat pImage, feature_descriptor_method method);
    void featureDescription(cv::Mat pImage);
    /**
     * all sets of getters for the class
     */
    feature_detector_method getDetectorMethod();
    feature_descriptor_method getDescriptorMethod();
    feature_filtration_method getFeatureFiltrationMethod();
    vector<cv::KeyPoint> getFeatures();
    cv::Mat getDescriptors();
    void setFeatures(vector<cv::KeyPoint> keypoint);
    void setDescriptors(cv::Mat descriptor);
    void getDescription(description_method method=ALL);
    unsigned int getNumberOfFeatures();
  };
}
#endif /* FEATURE_H_ */ 
