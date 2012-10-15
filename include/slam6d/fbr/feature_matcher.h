/**
 * @file feature_matcher.h
 * @brife matches two sets of features
 * This class is a feature matcher that findes the matching features.
 * It determines the matching features from two sets of feature descriptors and features.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @date Date: 2012/05/29 2:00
 */

#ifndef FEATURE_MATCHER_H_
#define FEATURE_MATCHER_H_

#include "fbr_global.h"
#include "feature.h"

namespace fbr{
  /**
   * @class feature_mathcer : match features that has same descriptor method. 
   * @param matches set of matched features
   * @param mMethod matcher_method
   * @param knn incase of knn search indecate the number of nearest neighbors
   * @param radius incase of radius search indicates the radius of the search
   * @param nOfMatches number of matches after the first step
   * @param nOfFilteredMatches number of matches after the filteration
   */
  class feature_matcher{
    vector<cv::DMatch> matches;
    matcher_method mMethod;
    unsigned int knn;
    double radius;
    unsigned int nOfMatches;
    unsigned int nOfFilteredMatches;
    matching_filtration_method mFiltrationMethod;
    
    void init(matcher_method method, int k, double r, matching_filtration_method filtration);
    void findMatches(feature qFeature, feature tFeature);
    
  public:
    feature_matcher();
    feature_matcher(matcher_method method);
    feature_matcher(matcher_method method, double p);
    feature_matcher(matcher_method method, double p, matching_filtration_method filtration);

    void match(feature qFeature, feature tFeature);

    vector<cv::DMatch> getMatches();
    matcher_method getMatcherMethod();
    matching_filtration_method getMatchingFiltrationMethod();
    unsigned int getKnn();
    double getRadius();
    unsigned int getNumberOfMatches();
    unsigned int getNumberOfFilteredMatches();
    void getDescription();

    void setMatchingFiltrationMethod(matching_filtration_method method);
  };
}
#endif /* FEATURE_MATCHER_H_ */
