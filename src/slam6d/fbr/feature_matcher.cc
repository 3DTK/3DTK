/*
 * feature_mathcer implementation
 *
 * Copyright (C) HamidReza Houshiar
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/fbr/feature_matcher.h"

using namespace std;

namespace fbr{
  
  void feature_matcher::init(matcher_method method, int k, double r, matching_filtration_method filtration){
    mMethod = method;
    knn = k;
    radius = r;
    nOfMatches = 0;
    nOfFilteredMatches = 0;
    mFiltrationMethod = filtration;
  }

  feature_matcher::feature_matcher(){
    init(RATIO, 0, 0, DISABLE_MATCHING_FILTER);
  }

  feature_matcher::feature_matcher(matcher_method method){
    if(method == KNN)
      init(method, 3, 0, DISABLE_MATCHING_FILTER);
    else if(method == RADIUS)
      init(method, 0, 1, DISABLE_MATCHING_FILTER);
    else
      init(method, 0, 0, DISABLE_MATCHING_FILTER);
  }

  feature_matcher::feature_matcher(matcher_method method, double p){
    if(method == KNN)
      init(method, p, 0, DISABLE_MATCHING_FILTER);
    else if(method == RADIUS)
      init(method, 0, p, DISABLE_MATCHING_FILTER);
    else
      init(method, 0, 0, DISABLE_MATCHING_FILTER);
  }

  feature_matcher::feature_matcher(matcher_method method, double p, matching_filtration_method filtration){
    if(method == KNN)
      init(method, p, 0, filtration);
    else if(method == RADIUS)
      init(method, 0, p, filtration);
    else
      init(method, 0, 0, filtration);
  }

  void feature_matcher::findMatches(feature qFeature, feature tFeature){
    vector< cv::DMatch > qtInitialMatches;
    vector<vector<cv::DMatch> > qtInitialMatchesVector;
    if(qFeature.getFeatures().size() == 0 || tFeature.getFeatures().size() == 0){
      cout<<"No features has found in one or both scans!!"<<endl;
      exit(-1);
    }

    //Matching descriptors using one of the mMethods for SURF and SIFT feature descriptors
    if(qFeature.getDescriptorMethod() != tFeature.getDescriptorMethod()){
      cout<<"inputs features don't have the same descriptors!"<<endl;
      cout<<"qFeature DescriptorMethod="<<qFeature.getDescriptorMethod()<<endl;
      cout<<"tFeature DescriptorMethod="<<tFeature.getDescriptorMethod()<<endl;
    } else if( qFeature.getDescriptorMethod() == SURF_DES || qFeature.getDescriptorMethod() == SIFT_DES){
      if(mMethod == KNN){
	cv::FlannBasedMatcher matcher;
	matcher.knnMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, knn);
      }
      if(mMethod == RADIUS){
	cv::FlannBasedMatcher matcher;
	matcher.radiusMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, radius);
      }
      if(mMethod == RATIO){
	cv::FlannBasedMatcher matcher;
	matcher.knnMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, 2);
      }
      if(mMethod == BRUTEFORCE){	
	//opencv 2.4
#if (CV_MAJOR_VERSION >= 2) && (CV_MINOR_VERSION >= 4)
	cv::BFMatcher matcher (cv::NORM_L2);
#else  //older version of opencv than 2.4
	cv::BruteForceMatcher< cv::L2<float> > matcher;
#endif
	matcher.match(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatches);
	qtInitialMatchesVector.push_back(qtInitialMatches);
      }
      if(mMethod == FLANN){
	cv::FlannBasedMatcher matcher;
	matcher.match(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatches);
	qtInitialMatchesVector.push_back(qtInitialMatches);
      }
    }
    //Matching descriptors using Hamming distance for ORB descriptor
    else if(qFeature.getDescriptorMethod() == ORB_DES){
      if(mMethod == KNN){
	//opencv 2.4
#if (CV_MAJOR_VERSION >= 2) && (CV_MINOR_VERSION >= 4)
	cv::BFMatcher matcher (cv::NORM_HAMMING);
#else  //older version of opencv than 2.4
	cv::BruteForceMatcher< cv::Hamming > matcher;
#endif
	matcher.knnMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, knn);
      }
      if(mMethod == RADIUS){
	//opencv 2.4
#if (CV_MAJOR_VERSION >= 2) && (CV_MINOR_VERSION >= 4)
	cv::BFMatcher matcher (cv::NORM_HAMMING);
#else  //older version of opencv than 2.4
	cv::BruteForceMatcher< cv::Hamming > matcher;
#endif
	matcher.radiusMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, radius);
      }
      if(mMethod == RATIO){
	//opencv 2.4
#if (CV_MAJOR_VERSION >= 2) && (CV_MINOR_VERSION >= 4)
	cv::BFMatcher matcher (cv::NORM_HAMMING);
#else  //older version of opencv than 2.4
	cv::BruteForceMatcher< cv::Hamming > matcher;
#endif
	matcher.knnMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, 2);
      }
      if(mMethod == BRUTEFORCE){
	//opencv 2.4
#if (CV_MAJOR_VERSION >= 2) && (CV_MINOR_VERSION >= 4)
	cv::BFMatcher matcher (cv::NORM_HAMMING);
#else  //older version of opencv than 2.4
	cv::BruteForceMatcher< cv::Hamming > matcher;
#endif
	matcher.match(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatches);
	qtInitialMatchesVector.push_back(qtInitialMatches);
      }
    }
    if(mMethod == RATIO){
      for(unsigned int i = 0; i < qtInitialMatchesVector.size(); i++){
	float ratio = qtInitialMatchesVector[i][0].distance/qtInitialMatchesVector[i][1].distance;
	if(ratio < 0.66)
	  matches.push_back(qtInitialMatchesVector[i][0]);
      }
    }else{
      for(unsigned int i = 0; i < qtInitialMatchesVector.size(); i++){
	for(unsigned int j = 0; j < qtInitialMatchesVector[i].size(); j++){
	  matches.push_back(qtInitialMatchesVector[i][j]);
	}
      }
    }
  }
  
  void feature_matcher::match(feature qFeature, feature tFeature){
    vector< cv::DMatch > gMatches;
    findMatches(qFeature, tFeature);

    if(mFiltrationMethod == FUNDEMENTAL_MATRIX){
      //filter the matches with RANSAC and FundementalMatrix
      vector<cv::Point2f> points_1, points_2;
      for( unsigned int i = 0; i < matches.size(); i++ ){
	//Get the keypoints from the intersection of both matches
	points_1.push_back( qFeature.getFeatures()[ matches[i].queryIdx ].pt );
	points_2.push_back( tFeature.getFeatures()[ matches[i].trainIdx ].pt );
      }
      //calculating the fundemental matrix
      cv::Mat fStatus;
      cv::Mat fundementalMatrix = findFundamentalMat( points_1, points_2, cv::FM_RANSAC, 3, 0.99, fStatus);
      cv::MatIterator_<uchar> it, end; 
      int counter = 0;
      //get the inliers from fundemental matrix
      for( it = fStatus.begin<uchar>(), end = fStatus.end<uchar>(); it != end; ++it){
	if(*it == 1)
	  gMatches.push_back(matches[counter]);
	counter++;
      }
      nOfMatches = matches.size();
      matches = gMatches;
      nOfFilteredMatches = matches.size();
    }else{
      nOfMatches = matches.size();
      nOfFilteredMatches = matches.size(); 
    }
  }
  
  vector<cv::DMatch> feature_matcher::getMatches(){
    return matches;
  }
  
  matcher_method feature_matcher::getMatcherMethod(){
    return mMethod;
  }
  
  matching_filtration_method feature_matcher::getMatchingFiltrationMethod(){
    return mFiltrationMethod;
  }

  unsigned int feature_matcher::getKnn(){
    return  knn;
  }

  double feature_matcher::getRadius(){
    return radius;
  }
  
  unsigned int feature_matcher::getNumberOfMatches(){
    return nOfMatches;
  }
  
  unsigned int feature_matcher::getNumberOfFilteredMatches(){
    return nOfFilteredMatches;
  }
  
  void feature_matcher::getDescription(){
    cout<<"number of Matches: "<<nOfMatches<<", number of Matches after filteration: "<<nOfFilteredMatches<<", matching method: "<<matcherMethodToString(mMethod)<<", matching filtration method: "<<matchingFiltrationMethodToString(mFiltrationMethod)<<", knn: "<<knn<<", radius: "<<radius<<"."<<endl;
    cout<<endl;
  }

  void feature_matcher::setMatchingFiltrationMethod(matching_filtration_method method){
    mFiltrationMethod = method;
  }

}
