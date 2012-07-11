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
  
  void feature_matcher::init(matcher_method method, int k, double r){
    mMethod = method;
    knn = k;
    radius = r;
    nOfMatches = 0;
    nOfFilteredMatches = 0;
  }

  feature_matcher::feature_matcher(){
    init(RATIO, 0, 0);
  }

  feature_matcher::feature_matcher(matcher_method method){
    if(method == KNN)
      init(method, 3, 0);
    else if(method == RADIUS)
      init(method, 0, 1);
    else
      init(method, 0, 0);
  }

  feature_matcher::feature_matcher(matcher_method method, double p){
    if(method == KNN)
      init(method, p, 0);
    else if(method == RADIUS)
      init(method, 0, p);
    else
      init(method, 0, 0);
  }
  
  void feature_matcher::match(feature qFeature, feature tFeature){
    vector< cv::DMatch > qtInitialMatches, tqInitialMatches, gMatches;
    vector<vector<cv::DMatch> > qtInitialMatchesVector, tqInitialMatchesVector;
    //Matching descriptors using one of the mMethods for SURF and SIFT feature descriptors
    if(qFeature.getDescriptorMethod() != tFeature.getDescriptorMethod()){
      cout<<"inputs features don't have the same descriptors!"<<endl;
      cout<<"qFeature DescriptorMethod="<<qFeature.getDescriptorMethod()<<endl;
      cout<<"tFeature DescriptorMethod="<<tFeature.getDescriptorMethod()<<endl;
    } else if( qFeature.getDescriptorMethod() == SURF_DES || qFeature.getDescriptorMethod() == SIFT_DES){
      
      if(mMethod == KNN){
	cv::FlannBasedMatcher matcher;
	matcher.knnMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, knn);
	matcher.knnMatch(tFeature.getDescriptors(), qFeature.getDescriptors(), tqInitialMatchesVector, knn);
      }
      if(mMethod == RADIUS){
	cv::FlannBasedMatcher matcher;
	matcher.radiusMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, radius);
	matcher.radiusMatch(tFeature.getDescriptors(), qFeature.getDescriptors(), tqInitialMatchesVector, radius);
      }
      if(mMethod == KNN || mMethod == RADIUS){
	//find the matches that has been found in both way
	for(unsigned int i = 0; i < qtInitialMatchesVector.size(); i++){
	  for(unsigned int j = 0; j < qtInitialMatchesVector[i].size(); j++){
	    cv::DMatch forward = qtInitialMatchesVector[i][j];
	    for(unsigned int k = 0; k < tqInitialMatchesVector[forward.trainIdx].size(); k++){
	      cv::DMatch backward = tqInitialMatchesVector[forward.trainIdx][k];
	      if(backward.trainIdx == forward.queryIdx)
		matches.push_back(forward);
	    }
	    //matches.push_back(qtInitialMatchesVector[i][j]);
	  }
	}
      }
      if(mMethod == RATIO){
	cv::FlannBasedMatcher matcher;
	matcher.knnMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, 2);
	for(unsigned int i = 0; i < qtInitialMatchesVector.size(); i++){
	  float ratio = qtInitialMatchesVector[i][0].distance/qtInitialMatchesVector[i][1].distance;
	  if(ratio < 0.8)
	    matches.push_back(qtInitialMatchesVector[i][0]);
	}	
	matcher.knnMatch(tFeature.getDescriptors(), qFeature.getDescriptors(), tqInitialMatchesVector, 2);
	for(unsigned int i = 0; i < tqInitialMatchesVector.size(); i++){
	  float ratio = tqInitialMatchesVector[i][0].distance/tqInitialMatchesVector[i][1].distance;
	  if(ratio < 0.8){
	    cv::DMatch tq_qt;
	    tq_qt.queryIdx = tqInitialMatchesVector[i][0].trainIdx;
	    tq_qt.trainIdx = tqInitialMatchesVector[i][0].queryIdx;
	    tq_qt.imgIdx = tqInitialMatchesVector[i][0].imgIdx;
	    tq_qt.distance = tqInitialMatchesVector[i][0].distance;
	    matches.push_back(tq_qt);
	  }
	}	
      }
      if(mMethod == BRUTEFORCE){
	cv::BruteForceMatcher< cv::L2<float> > matcher;
	matcher.match(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatches);
	matcher.match(tFeature.getDescriptors(), qFeature.getDescriptors(), tqInitialMatches);
    }
      if(mMethod == FLANN){
	cv::FlannBasedMatcher matcher;
	matcher.match(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatches);
	matcher.match(tFeature.getDescriptors(), qFeature.getDescriptors(), tqInitialMatches);
      }
      if(mMethod == FLANN || mMethod == BRUTEFORCE)
	{
	  //add the intersection of both way matches
	  for(unsigned int i = 0; i < qtInitialMatches.size(); i++){
	    for(unsigned int j =0 ; j<tqInitialMatches.size(); j++){
	      if(qtInitialMatches[i].queryIdx == tqInitialMatches[j].trainIdx && qtInitialMatches[i].trainIdx == tqInitialMatches[j].queryIdx){
		matches.push_back(qtInitialMatches[i]);    
	      }
	    }
	  }
	}
    }
    //Matching descriptors using BruteFore with Hamming distance for ORB descriptor
    else if(qFeature.getDescriptorMethod() == ORB_DES){
      if(mMethod == KNN){
	cv::BruteForceMatcher< cv::Hamming > matcher;
	matcher.knnMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, knn);
	matcher.knnMatch(tFeature.getDescriptors(), qFeature.getDescriptors(), tqInitialMatchesVector, knn);
      }
      if(mMethod == RADIUS){
	cv::BruteForceMatcher< cv::Hamming > matcher;
	matcher.radiusMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, radius);
	matcher.radiusMatch(tFeature.getDescriptors(), qFeature.getDescriptors(), tqInitialMatchesVector, radius);
      }
      if(mMethod == KNN || mMethod == RADIUS){
	for(unsigned int i = 0; i < qtInitialMatchesVector.size(); i++){
	  for(unsigned int j = 0; j < qtInitialMatchesVector[i].size(); j++){
	    cv::DMatch forward = qtInitialMatchesVector[i][j];
	    for(unsigned int k = 0; k < tqInitialMatchesVector[forward.trainIdx].size(); k++){
	      cv::DMatch backward = tqInitialMatchesVector[forward.trainIdx][k];
	      if(backward.trainIdx == forward.queryIdx)
		matches.push_back(forward);
	    }
	    //matches.push_back(qtInitialMatchesVector[i][j]);
	  }
	}
      }
      if(mMethod == RATIO){
	cv::BruteForceMatcher< cv::Hamming > matcher;
	matcher.knnMatch(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatchesVector, 2);
	for(unsigned int i = 0; i < qtInitialMatchesVector.size(); i++){
	  float ratio = qtInitialMatchesVector[i][0].distance/qtInitialMatchesVector[i][1].distance;
	  if(ratio < 0.8)
	    matches.push_back(qtInitialMatchesVector[i][0]);
	}
	matcher.knnMatch(tFeature.getDescriptors(), qFeature.getDescriptors(), tqInitialMatchesVector, 2);
	for(unsigned int i = 0; i < tqInitialMatchesVector.size(); i++){
	  float ratio = tqInitialMatchesVector[i][0].distance/tqInitialMatchesVector[i][1].distance;
	  if(ratio < 0.8){
	    cv::DMatch tq_qt;
	    tq_qt.queryIdx = tqInitialMatchesVector[i][0].trainIdx;
	    tq_qt.trainIdx = tqInitialMatchesVector[i][0].queryIdx;
	    tq_qt.imgIdx = tqInitialMatchesVector[i][0].imgIdx;
	    tq_qt.distance = tqInitialMatchesVector[i][0].distance;
	    matches.push_back(tq_qt);
	  }
	}
      }
      if(mMethod == BRUTEFORCE){
	cv::BruteForceMatcher< cv::Hamming > matcher;
	matcher.match(qFeature.getDescriptors(), tFeature.getDescriptors(), qtInitialMatches);
	matcher.match(tFeature.getDescriptors(), qFeature.getDescriptors(), tqInitialMatches); 
	for(unsigned int i = 0; i < qtInitialMatches.size(); i++){
	  for(unsigned int j =0 ; j<tqInitialMatches.size(); j++){
	    if(qtInitialMatches[i].queryIdx == tqInitialMatches[j].trainIdx && qtInitialMatches[i].trainIdx == tqInitialMatches[j].queryIdx){
	      matches.push_back(qtInitialMatches[i]);    
	    }
	  }
	}
      }
    }
    
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
  }
  
  vector<cv::DMatch> feature_matcher::getMatches(){
    return matches;
  }

  matcher_method feature_matcher::getMatcherMethod(){
    return mMethod;
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
    cout<<"number of Matches: "<<nOfMatches<<", number of Matches after filteration: "<<nOfFilteredMatches<<", matching method: "<<matcherMethodToString(mMethod)<<", knn: "<<knn<<", radius: "<<radius<<"."<<endl;
    cout<<endl;
  }
}
