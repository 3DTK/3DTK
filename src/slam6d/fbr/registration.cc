/*
 * registration implementation
 *
 * Copyright (C) HamidReza Houshiar
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/fbr/registration.h"

using namespace std;

namespace fbr{

  registration::registration(double minD, double minE, unsigned int minI, registration_method method){
    minDistance = minD;
    minError = minE;
    minInlier = minI;
    rMethod = method;
    bestError = minError;
    bestErrorIndex = 0;
    for(int i = 0; i < 16; i++)
      bestAlign[i] = 0;
  }
  
  registration::registration(){
    minDistance = 50;
    minError = 50;
    minInlier = 5;
    rMethod = RANSAC;
    bestError = minError;
    bestErrorIndex = 0;
    for(int i = 0; i < 16; i++)
      bestAlign[i] = 0;
  }
  
  int registration::getCoord(vector<cv::KeyPoint> fKeypoints, vector<cv::KeyPoint> sKeypoints, vector<cv::DMatch> matches, cv::Mat fPMap, cv::Mat sPMap, int idx, cv::Point3f& cq, cv::Point3f& ct){
    int x, y;
    y = fKeypoints[matches[idx].queryIdx].pt.x;
    x = fKeypoints[matches[idx].queryIdx].pt.y;

    if(fKeypoints[matches[idx].queryIdx].pt.x - x > 0.5)
      x++;
    if(fKeypoints[matches[idx].queryIdx].pt.y - y > 0.5)
      y++;
    cv::Mat_<cv::Vec3f> _fPMap = fPMap;
    float sqr = sqrt (_fPMap(x,y)[0] * _fPMap(x,y)[0] + _fPMap(x,y)[1] * _fPMap(x,y)[1] + _fPMap(x,y)[2] * _fPMap(x,y)[2]);
    if(sqr != 0){
      cq.x = _fPMap(x,y)[0];
      cq.y = _fPMap(x,y)[1];
      cq.z = _fPMap(x,y)[2]; 
    }
    else
      return 0;
   
    y = sKeypoints[matches[idx].trainIdx].pt.x;
    x = sKeypoints[matches[idx].trainIdx].pt.y;
    if(sKeypoints[matches[idx].trainIdx].pt.x - x > 0.5)
      x++;
    if(sKeypoints[matches[idx].trainIdx].pt.y - y > 0.5)
      y++;
    cv::Mat_<cv::Vec3f> _sPMap = sPMap;
    sqr = sqrt (_sPMap(x,y)[0] * _sPMap(x,y)[0] + _sPMap(x,y)[1] * _sPMap(x,y)[1] + _sPMap(x,y)[2] * _sPMap(x,y)[2]);
    if(sqr != 0){
      ct.x = _sPMap(x,y)[0];
      ct.y = _sPMap(x,y)[1];
      ct.z = _sPMap(x,y)[2];
    }
    else
      return 0;
  
    return 1;
  }

  void registration::pointToArray(cv::Point3f c, double* cd){
    cd[0] = c.x;
    cd[1] = c.y;
    cd[2] = c.z;
  }

  cv::Point3f registration::coordTransform(cv::Point3f p, double* align){
    cv::Point3f tp;
    tp.x = align[0]*p.x + align[4]*p.y + align[8]*p.z + align[12];
    tp.y = align[1]*p.x + align[5]*p.y + align[9]*p.z + align[13];
    tp.z = align[2]*p.x + align[6]*p.y + align[10]*p.z + align[14];
    return tp;
  }


  int registration::findAlign(unsigned int i, unsigned int j, unsigned int k, cv::Mat fPMap, vector<cv::KeyPoint> fKeypoints, cv::Mat sPMap, vector<cv::KeyPoint> sKeypoints, vector<cv::DMatch> matches){
    cv::Point3f c1q, c2q, c3q, c1t, c2t, c3t;
    if(i == j || i == k || j == k)
      return 0;
    //get the coordinates
    if(getCoord(fKeypoints, sKeypoints, matches, fPMap, sPMap, i, c1q, c1t) == 0)
      return 0;
    if(getCoord(fKeypoints, sKeypoints, matches, fPMap, sPMap, j, c2q, c2t) == 0)
      return 0;
    if(getCoord(fKeypoints, sKeypoints, matches, fPMap, sPMap, k, c3q, c3t) == 0)
      return 0;
    //check for min distance
    if(norm(c1q - c2q) < minDistance || norm(c1q - c3q) < minDistance || norm(c2q - c3q) < minDistance || norm(c1t - c2t) < minDistance || norm(c1t - c3t) < minDistance || norm(c2t - c3t) < minDistance)
      return 0;
    //calculate the centroids
    cv::Point3f centroidq, centroidt;
    centroidq = (c1q + c2q + c3q);
    centroidt = (c1t + c2t + c3t);
    centroidq.x = centroidq.x / 3;
    centroidq.y = centroidq.y / 3;
    centroidq.z = centroidq.z / 3;
    centroidt.x = centroidt.x / 3;
    centroidt.y = centroidt.y / 3;
    centroidt.z = centroidt.z / 3;
    //put each point into double array
    double c1qd[3], c2qd[3], c3qd[3], c1td[3], c2td[3], c3td[3];
    cv::Point3f temp;
    pointToArray(c1q, c1qd);
    pointToArray(c2q, c2qd);
    pointToArray(c3q, c3qd);
    pointToArray(c1t, c1td);
    pointToArray(c2t, c2td);
    pointToArray(c3t, c3td);
    //create PtPair and calc the align with icp6D_QUAT
    vector<PtPair> pairs;
    pairs.push_back(PtPair(c1qd, c1td));
    pairs.push_back(PtPair(c2qd, c2td));
    pairs.push_back(PtPair(c3qd, c3td));
    double align[16];
    double centroidqd[3], centroidtd[3];
    pointToArray(centroidq, centroidqd);
    pointToArray(centroidt, centroidtd);
    icp6D_QUAT q(true);
    q.Align(pairs, align, centroidqd, centroidtd);
    //transform the matches with align if the error is less than minerror
    double iError = 0;
    unsigned int eIdx = 0;
    for(unsigned int p = 0; p < matches.size(); p++){
      if(p == i || p == j || p == k)
	continue;
      cv::Point3f cq, ct, ct_trans;
      if(getCoord(fKeypoints, sKeypoints, matches, fPMap, sPMap, p, cq, ct) == 0)
	continue;
      ct_trans = coordTransform(ct, align);
      if(norm(ct_trans - cq) < minError){
	iError += norm(ct_trans - cq);
	eIdx++;
      }
    }
    //check for mininlier and find the best align
    if(eIdx > minInlier){ 
      double aError = iError / eIdx;
      if(aError - iInfluence*eIdx < bestError - iInfluence*bestErrorIndex){
	bestError = aError;
	bestErrorIndex = eIdx;
	for(int a = 0; a < 16; a++) 
	  bestAlign[a] = align[a];
      }
    }
    return 1;
  }
  
  void registration::findRegistration(cv::Mat fPMap, vector<cv::KeyPoint> fKeypoints, cv::Mat sPMap, vector<cv::KeyPoint> sKeypoints, vector<cv::DMatch> matches){
    //go through all matches
    if(rMethod == ALL){
      for(unsigned int i = 0; i < matches.size(); i++)
	for(unsigned int j = 0; j < matches.size(); j++)
	  for(unsigned int k = 0; k < matches.size(); k++){
	    findAlign(i, j, k, fPMap, fKeypoints, sPMap, sKeypoints, matches);
	  }
    }
    //RANSAC
    if(rMethod == RANSAC)
      for(int r = 0; r < RANSACITR; r++){
	if((r % (RANSACITR/10)) == 0){
	  cout<<"RANSAC iteration: "<<(r / (RANSACITR/10) + 1) * 10 <<"%"<<endl;
	}
	unsigned int i = rand() % matches.size();
	unsigned int j = rand() % matches.size();
	unsigned int k = rand() % matches.size();
	findAlign(i, j, k, fPMap, fKeypoints, sPMap, sKeypoints, matches);
      }
  }

  double registration::getMinDistance(){
    return minDistance;
  }

  double registration::getMinError(){
    return minError;
  }

  unsigned int registration::getMinInlier(){
    return minInlier;
  }

  registration_method registration::getRegistrationMethod(){
    return rMethod;
  }

  double * registration::getBestAlign(){
    return bestAlign;
  }

  double registration::getBestError(){
    return bestError;
  }

  unsigned int registration::getBestErrorIndex(){
    return bestErrorIndex;
  }
  
  void registration::getDescription(){
    cout<<"Registration minDistance: "<<minDistance<<", minError: "<<minError<<", minInlier: "<<minInlier<<", registrationMethod: "<<registrationMethodToString(rMethod)<<"."<<endl;
    cout<<"Registration finished with besterror of: "<<bestError<<" and best error index of: "<<bestErrorIndex<<"."<<endl;
    cout<<"align Matrix:"<<endl;
    if(bestErrorIndex > 0){
      cout<<bestAlign[0]<<"  "<<bestAlign[4]<<"  "<<bestAlign[8]<<"  "<<bestAlign[12]<<endl;
      cout<<bestAlign[1]<<"  "<<bestAlign[5]<<"  "<<bestAlign[9]<<"  "<<bestAlign[13]<<endl;
      cout<<bestAlign[2]<<"  "<<bestAlign[6]<<"  "<<bestAlign[10]<<"  "<<bestAlign[14]<<endl;
      cout<<bestAlign[3]<<"  "<<bestAlign[7]<<"  "<<bestAlign[11]<<"  "<<bestAlign[15]<<endl;
    }
    cout<<endl; 
  }
}
