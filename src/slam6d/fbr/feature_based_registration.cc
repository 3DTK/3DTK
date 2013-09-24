/*
 * feature_based_registration implementation
 *
 * Copyright (C) HamidReza Houshiar
 *
 * Released under the GPL version 3.
 *
 */

#include <stdio.h>
#include <fstream>
#include "slam6d/fbr/fbr_global.h"
#include "slam6d/fbr/scan_cv.h"
#include "slam6d/fbr/panorama.h"
#include "slam6d/fbr/feature.h"
#include "slam6d/fbr/feature_matcher.h"
#include "slam6d/fbr/registration.h"
#include "slam6d/fbr/feature_drawer.h"

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

using namespace std;
using namespace fbr;

struct information{
  string local_time;
  string dir, outDir;
  int iWidth, iHeight, nImages, fScanNumber, sScanNumber, verbose;
  double minDistance, minError, minInlier;
  double pParam, mParam;
  IOType sFormat;
  projection_method pMethod;
  feature_detector_method fMethod;
  feature_descriptor_method dMethod;
  feature_filtration_method fFiltrationMethod;
  matcher_method mMethod;
  matching_filtration_method mFiltrationMethod;
  registration_method rMethod;
  bool scanServer;
  scanner_type sType;
  bool loadOct, saveOct;
  bool reflectance, color;
  int MIN_ANGLE, MAX_ANGLE;
  bool iSizeOptimization;
  
  int fSPoints, sSPoints, fFNum, sFNum, mNum, filteredMNum;
  double fSTime, sSTime, fPTime, sPTime, fFTime, sFTime, fDTime, sDTime, mTime, rTime; 
} info;

/**
 * usage : explains how to use the program CMD
 */
void usage(int argc, char** argv){
  printf("\n");
  printf("USAGE: %s dir -s firstScanNumber -e secondScanNumber \n", argv[0]);
  printf("\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\t\t-f scanFormat\t\t input scan file format [RIEGL_TXT|RXP|ALL SLAM6D SCAN_IO]\n");
  printf("\t\t-W iWidth\t\t panorama image width\n");
  printf("\t\t-H iHeight\t\t panorama image height\n");
  printf("\t\t-t sType \t\t Scannner Type [RIEGL | FARO]\n");
  printf("\t\t-n MIN_ANGLE \t\t Scanner vertical view MIN_ANGLE \n");
  printf("\t\t-x MAX_ANGLE \t\t Scanner vertical view MAX_ANGLE \n"); 
  printf("\n");
  printf("\n");
  printf("\t\t-p pMethod\t\t projection method [EQUIRECTANGULAR|CONIC|CYLINDRICAL|MERCATOR|RECTILINEAR|PANNINI|STEREOGRAPHIC|ZAXIS]\n");
  printf("\t\t-N nImages\t\t number of Horizontal images used for some projections\n");
  printf("\t\t-P pParam\t\t special projection parameter (d for Pannini and r for stereographic)\n");
  printf("\n");
  printf("\n");
  printf("\t\t-F fMethod\t\t feature detection method [SURF|SIFT|ORB|FAST|STAR]\n");
  printf("\t\t-a fFiltrationMethod\t feature filtration method [DISABLE_FILTER|STANDARD_DEVIATION|OCCLUSION]\n");
  printf("\t\t-d dMethod\t\t feature description method [SURF|SIFT|ORB]\n");
  printf("\t\t-m mMethod\t\t feature matching method [BRUTEFORCE|FLANN|KNN|RADIUS|RATIO]\n");
  printf("\t\t-M mParam \t\t special matching paameter (knn for KNN and r for radius)\n");
  printf("\t\t-A mFiltrationMethod\t feature matching filtrtion method [DISABLE_MATCHING_FILTER|FUNDEMENTAL_MATRIX]\n");
  printf("\n");
  printf("\n");
  printf("\t\t-D minDistance \t\t threshold for min distance in registration process\n");
  printf("\t\t-E minError \t\t threshold for min error in registration process\n");
  printf("\t\t-I minInlier \t\t threshold for min number of inliers in registration process\n");
  printf("\t\t-r registration \t registration method [ALL|ransac]\n");
  printf("\n");
  printf("\n");
  printf("\t\t-V verbose \t\t level of verboseness\n");
  printf("\t\t-O outDir \t\t output directory if not stated same as input\n");
  printf("\t\t-S scanServer \t\t Scan Server\n");
  printf("\t\t-l loadOct \t\t Load Octree\n");
  printf("\t\t-o saveOct \t\t Save Octree\n");
  printf("\t\t-R reflectance \t\t Use Reflectance\n");
  printf("\t\t-C color \t\t Use Color\n");
  printf("\t\t-i iSizeOptimization \t Optimize the panorama image size based on projection \n");
  printf("\n");
  printf("\n");
  printf("\tExamples:\n");
  printf("\tUsing Bremen City dataset:\n");
  printf("\tLoading scan000.txt and scan001.txt:\n");
  printf("\t\t %s ~/dir/to/bremen_city -s 0 -e 1\n", argv[0]);
  printf("\tLoading scan005.txt and scan006.txt and output panorma images and feature images and match images in ~/dir/to/bremen_city/out dir:\n");
  printf("\t\t %s -V 1 -O ~/dir/to/bremen_city/out/ ~/dir/to/bremen_city -s 5 -e 6 \n", argv[0]);
  printf("\tLoading scan010.txt and scan011.txt using Mercator projection and SURF feature detector and SIFT descriptor:\n");
  printf("\t\t %s -p MERCATOR -F SURF -d SIFT -O ~/dir/to/bremen_city/out/ ~/dir/to/bremen_city -s 10 -e 11 \n", argv[0]);
  printf("\n");
  exit(1);
}

void parssArgs(int argc, char** argv, information& info){
  time_t rawtime;
  struct tm *timeinfo;
  time(&rawtime);
  char time[50];
  timeinfo = localtime (&rawtime);
  sprintf(time, "%d-%d-%d-%d:%d:%d", timeinfo->tm_year + 1900, timeinfo->tm_mon + 1, timeinfo->tm_mday, timeinfo->tm_hour, timeinfo->tm_min, timeinfo->tm_sec);
  info.local_time = time;
  
  //default values
  info.iWidth = 3600;
  info.iHeight = 1000;
  info.nImages = 1;
  info.minDistance = 50;
  info.minError = 50;
  info.minInlier = 5;
  info.verbose = 0;
  //depend on the projection method
  info.pParam = 0;
  info.mParam = 0;
  //===============================
  info.sFormat = RIEGL_TXT;
  info.pMethod = EQUIRECTANGULAR;
  info.fMethod = SIFT_DET;
  info.fFiltrationMethod = DISABLE_FILTER;
  info.dMethod = SIFT_DES;
  info.mMethod = RATIO;
  info.mFiltrationMethod = DISABLE_MATCHING_FILTER;
  info.rMethod = RANSAC;
  info.outDir = "";
  info.scanServer = false;
  //=============================
  info.sType = stringToScannerType("RIEGL");
  info.loadOct = false;
  info.saveOct = false;
  info.reflectance = true;
  info.color = false;
  info.MIN_ANGLE = -40;
  info.MAX_ANGLE = 60;
  info.iSizeOptimization = false;

  int c;
  opterr = 0;
  //reade the command line and get the options

  while ((c = getopt (argc, argv, "a:A:Cd:D:e:E:f:F:H:iI:lm:M:n:N:oO:p:P:r:Rs:St:V:W:x:")) != -1)
    switch (c)
      {
      case 's':
	info.fScanNumber = atoi(optarg);
	break;
      case 'e':
	info.sScanNumber = atoi(optarg);
	break;
      case 'f':
	info.sFormat = stringToScanFormat(optarg);
	break;
      case 'W':
	info.iWidth = atoi(optarg);
	break;
      case 'H':
        info.iHeight = atoi(optarg);
	break;
      case 'p':
	info.pMethod = stringToProjectionMethod(optarg);
	break;
      case 'N':
	info.nImages = atoi(optarg);
	break;
      case 'P':
	info.pParam = atof(optarg);
	break;
      case 'F':
	info.fMethod = stringToFeatureDetectorMethod(optarg);
	break;
      case 'd':
	info.dMethod = stringToFeatureDescriptorMethod(optarg);
	break;
      case 'm':
	info.mMethod = stringToMatcherMethod(optarg);
	break;
      case 'D':
	info.minDistance = atof(optarg);
	break;
      case 'E':
	info.minError = atof(optarg);
	break;
      case 'I':
	info.minInlier = atof(optarg);
	break;
      case 'M':
	info.mParam = atof(optarg);
	break;
      case 'r':
	info.rMethod = stringToRegistrationMethod(optarg);
	break;
      case 'V':
	info.verbose = atoi(optarg);
	break;
      case 'O':
	info.outDir = optarg;
	break;
      case 'S':
	info.scanServer = true;
	break;
      case 't':
	info.sType = stringToScannerType(optarg);
	break;
      case 'l':
	info.loadOct = true;
	break;
      case 'o':
	info.saveOct = true;
	break;
      case 'R':
	info.reflectance = true;
	break;
      case 'C':
	info.color = true;
	break;
      case 'n':
	info.MIN_ANGLE = atoi(optarg);
	break;
      case 'x':
	info.MAX_ANGLE = atoi(optarg);
	break;
      case 'i':
	info.iSizeOptimization = true;
	break;
      case 'a':
	info.fFiltrationMethod = stringToFeatureFiltrationMethod(optarg);
	break;
      case 'A':
	info.mFiltrationMethod = stringToMatchingFiltrationMethod(optarg);
	break;


      case '?':
	cout<<"Unknown option character "<<optopt<<endl;
	usage(argc, argv);
	break;
      default:
	usage(argc, argv);
      }
  if(info.pMethod == PANNINI && info.pParam == 0){
    info.pParam = 1;
    if(info.nImages < 2) info.nImages = 2;
  }
  if(info.pMethod == STEREOGRAPHIC && info.pParam == 0){
    info.pParam = 2;
    if(info.nImages < 2) info.nImages = 2;
  }
  if(info.pMethod == RECTILINEAR && info.nImages < 3)
    info.nImages = 3;
  if(info.mMethod == KNN && info.mParam == 0)
    info.mParam = 3;
  if(info.mMethod == RADIUS && info.mParam == 0)
    info.mParam = 100;
  if(info.dMethod == ORB_DES && info.fMethod == SIFT_DET){
    cout<<"Error: SIFT feature doesn't work with ORB descriptor."<<endl;
    usage(argc, argv);
  }
  if(info.mMethod == FLANN && info.dMethod == ORB_DES){
    cout<<"Error: ORB descriptoronly works with BRUTEFORCE matcher."<<endl;
    usage(argc, argv);
  }

  if (optind > argc - 1)
    {
      cout<<"Too few input arguments. At least dir and two scan numbers are required."<<endl;
      usage(argc, argv);
    }
  
  info.dir = argv[optind];
  //info.fScanNumber = atoi(argv[optind+1]);
  //info.sScanNumber = atoi(argv[optind+2]);
  if(info.outDir.empty()) info.outDir = info.dir;
  else if(info.outDir.compare(info.outDir.size()-1, 1, "/") != 0) info.outDir += "/";
  cout<<info.fScanNumber<<endl;
  cout<<info.sScanNumber<<endl;
}

void informationDescription(information info){
  cout<<"program parameters are:"<<endl;
  cout<<endl;
  cout<<"local time: "<<info.local_time<<endl;
  cout<<"input dir: "<<info.dir<<endl;
  cout<<"output dir: "<<info.outDir<<endl;
  cout<<"first scan number: "<<info.fScanNumber<<endl;
  cout<<"second scan number: "<<info.sScanNumber<<endl;
  cout<<"scan format: "<<scanFormatToString(info.sFormat)<<endl;
  cout<<"scanner type:"<<scannerTypeToString(info.sType)<<endl;
  cout<<"reflectance: "<<info.reflectance<<endl;
  cout<<"color: "<<info.color<<endl;
  cout<<endl;
  cout<<"image width: "<<info.iWidth<<endl;
  cout<<"image height: "<<info.iHeight<<endl;
  cout<<"number of images: "<<info.nImages<<endl;
  cout<<"projection parameter: "<<info.pParam<<endl;
  cout<<"projection method: "<<projectionMethodToString(info.pMethod)<<endl;
  cout<<"Scanner Verticla field of view: "<<info.MIN_ANGLE<<"<-->"<<info.MAX_ANGLE<<endl;
  cout<<endl;
  cout<<"feature detector method: "<<featureDetectorMethodToString(info.fMethod)<<endl;
  cout<<"feature filtration method: "<<featureFiltrationMethodToString(info.fFiltrationMethod)<<endl;
  cout<<"feature descriptor method: "<<featureDescriptorMethodToString(info.dMethod)<<endl;
  cout<<endl;
  cout<<"matcher parameter: "<<info.mParam<<endl;
  cout<<"matcher method: "<<matcherMethodToString(info.mMethod)<<endl;
  cout<<"matching filtration method: "<<matchingFiltrationMethodToString(info.mFiltrationMethod)<<endl;
  cout<<endl;
  cout<<"min distacne: "<<info.minDistance<<endl;
  cout<<"min error: "<<info.minError<<endl;
  cout<<"min inlier: "<<info.minInlier<<endl;
  cout<<"registration method: "<<registrationMethodToString(info.rMethod)<<endl;
  cout<<endl;
}

void info_yml(information info, double bError, double bErrorIdx, double* bAlign){
  cv::Mat align(16, 1, CV_32FC(1), cv::Scalar::all(0));
  for(int i = 0 ; i < 16 ; i++)
    align.at<float>(i,0) = bAlign[i];
  
  string yml;
  yml = info.outDir+"fbr-yml.yml";
  cv::FileStorage fs(yml.c_str(), cv::FileStorage::APPEND);
  fs << "feature_bas_registration" << "{";
  
  fs << "pair" << "{" << "scan" << to_string(info.fScanNumber, 3);
  fs << "scan" << to_string(info.sScanNumber, 3) << "}";
  
  fs << "time" << "{" << "local_time" << info.local_time << "}";
  
  fs << "param" << "{";
  fs << "DIR" << info.dir;
  fs << "sFormat" << scanFormatToString(info.sFormat);
  fs << "pMethod" << projectionMethodToString(info.pMethod);
  fs << "nImages" << info.nImages;
  fs << "pParam" << info.pParam;
  fs << "iWidth" << info.iWidth;
  fs << "iHeight" << info.iHeight;
  fs << "fMethod" << featureDetectorMethodToString(info.fMethod);
  fs << "dMethod" << featureDescriptorMethodToString(info.dMethod);
  fs << "mMethod" << matcherMethodToString(info.mMethod);
  fs << "mParam" << info.mParam;
  fs << "rMethod" << registrationMethodToString(info.rMethod);
  fs << "minDistance" << info.minDistance;
  fs << "minInlier" << info.minInlier;
  fs << "minError" << info.minError;
  fs << "}";

  fs << "input" << "{";
  fs << "first_input" << "{";
  fs << "name" << "{" << "scan" << to_string(info.fScanNumber, 3) << "}";
  fs << "point" << "{" << "amount" << info.fSPoints << "time" << info.fSTime << "}";
  fs << "projection" << "{" << "time" << info.fPTime << "}";
  fs << "feature" << "{" << "amount" << info.fFNum << "fTime" << info.fFTime << "dTime" << info.fDTime << "}";
  fs << "}";
  fs << "second_input" << "{";
  fs << "name" << "{" << "scan" << to_string(info.sScanNumber, 3) << "}";
  fs << "point" << "{" << "amount" << info.sSPoints << "time" << info.sSTime << "}";
  fs << "projection" << "{" << "time" << info.sPTime << "}";
  fs << "feature" << "{" << "amount" << info.sFNum << "fTime" << info.sFTime << "dTime" << info.sDTime << "}";
  fs << "}";
  fs << "}";
  
  fs << "matches" << "{";
  fs << "amount" << info.mNum << "filteration" << info.filteredMNum << "time" << info.mTime << "}";
  
  fs << "reg" << "{";
  fs << "bestError" << bError << "bestErrorIdx" << bErrorIdx << "time" << info.rTime << "bAlign" << align << "}";

  fs << "}";
}

int main(int argc, char** argv){
  cout<<CV_VERSION<<endl;
  string out;
  cv::Mat outImage;
  feature_drawer drawer;
  parssArgs(argc, argv, info);
  if(info.verbose >= 1) informationDescription(info);
  
  //get first scan
  scan_cv fScan (info.dir, info.fScanNumber, info.sFormat, info.scanServer, info.sType, info.loadOct, info.saveOct, info.reflectance, info.color);
  if(info.verbose >= 4) info.fSTime = (double)cv::getTickCount();
  fScan.convertScanToMat();
  if(info.verbose >= 4) info.fSTime = ((double)cv::getTickCount() - info.fSTime)/cv::getTickFrequency();
  if(info.verbose >= 2) fScan.getDescription();

  //create first panorama
  panorama fPanorama (info.iWidth, info.iHeight, info.pMethod, info.nImages, info.pParam, FARTHEST, fScan.getZMin(), fScan.getZMax(), info.MIN_ANGLE, info.MAX_ANGLE, info.iSizeOptimization);
  if(info.verbose >= 4) info.fPTime = (double)cv::getTickCount();
  if((fScan.getMatScanColor()).empty() == 1)
    fPanorama.createPanorama(fScan.getMatScan());
  else
    fPanorama.createPanorama(fScan.getMatScan(), fScan.getMatScanColor());
  
  //get the new panorama size incase of optimized size panorama
  //info.iWidth = fPanorama.getImageWidth();
  //info.iHeight = fPanorama.getImageHeight();

  if(info.verbose >= 4) info.fPTime = ((double)cv::getTickCount() - info.fPTime)/cv::getTickFrequency();
  if(info.verbose >= 2) fPanorama.getDescription();

  //write first panorama to image
  if(info.verbose >= 1){
    out = info.outDir+info.local_time+"_scan"+to_string(info.fScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(fPanorama.getImageWidth())+"x"+to_string(fPanorama.getImageHeight())+".png";
    imwrite(out, fPanorama.getReflectanceImage());
    out = info.outDir+info.local_time+"_scan"+to_string(info.fScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(fPanorama.getImageWidth())+"x"+to_string(fPanorama.getImageHeight())+"_Range.png";
    imwrite(out, fPanorama.getRangeImage());
    if((fPanorama.getColorImage()).empty() != 1){
      out = info.outDir+info.local_time+"_scan"+to_string(info.fScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(fPanorama.getImageWidth())+"x"+to_string(fPanorama.getImageHeight())+"_color.png";
      imwrite(out, fPanorama.getColorImage());
    }
  }

  //create first feature set
  //feature fFeature;
  feature fFeature(info.fMethod, info.dMethod, info.fFiltrationMethod);
  if(info.verbose >= 4) info.fFTime = (double)cv::getTickCount();
  //fFeature.featureDetection(fPanorama.getReflectanceImage(), info.fMethod); 
  fFeature.featureDetection(fPanorama.getReflectanceImage(), info.fMethod, fPanorama.getRangeImage(), info.fFiltrationMethod); 
  if(info.verbose >= 4) info.fFTime = ((double)cv::getTickCount() - info.fFTime)/cv::getTickFrequency();

  //write panorama with keypoints to image
  if(info.verbose >= 1){
    drawer.DrawKeypoints(fPanorama.getReflectanceImage(), fFeature.getFeatures(), outImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    out = info.outDir+info.local_time+"_scan"+to_string(info.fScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(fPanorama.getImageWidth())+"x"+to_string(fPanorama.getImageHeight())+"_"+featureDetectorMethodToString(info.fMethod)+".png";
    imwrite(out, outImage);
    outImage.release();
  }
  
  //create first descriptor set
  if(info.verbose >= 4) info.fDTime = (double)cv::getTickCount();
  //fFeature.featureDescription(fPanorama.getReflectanceImage(), info.dMethod);
  fFeature.featureDescription(fPanorama.getReflectanceImage());
  if(info.verbose >= 4) info.fDTime = ((double)cv::getTickCount() - info.fDTime)/cv::getTickFrequency();
  if(info.verbose >= 2) fFeature.getDescription();
  
  //get secon scan
  scan_cv sScan (info.dir, info.sScanNumber, info.sFormat, info.scanServer, info.sType, info.loadOct, info.saveOct, info.reflectance, info.color);
  if(info.verbose >= 4) info.sSTime = (double)cv::getTickCount();
  sScan.convertScanToMat();
  if(info.verbose >= 4) info.sSTime = ((double)cv::getTickCount() - info.sSTime)/cv::getTickFrequency();
  if(info.verbose >= 2) sScan.getDescription();

  //create secon panoram
  panorama sPanorama (info.iWidth, info.iHeight, info.pMethod, info.nImages, info.pParam, FARTHEST, sScan.getZMin(), sScan.getZMax(), info.MIN_ANGLE, info.MAX_ANGLE, info.iSizeOptimization);
  if(info.verbose >= 4) info.sPTime = (double)cv::getTickCount();
  if((sScan.getMatScanColor()).empty() == 1)
    sPanorama.createPanorama(sScan.getMatScan());
  else
    sPanorama.createPanorama(sScan.getMatScan(), sScan.getMatScanColor());

  //get the new panorama size incase of optimized size panorama
  //info.iWidth = sPanorama.getImageWidth();
  //info.iHeight = sPanorama.getImageHeight();

  if(info.verbose >= 4) info.sPTime = ((double)cv::getTickCount() - info.sPTime)/cv::getTickFrequency();
  if(info.verbose >= 2) sPanorama.getDescription();

  //write panorama to image
  if(info.verbose >= 1){
    out = info.outDir+info.local_time+"_scan"+to_string(info.sScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(fPanorama.getImageWidth())+"x"+to_string(fPanorama.getImageHeight())+".png";
    imwrite(out, sPanorama.getReflectanceImage());
    out = info.outDir+info.local_time+"_scan"+to_string(info.sScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(fPanorama.getImageWidth())+"x"+to_string(fPanorama.getImageHeight())+"_Range.png";
    imwrite(out, sPanorama.getRangeImage());
    if((sPanorama.getColorImage()).empty() != 1){
      out = info.outDir+info.local_time+"_scan"+to_string(info.sScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(fPanorama.getImageWidth())+"x"+to_string(fPanorama.getImageHeight())+"_color.png";
      imwrite(out, sPanorama.getColorImage());
    }
  }

  //create secon feature set
  //feature sFeature;
  feature sFeature(info.fMethod, info.dMethod, info.fFiltrationMethod);
  if(info.verbose >= 4) info.sFTime = (double)cv::getTickCount();
  //sFeature.featureDetection(sPanorama.getReflectanceImage(), info.fMethod);
  sFeature.featureDetection(sPanorama.getReflectanceImage(), info.fMethod, sPanorama.getRangeImage(), info.fFiltrationMethod); 
  if(info.verbose >= 4) info.sFTime = ((double)cv::getTickCount() - info.sFTime)/cv::getTickFrequency();

  //write panorama with keypoints to image
  if(info.verbose >= 1){
    drawer.DrawKeypoints(sPanorama.getReflectanceImage(), sFeature.getFeatures(), outImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
    out = info.outDir+info.local_time+"_scan"+to_string(info.sScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(fPanorama.getImageWidth())+"x"+to_string(fPanorama.getImageHeight())+"_"+featureDetectorMethodToString(info.fMethod)+".png";
    imwrite(out, outImage);
    outImage.release();
  }

  //create secon descriptor set
  if(info.verbose >= 4) info.sDTime = (double)cv::getTickCount();
  //sFeature.featureDescription(sPanorama.getReflectanceImage(), info.dMethod);
  sFeature.featureDescription(sPanorama.getReflectanceImage());
  if(info.verbose >= 4) info.sDTime = ((double)cv::getTickCount() - info.sDTime)/cv::getTickFrequency();
  if(info.verbose >= 2) sFeature.getDescription();


  //get the new panorama size incase of optimized size panorama
  info.iWidth = sPanorama.getImageWidth();
  info.iHeight = sPanorama.getImageHeight();

  //match features
  feature_matcher matcher (info.mMethod, info.mParam, info.mFiltrationMethod);
  if(info.verbose >= 4) info.mTime = (double)cv::getTickCount();
  matcher.match(fFeature, sFeature);
  if(info.verbose >= 4) info.mTime = ((double)cv::getTickCount() - info.mTime)/cv::getTickFrequency();
  if(info.verbose >= 2) matcher.getDescription();

  //write matcheed feature to image
  if(info.verbose >= 1){
    drawer.DrawMatches(fPanorama.getReflectanceImage(), fFeature.getFeatures(), sPanorama.getReflectanceImage(), sFeature.getFeatures(), matcher.getMatches(), outImage, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
    out = info.outDir+info.local_time+"_scan"+to_string(info.fScanNumber, 3)+"_scan"+to_string(info.sScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(info.iWidth)+"x"+to_string(info.iHeight)+"_"+featureDetectorMethodToString(info.fMethod)+"_"+featureDescriptorMethodToString(info.dMethod)+"_"+matcherMethodToString(info.mMethod)+".png";
    imwrite(out, outImage);
    outImage.release();
  }

  //start the regisration
  registration reg (info.minDistance, info.minError, info.minInlier, info.rMethod);
  if(info.verbose >= 4) info.rTime = (double)cv::getTickCount();
  reg.findRegistration(fPanorama.getMap(), fFeature.getFeatures(), sPanorama.getMap(), sFeature.getFeatures(), matcher.getMatches());
  if(info.verbose >= 4) info.rTime = ((double)cv::getTickCount() - info.rTime)/cv::getTickFrequency();
  if(info.verbose >= 2) reg.getDescription();

  //write .dat and .frames files
  if(info.verbose >= 0){
    double *bAlign = reg.getBestAlign();
    
    out = info.outDir+info.local_time+"_scan"+to_string(info.fScanNumber, 3)+"_scan"+to_string(info.sScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(info.iWidth)+"x"+to_string(info.iHeight)+"_"+featureDetectorMethodToString(info.fMethod)+"_"+featureDescriptorMethodToString(info.dMethod)+"_"+matcherMethodToString(info.mMethod)+"_"+registrationMethodToString(info.rMethod)+".dat";
    ofstream dat(out.c_str());
    dat << bAlign[0] << " " << bAlign[4] << " " << bAlign[8] << " " << bAlign[12] <<endl;
    dat << bAlign[1] << " " << bAlign[5] << " " << bAlign[9] << " " << bAlign[13] <<endl;
    dat << bAlign[2] << " " << bAlign[6] << " " << bAlign[10] << " " << bAlign[14] <<endl;
    dat << bAlign[3] << " " << bAlign[7] << " " << bAlign[11] << " " << bAlign[15] <<endl;
    dat.close();
    
    out = info.outDir+info.local_time+"_scan"+to_string(info.fScanNumber, 3)+"_scan"+to_string(info.sScanNumber, 3)+"_"+projectionMethodToString(info.pMethod)+"_"+to_string(info.iWidth)+"x"+to_string(info.iHeight)+"_"+featureDetectorMethodToString(info.fMethod)+"_"+featureDescriptorMethodToString(info.dMethod)+"_"+matcherMethodToString(info.mMethod)+"_"+registrationMethodToString(info.rMethod)+".frames";
    ofstream frames(out.c_str());
    for (int n = 0 ; n < 2 ; n++)
      {
	for(int i = 0; i < 16; i++)
	  frames << bAlign[i] <<" ";
	frames << "2" << endl;
      }
    frames.close();
  }
  
  if(info.verbose >= 3){
    info.fSPoints = fScan.getNumberOfPoints();
    info.sSPoints = sScan.getNumberOfPoints();
    info.fFNum = fFeature.getNumberOfFeatures();
    info.sFNum = sFeature.getNumberOfFeatures();
    info.mNum = matcher.getNumberOfMatches();
    info.filteredMNum = matcher.getNumberOfFilteredMatches();
    
    info_yml(info, reg.getBestError(), reg.getBestErrorIndex(), reg.getBestAlign());
  }
  
  return 0;
}
