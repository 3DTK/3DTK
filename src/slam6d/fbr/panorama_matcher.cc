#include <stdio.h>
#include <fstream>
#include "slam6d/fbr/fbr_global.h"
#include "slam6d/fbr/scan_cv.h"
#include "slam6d/fbr/panorama.h"
#include "slam6d/fbr/feature.h"
#include "slam6d/fbr/feature_matcher.h"
#include "slam6d/fbr/registration.h"
#include "slam6d/fbr/feature_drawer.h"

using namespace std;
using namespace fbr;

struct information{
  string dir, outDir;
  int start, end;
  feature_detector_method fMethod;
  feature_descriptor_method dMethod;
  matcher_method mMethod;
} info;

void usage(int argc, char** argv){
  printf("\n");
  printf("USAGE: %s dir -s start -e end \n", argv[0]);
  printf("\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\t\t-F fMethod\t\t feature detection method [SURF|SIFT|ORB|FAST|STAR]\n");
  printf("\t\t-D dMethod\t\t feature description method [SURF|SIFT|ORB]\n");
  printf("\t\t-M mMethod\t\t feature matching method [BRUTEFORCE|FLANN|KNN|RADIUS|RATIO]\n");
  printf("\t\t-O outDir \t\t output directory if not stated same as input\n");
  printf("\n");
  exit(1);
}

void parssArgs(int argc, char** argv, information& info){

  //default values
  info.fMethod = SIFT_DET;
  info.dMethod = SIFT_DES;
  info.mMethod = RATIO;
  info.outDir = "";

  int c;
  opterr = 0;
  while((c = getopt (argc, argv, "s:e:F:D:M:O:")) != -1)
    switch (c)
      {
      case 's':
	info.start = atoi(optarg);
	break;
      case 'e':
	info.end = atoi(optarg);
	break;
      case 'F':
	info.fMethod = stringToFeatureDetectorMethod(optarg);
	break;
      case 'D':
	info.dMethod = stringToFeatureDescriptorMethod(optarg);
	break;
      case 'M':
	info.mMethod = stringToMatcherMethod(optarg);
	break;
      case 'O':
	info.outDir = optarg;
	break;
      case '?':
	cout<<"Unknown option character "<<optopt<<endl;
	usage(argc, argv);
	break;
      default:
	usage(argc, argv);
      }

  if (optind > argc - 1)
    {
      cout<<"Too few input arguments. At least dir and two scan numbers are required."<<endl;
      usage(argc, argv);
    }

  info.dir = argv[optind];
  if(info.outDir.empty()) info.outDir = info.dir;
  else if(info.outDir.compare(info.outDir.size()-1, 1, "/") != 0) info.outDir += "/";
}

int main(int argc, char** argv){
  parssArgs(argc, argv, info);

  cout<<info.start<<endl;
  cout<<info.end<<endl;
  
  cv::Mat panorama1, panorama2;
  panorama1.create(3600,1000,CV_8U);
  panorama2.create(3600,1000,CV_8U);
  panorama1 = cv::Scalar::all(0);
  panorama2 = cv::Scalar::all(0);

  panorama1 = cv::imread(info.dir+"scan"+to_string(info.start, 3)+".png", CV_LOAD_IMAGE_GRAYSCALE );
  panorama2 = cv::imread(info.dir+"scan"+to_string(info.end, 3)+".png", CV_LOAD_IMAGE_GRAYSCALE );

  feature feature1, feature2;
  
  feature1.featureDetection(panorama1, info.fMethod);
  feature2.featureDetection(panorama2, info.fMethod);

  cv::Mat outImage;
  outImage.create(3600,1000,CV_8U);
  outImage = cv::Scalar::all(0);

  feature_drawer drawer;
  //cv::drawKeypoints(panorama1, feature1.getFeatures(), outImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  drawer.DrawKeypoints(panorama1, feature1.getFeatures(), outImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  imwrite(info.outDir+"feature"+"-"+featureDetectorMethodToString(info.fMethod)+"-"+to_string(info.start, 3)+".png", outImage);
  outImage.release();

  //cv::drawKeypoints(panorama2, feature2.getFeatures(), outImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  drawer.DrawKeypoints(panorama2, feature2.getFeatures(), outImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
  imwrite(info.outDir+"feature"+"-"+featureDetectorMethodToString(info.fMethod)+"-"+to_string(info.end, 3)+".png", outImage);
  outImage.release();

  feature1.featureDescription(panorama1, info.dMethod);
  feature2.featureDescription(panorama1, info.dMethod);


  feature_matcher matcher (info.mMethod, 0);
  matcher.match(feature1, feature2);
  //cv::drawMatches(panorama1, feature1.getFeatures(), panorama2, feature2.getFeatures(), matcher.getMatches(), outImage, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  drawer.DrawMatches(panorama1, feature1.getFeatures(), panorama2, feature2.getFeatures(), matcher.getMatches(), outImage, cv::Scalar::all(-1), cv::Scalar::all(-1), vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
  imwrite(info.outDir+"match"+"-"+featureDetectorMethodToString(info.fMethod)+"-"+to_string(info.start, 3)+"_"+to_string(info.end, 3)+".png", outImage);
  outImage.release();
}
