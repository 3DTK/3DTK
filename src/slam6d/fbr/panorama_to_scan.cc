/*
 * panorama_to_scan implementation
 *
 * Copyright (C) Hamidreza Houshiar
 *
 * Released under the GPL version 3.
 *
 */

#include <stdio.h>
#include <fstream>
#include "slam6d/fbr/fbr_global.h"
#include "slam6d/fbr/panorama.h"
#include "slam6d/fbr/projection.h"
#include <unordered_map>

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

using namespace std;
using namespace fbr;

struct information{
  panorama_type inputImageType;
  string inputThreeChannel24BitRangeImage;
  string inputThreeGrayscaleRangeImage1;
  string inputThreeGrayscaleRangeImage2;
  string inputThreeGrayscaleRangeImage3;
  string inputRangeImage;
  string inputReflectanceImage;
  string inputColorImage;
  int imageWidth, imageHeight;
  projection_method projectionMethod;
  int minVertAngle, maxVertAngle;
  int minHorizAngle, maxHorizAngle;
  recovered_range_filteration_method recoveredRangeFilterationMethod;
  int numberOfNeighbours;
  float interquartileScaleFactor;
  float averageDiff;
  string outputDir;
} info;

void usage(int argc, char** argv)
{
  printf("\n");
  printf("USAGE: %s input images -t the input image type -O output dir\n", argv[0]);
  printf("\n");
  printf("\n");
  printf("\t-t the input images type\t\t [ThreeChannel24BitRange|ThreeGrayscaleRange]\n");
  printf("\t-O output dir\t\t\t\t output Dir\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\n");
  printf("\t-R reflectance input image\t\t load the reflectance image\n");
  printf("\t-C color input image\t\t\t load the color image\n");
  printf("\t-A range input image\t\t\t load the range image\n");
  printf("\t-p projectionMethod\t\t\t projection method [EQUIRECTANGULAR|CONIC|CYLINDRICAL|MERCATOR|RECTILINEAR|PANNINI|STEREOGRAPHIC|ZAXIS]\n");
  printf("\t-m minHorizAngle\t\t\t Scanner horizontal view minAngle \n");
  printf("\t-w maxHorizAngle\t\t\t Scanner horizontal view maxAngle \n");
  printf("\t-n minVertAngle\t\t\t\t Scanner vertical view minAngle \n");
  printf("\t-x maxVertAngle\t\t\t\t Scanner vertical view maxAngle \n");
  printf("\t-F recoveredRangeFilterationMethod\t Filteration method to use for recovered point cloud from panorama image, use specially for lossy JPEG \n");
  printf("\t\t\t\t\t\t [INTERQUARTILE | INTERQUARTILE_AVERAGEDIFF | DISABLE_RECOVERED_RANGE_FILTERATION] default is DISABLE_RECOVERED_RANGE_FILTERATION \n");
  printf("\t-N numberOfNeighbours\t\t\t Number of pixels in each direction to check for filteration process, a squareof 2*numberOfNeighbours by 2*numberOfNeighbours default is 3\n");
  printf("\t-I interquartileScaleFactor\t\t Scale factor to use for INTERQUARTILE filteration, lower numbers make the method more agressive default is 3\n");
  printf("\t-D averageDiff\t\t\t\t Max diff to the average of the range in the neighbourhood around the pixel for AVERAGEDIFF method in meters default is 1\n");
  printf("\n");
  exit(1);
}

void parssArgs(int argc, char** argv, information& info)
{
  //default values
  info.projectionMethod = EQUIRECTANGULAR;
  info.minHorizAngle = 0;
  info.maxHorizAngle = 360;
  info.minVertAngle = -40;
  info.maxVertAngle = 60;
  info.recoveredRangeFilterationMethod = DISABLE_RECOVERED_RANGE_FILTERATION;
  info.numberOfNeighbours = 3;
  info.interquartileScaleFactor = 3;
  info.averageDiff = 1;
  int c;
  opterr = 0;
  //reade the command line and get the options
  while ((c = getopt (argc, argv, "A:C:D:F:I:m:n:N:O:p:R:t:w:x:")) != -1)
    switch (c)
      {
      case 'A':	
	info.inputRangeImage = optarg;
	break;
      case 'C':
	info.inputColorImage = optarg;
	break;
      case 'D':
	info.averageDiff = atof(optarg);
	break;
      case 'F':
	info.recoveredRangeFilterationMethod = stringToRecoveredRangeFilterationMethod(optarg);
	break;
      case 'I':
	info.interquartileScaleFactor = atof(optarg);
	break;
      case 'm':
	info.minHorizAngle = atoi(optarg);
	break;
      case 'n':
	info.minVertAngle = atoi(optarg);
	break;
      case 'N':
	info.numberOfNeighbours = atoi(optarg);
	break;
      case 'O':
	info.outputDir = optarg;
	if(info.outputDir.back() != '/')
	  info.outputDir += '/';
	break;
      case 'p':
	info.projectionMethod = stringToProjectionMethod(optarg);
	break;
      case 'R':
	info.inputReflectanceImage = optarg;
	break;
      case 't':
	info.inputImageType = stringToPanoramaType(optarg);
	break;
      case 'w':
	info.maxHorizAngle = atoi(optarg);
	break;
      case 'x':
	info.maxVertAngle = atoi(optarg);
	break;
      case '?':
	cout<<"Unknown option character "<<optopt<<endl;
	usage(argc, argv);
	break;
      default:
	usage(argc, argv);
      }

  if(info.outputDir.empty())
    {
      cout<<"The output dir is not available."<<endl;
      usage(argc, argv);
    }
  if(info.inputImageType == ThreeChannel24BitRange)
    {
      cout<<"argc= "<<argc<<endl;
      cout<<"optind= "<<optind<<endl;
      if(argc - optind != 1)
	{
	  cout<<"Error with  input arguments. The ThreeChannel24BitRange image is requiered"<<endl;
	  usage(argc, argv);
	}
      else
	{
	  info.inputThreeChannel24BitRangeImage = argv[optind];
	}
    }

  if(info.inputImageType == ThreeGrayscaleRange)
    {
      if(argc - optind != 3)
	{
	  cout<<"Error with  input arguments. Three grayscale range images are requiered"<<endl;
	  usage(argc, argv);
	}
      else
	{
	  info.inputThreeGrayscaleRangeImage1 = argv[optind];
	  info.inputThreeGrayscaleRangeImage2 = argv[optind+1];
	  info.inputThreeGrayscaleRangeImage3 = argv[optind+2];
	}
    }

}

void printInfo(information info)
{
  cout<<"Input eange image type= "<<panoramaTypeToString(info.inputImageType)<<endl;
  cout<<"Input Three Channel 24 Bit Range Image= "<<info.inputThreeChannel24BitRangeImage<<endl;
  cout<<"Input Three Grayscale Range Image 1= "<<info.inputThreeGrayscaleRangeImage1<<endl;
  cout<<"Input Three Grayscale Range Image 2= "<<info.inputThreeGrayscaleRangeImage2<<endl;
  cout<<"Input Three Grayscale Range Image 3= "<<info.inputThreeGrayscaleRangeImage3<<endl;
  cout<<"Input Range Image= "<<info.inputRangeImage<<endl;
  cout<<"Input Reflectance Image= "<<info.inputReflectanceImage<<endl;
  cout<<"Input Color Image= "<<info.inputColorImage<<endl;
  cout<<"Output Dir= "<<info.outputDir<<endl;
  cout<<"Projection Method= "<<projectionMethodToString(info.projectionMethod)<<endl;
  cout<<"minHorizAngle= "<<info.minHorizAngle<<endl;
  cout<<"maxHorizAngle= "<<info.maxHorizAngle<<endl;
  cout<<"minVertAngle= "<<info.minVertAngle<<endl;
  cout<<"maxVertAngle= "<<info.maxVertAngle<<endl;
  cout<<"recoveredRangeFilterationMethod= "<<recoveredRangeFilterationMethodToString(info.recoveredRangeFilterationMethod)<<endl;
  cout<<"numberOfNeighbours= "<<info.numberOfNeighbours<<endl;
  cout<<"interquartileScaleFactor= "<<info.interquartileScaleFactor<<endl;
  cout<<"averageDiff= "<<info.averageDiff<<endl;
  cout<<"------------------------------------"<<endl<<endl;
}

bool filterRecoveredRange(information info, int height, int width, cv::Mat iThreeChannel24BitRange, cv::Mat iThreeGrayscaleRange1, cv::Mat iThreeGrayscaleRange2, cv::Mat iThreeGrayscaleRange3)
{
  bool interquartileStatus = true;
  bool averageDiffStatus= true;
  
  //filtering only if the filteration method is != DISABLE_RECOVERED_RANGE_FILTERATION
  if(info.recoveredRangeFilterationMethod != DISABLE_RECOVERED_RANGE_FILTERATION)
    { 
      vector<float> rangeNeighbours;
      float range;
      for(int i = -info.numberOfNeighbours; i <= info.numberOfNeighbours; i++)
	{
	  for(int j = -info.numberOfNeighbours; j <= info.numberOfNeighbours; j++)
	    {
	      if(height >= abs(i) && height <= info.imageHeight-1-abs(i) && width >= abs(j) && width <= info.imageWidth-1-abs(j))// && i !=0 && j != 0)
		{
		  float tempRange = 0.0;
		  unsigned char tempByte;
		  if(info.inputImageType == ThreeChannel24BitRange)
		    {
		      tempByte = iThreeChannel24BitRange.at<cv::Vec3b>(height + i, width + j)[0];
		      tempRange+= tempByte<<16;
		      tempByte = iThreeChannel24BitRange.at<cv::Vec3b>(height + i, width + j)[1];
		      tempRange+= tempByte<<8;
		      tempByte = iThreeChannel24BitRange.at<cv::Vec3b>(height + i, width + j)[2];
		      tempRange+= tempByte;
		      tempRange/= 10000;
		    }
		  else if(info.inputImageType == ThreeGrayscaleRange)
		    {
		      tempByte = iThreeGrayscaleRange1.at<uchar>(height + i, width + j);
		      tempRange+= tempByte<<16;
		      tempByte = iThreeGrayscaleRange2.at<uchar>(height + i, width + j);
		      tempRange+= tempByte<<8;
		      tempByte = iThreeGrayscaleRange3.at<uchar>(height + i, width + j);
		      tempRange+= tempByte;
		      tempRange/= 10000;
		    }
		  if(tempRange != 0)
		    rangeNeighbours.push_back(tempRange);
		  if(i == 0 && j == 0)
		    range = tempRange;
		}
	    }
	}
      
      //sort the data in ranges
      sort(rangeNeighbours.begin(), rangeNeighbours.end());
      
      //find the outliers
      double q1 = 0, q2 = 0, q3 = 0;
      int size = rangeNeighbours.size();
      if(size >= 4)
	{
	  //even
	  if(size % 2 == 0)
	    {
	      q1 = (rangeNeighbours[(int)(size/4)] + rangeNeighbours[(int)(size/4) -1]) / 2;
	      q2 = (rangeNeighbours[(int)(size/2)] + rangeNeighbours[(int)(size/2) -1]) / 2;
	      q3 = (rangeNeighbours[(int)(size/4)*3] + rangeNeighbours[(int)(size/4)*3 -1]) / 2;
	    }
	  //odd
	  else
	    {
	      q1 = rangeNeighbours[(int)(size/4)];
	      q2 = rangeNeighbours[(int)(size/2)];
	      q3 = rangeNeighbours[3*(int)(size/4)];
	    }
	}
      
      double interQuartile = q3 - q1;
      double innerFence = interQuartile * info.interquartileScaleFactor;
      double minInnerBound = q1 - innerFence;
      double maxInnerBound = q3 - innerFence;
      //double outerFence = interQuartile * 3;
      //double minOuterBound = q1 - outerFence;
      //double maxOuterBound = q3 + outerFence;
      
      //startus of interquartile filteration
      interquartileStatus = (range > minInnerBound && range < maxInnerBound);
      
      if(info.recoveredRangeFilterationMethod == INTERQUARTILE_AVERAGEDIFF)
	{
	  double average = 0.0;
	  if(rangeNeighbours.size() != 0)
	    {
	      for(int r = 0; r < rangeNeighbours.size(); r++)
		average += rangeNeighbours[r];
	      
	      average /= rangeNeighbours.size();
	      double diff = fabs(range - average);
	      
	      //status of average diff filteration
	      averageDiffStatus = diff < info.averageDiff;
	    }
	}
    }
  return interquartileStatus && averageDiffStatus;
}

int main(int argc, char** argv)
{
  parssArgs(argc, argv, info);
  printInfo(info);

  //panorama containers
  cv::Mat iRange;
  cv::Mat iThreeChannel24BitRange;
  cv::Mat iThreeGrayscaleRange1;
  cv::Mat iThreeGrayscaleRange2;
  cv::Mat iThreeGrayscaleRange3;
  cv::Mat iReflectance;
  cv::Mat iColor;

  switch(info.inputImageType){
  case ThreeChannel24BitRange:
    if(info.inputThreeChannel24BitRangeImage.empty() == false)
      {
	iThreeChannel24BitRange = cv::imread(info.inputThreeChannel24BitRangeImage.c_str(), -1);
	if(iThreeChannel24BitRange.type() != CV_8UC3)
	  {
	    cout<<"The Three Channel 24 Bit Range has a wrong type."<<endl;
	    return 0;
	  }
	
	info.imageHeight = iThreeChannel24BitRange.rows;
	info.imageWidth = iThreeChannel24BitRange.cols;
      }
    else
      {
	cout<<"The Three Channel 24 Bit Range is not available."<<endl;
	return 0;
      }
    break;
  case ThreeGrayscaleRange:
    if(info.inputThreeGrayscaleRangeImage1.empty() == false
       && info.inputThreeGrayscaleRangeImage2.empty() == false
       && info.inputThreeGrayscaleRangeImage3.empty() == false)
      {
	iThreeGrayscaleRange1 = cv::imread(info.inputThreeGrayscaleRangeImage1.c_str(), -1);
	iThreeGrayscaleRange2 = cv::imread(info.inputThreeGrayscaleRangeImage2.c_str(), -1);
	iThreeGrayscaleRange3 = cv::imread(info.inputThreeGrayscaleRangeImage3.c_str(), -1);
	if(iThreeGrayscaleRange1.type() != CV_8UC1
	   || iThreeGrayscaleRange2.type() != CV_8UC1
	   || iThreeGrayscaleRange3.type() != CV_8UC1)
	  {
	    cout<<"One of the Three Grayscale Range images has a wrong type."<<endl;
	    return 0;
	  }
	if(iThreeGrayscaleRange1.rows != iThreeGrayscaleRange2.rows
	   || iThreeGrayscaleRange1.rows != iThreeGrayscaleRange3.rows
	   || iThreeGrayscaleRange2.rows != iThreeGrayscaleRange3.rows
	   || iThreeGrayscaleRange1.cols != iThreeGrayscaleRange2.cols
	   || iThreeGrayscaleRange1.cols != iThreeGrayscaleRange3.cols
	   || iThreeGrayscaleRange2.cols != iThreeGrayscaleRange3.cols)
	  {
	    cout<<"Input images are not the same size."<<endl;
	    return 0;
	  }
	info.imageHeight = iThreeGrayscaleRange1.rows;
	info.imageWidth = iThreeGrayscaleRange1.cols;
      }
    else
      {
	cout<<"The Three Channel 24 Bit Range is not available."<<endl;
	return 0;
      }
    break;
  default:
    cout<<"The proper input range image is not avaialable."<<endl;
    return 0;
    break;
  }
  
  //TODO: check for type and channels as well
  if(info.inputRangeImage.empty() == false)
    {
      iRange = cv::imread(info.inputRangeImage.c_str(), -1);
      if(info.imageHeight != iRange.rows)
	{
	  iRange.release();
	  cout<<"The Height of Range image is not the same as Three Channel 24 Bit Range image."<<endl;
	  cout<<"Not using Range image."<<endl;
	}
      if(info.imageWidth != iRange.cols)
	{
	  iRange.release();
	  cout<<"The Width of Range image is not the same as Three Channel 24 Bit Range image."<<endl;
	  cout<<"Not using Range image."<<endl;
	}
    }

  if(info.inputReflectanceImage.empty() == false)
    {
      iReflectance = cv::imread(info.inputReflectanceImage.c_str(), -1);
      if(info.imageHeight != iReflectance.rows)
	{
	  iReflectance.release();
	  cout<<"The Height of Reflectance image is not the same as Three Channel 24 Bit Range image."<<endl;
	  cout<<"Not using Reflectance image."<<endl;
	}
      if(info.imageWidth != iReflectance.cols)
	{
	  iReflectance.release();
	  cout<<"The Width of Reflectance image is not the same as Three Channel 24 Bit Range image."<<endl;
	  cout<<"Not using Reflectance image."<<endl;
	}
    }

  if(info.inputColorImage.empty() == false)
    {
      iColor = cv::imread(info.inputColorImage.c_str(), -1);
      if(info.imageHeight != iColor.rows)
	{
	  iColor.release();
	  cout<<"The Height of color image is not the same as Three Channel 24 Bit Range image."<<endl;
	  cout<<"Not using Color image."<<endl;
	}
      if(info.imageWidth != iColor.cols)
	{
	  iColor.release();
	  cout<<"The Width of Colore image is not the same as Three Channel 24 Bit Range image."<<endl;
	  cout<<"Not using Color image."<<endl;
	}
    }

  //get the projection object
  projection proj(info.imageWidth, info.imageHeight, info.projectionMethod, -1, -1, -1, -1, info.minHorizAngle, info.maxHorizAngle, info.minVertAngle, info.maxVertAngle, false);
    
  //read the panorama and write it to a file
  //get the name of outputscan from the input ThreeChannel24BitRange image
  //images are usualy in the scanXXX-info-image type.file format
  string inputFile;
  if(info.inputImageType == ThreeChannel24BitRange)
    {
      inputFile = info.inputThreeChannel24BitRangeImage;
    }
  else if(info.inputImageType == ThreeGrayscaleRange)
    {
      inputFile = info.inputThreeGrayscaleRangeImage1;
    }

  string inputImageName;
  string outputScanName;
  std::size_t found;
  //found = info.inputThreeChannel24BitRangeImage.find_last_of("/\\");
  found = inputFile.find_last_of("/\\");
  inputImageName = inputFile.substr(found+1);
  found = inputImageName.find("_");
  if (found == std::string::npos)
    {
      found = inputImageName.find(".");
    }
  outputScanName = inputImageName.substr (0, found);
  ofstream os(info.outputDir + outputScanName + ".3d");

  //get the color map and add it to unordered map
  vector<unsigned int> colorMapVector = getAllRGBSortedByHSL();
  std::unordered_map<unsigned int, unsigned int> colorMap;
  for(unsigned int c = 0; c < colorMapVector.size(); c++)
    {
      colorMap.insert({colorMapVector[c], c});
    }
  
  //calculate the range
  for(int h = 0; h < info.imageHeight; h++)
    {
      for(int w = 0; w < info.imageWidth; w++)
	{
	  float range = 0.0;
	  unsigned int color = 0;
	  unsigned int R = 0, G = 0, B = 0;;
	  unsigned char byte;
	  if(info.inputImageType == ThreeChannel24BitRange)
	    {
	      R = iThreeChannel24BitRange.at<cv::Vec3b>(h,w)[0];
	      //range+= byte<<16;
	      //R= byte<<16;
	      G = iThreeChannel24BitRange.at<cv::Vec3b>(h,w)[1];
	      //range+= byte<<8;
	      //G= byte<<8;
	      B = iThreeChannel24BitRange.at<cv::Vec3b>(h,w)[2];
	      //range+= byte;
	      //color+= byte;
	      //range/= 10000;
	    }
	  else if(info.inputImageType == ThreeGrayscaleRange)
	    {
	      R = iThreeGrayscaleRange1.at<uchar>(h,w);
	      //range+= byte<<16;
	      //color+= byte<<16;
	      G = iThreeGrayscaleRange2.at<uchar>(h,w);
	      //range+= byte<<8;
	      //color+= byte<<8;
	      B = iThreeGrayscaleRange3.at<uchar>(h,w);
	      //range+= byte;
	      //color+= byte;
	      //range/= 10000;
	    }

	  color = (R<<16) | (G<<8) | (B);
	  std::unordered_map<unsigned int , unsigned int>::const_iterator mapPair = colorMap.find (color);
	  if ( mapPair == colorMap.end() )
	    {
	      std::cout << "not found";
	      range = 0;
	    }
	  else
	    {
	      
	      range = (unsigned int)mapPair->second;
	      range/= 10000;
	      //std::cout << myPair->first << " is " << myPair->second;
	    }

	  
	  //get filteration status for pixel [h, w]
	  //if no filteration it returns true
	  bool filterationStatus = filterRecoveredRange(info, h, w, iThreeChannel24BitRange, iThreeGrayscaleRange1, iThreeGrayscaleRange2, iThreeGrayscaleRange3);

	  if(filterationStatus)
	    {
	      // cout<<h<<" "<<w<<" "<<range<<endl;
	      double x, y, z, reflectance, r, g, b;
	      proj.calcPointFromPanoramaPosition(x, y, z, h, w, range);
	      if(iReflectance.empty() == false)
		{
		  reflectance = iReflectance.at<uchar>(h,w);
		}
	      if(iColor.empty() == false)
		{
		  r =  iColor.at<cv::Vec3f>(h,w)[0];
		  g =  iColor.at<cv::Vec3f>(h,w)[1];
		  b =  iColor.at<cv::Vec3f>(h,w)[2];
		}
	      
	      if( fabs(x) > 1e-5 && fabs(y) > 1e-5 && fabs(z) > 1e-5) 
		{
		  
		  if(iReflectance.empty() == false && iColor.empty() == false)
		    {
		      os << x <<" "<< y <<" "<< z <<" "<< reflectance <<" "<< r <<" "<< g <<" "<< b <<"\n";
		    }
		  if(iReflectance.empty() == false && iColor.empty() == true)
		    {
		      os << x <<" "<< y <<" "<< z <<" "<< reflectance <<"\n";
		    }
		  if(iReflectance.empty() == true && iColor.empty() == false)
		    {
		      os << x <<" "<< y <<" "<< z <<" "<< r <<" "<< g <<" "<< b <<"\n";
		    }
		  if(iReflectance.empty() == true && iColor.empty() == true)
		    {
		      os << x <<" "<< y <<" "<< z <<" "<<"\n";
		    }
		}
	    }
	}
      //cout<<endl;
    }
}


