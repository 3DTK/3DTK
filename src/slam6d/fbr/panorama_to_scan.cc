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

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

using namespace std;
using namespace fbr;

struct information{
  string inputRangeImage;
  string input24BitThreeChannelRangeImage;
  string inputReflectanceImage;
  string inputColorImage;
  int imageWidth, imageHeight;
  projection_method projectionMethod;
  int minVertAngle, maxVertAngle;
  int minHorizAngle, maxHorizAngle;
  string outputDir;
} info;

void usage(int argc, char** argv)
{
  printf("\n");
  printf("USAGE: %s -c threechannel range image -O output dir\n", argv[0]);
  printf("\n");
  printf("\n");
  printf("\t-c 24BitThreeChannelRange\t load the 24 bit three channel range image\n");
  printf("\t-O output dir\t\t\t output Dir\n");
  printf("\n");
  printf("\tOptions:\n");
  printf("\n");
  printf("\t-R reflectance input image\t load the reflectance image\n");
  printf("\t-C color input image\t\t load the color image\n");
  printf("\t-A range input image\t\t load the range image\n");
  printf("\t-p projectionMethod\t\t projection method [EQUIRECTANGULAR|CONIC|CYLINDRICAL|MERCATOR|RECTILINEAR|PANNINI|STEREOGRAPHIC|ZAXIS]\n");
  printf("\t-m minHorizAngle\t\t Scanner horizontal view minAngle \n");
  printf("\t-w maxHorizAngle\t\t Scanner horizontal view maxAngle \n");
  printf("\t-n minVertAngle\t\t\t Scanner vertical view minAngle \n");
  printf("\t-x maxVertAngle\t\t\t Scanner vertical view maxAngle \n");
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

  int c;
  opterr = 0;
  //reade the command line and get the options
  while ((c = getopt (argc, argv, "A:c:C:m:n:p:R:O:w:x:")) != -1)
    switch (c)
      {
      case 'A':
	info.inputRangeImage = optarg;
	break;
      case 'c':
	info.input24BitThreeChannelRangeImage = optarg;
	break;
      case 'C':
	info.inputColorImage = optarg;
	break;
      case 'm':
	info.minHorizAngle = atoi(optarg);
	break;
      case 'n':
	info.minVertAngle = atoi(optarg);
	break;
      case 'p':
	info.projectionMethod = stringToProjectionMethod(optarg);
	break;
      case 'R':
	info.inputReflectanceImage = optarg;
	break;
      case 'O':
	info.outputDir = optarg;
	if(info.outputDir.back() != '/')
	  info.outputDir += '/';
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
  if(info.input24BitThreeChannelRangeImage.empty())
    {
      cout<<"The three channel range image is not available."<<endl;
      usage(argc, argv);
    }
}

void printInfo(information info)
{
  cout<<"Input 24 Bit Three Channel Range Image= "<<info.input24BitThreeChannelRangeImage<<endl;
  cout<<"Input Range Image= "<<info.inputRangeImage<<endl;
  cout<<"Input Reflectance Image= "<<info.inputReflectanceImage<<endl;
  cout<<"Input Color Image= "<<info.inputColorImage<<endl;
  cout<<"Output Dir= "<<info.outputDir<<endl;
  cout<<"Projection Method= "<<projectionMethodToString(info.projectionMethod)<<endl;
  cout<<"minHorizAngle= "<<info.minHorizAngle<<endl;
  cout<<"maxHorizAngle= "<<info.maxHorizAngle<<endl;
  cout<<"minVertAngle= "<<info.minVertAngle<<endl;
  cout<<"maxVertAngle= "<<info.maxVertAngle<<endl;
  cout<<"------------------------------------"<<endl<<endl;
}

int main(int argc, char** argv)
{
  parssArgs(argc, argv, info);
  printInfo(info);

  //panorama containers
  cv::Mat iRange;
  cv::Mat i24BitThreeChannelRange;
  cv::Mat iReflectance;
  cv::Mat iColor;
  
  if(info.input24BitThreeChannelRangeImage.empty() == false)
    {
      i24BitThreeChannelRange = cv::imread(info.input24BitThreeChannelRangeImage.c_str(), 1);
      if(i24BitThreeChannelRange.type() != CV_8UC3)
	{
	  cout<<"The 24 Bit Three Channel Range has a wrong type."<<endl;
	  return 0;
	}

      info.imageHeight = i24BitThreeChannelRange.rows;
      info.imageWidth = i24BitThreeChannelRange.cols;
    }
  else
    {
      cout<<"The 24 Bit Three Channel Range is not available."<<endl;
      return 0;
    }

  //TODO: check for type and channels as well
  if(info.inputRangeImage.empty() == false)
    {
      iRange = cv::imread(info.inputRangeImage.c_str(), 1);
      if(info.imageHeight != iRange.rows)
	{
	  iRange.release();
	  cout<<"The Height of Range image is not the same as 24 Bit Three Channel Range image."<<endl;
	  cout<<"Not using Range image."<<endl;
	}
      if(info.imageWidth != iRange.cols)
	{
	  iRange.release();
	  cout<<"The Width of Range image is not the same as 24 Bit Three Channel Range image."<<endl;
	  cout<<"Not using Range image."<<endl;
	}
    }

    if(info.inputReflectanceImage.empty() == false)
    {
      iReflectance = cv::imread(info.inputReflectanceImage.c_str(), 1);
      if(info.imageHeight != iReflectance.rows)
	{
	  iReflectance.release();
	  cout<<"The Height of Reflectance image is not the same as 24 Bit Three Channel Range image."<<endl;
	  cout<<"Not using Reflectance image."<<endl;
	}
      if(info.imageWidth != iReflectance.cols)
	{
	  iReflectance.release();
	  cout<<"The Width of Reflectance image is not the same as 24 Bit Three Channel Range image."<<endl;
	  cout<<"Not using Reflectance image."<<endl;
	}
    }

    if(info.inputColorImage.empty() == false)
    {
      iColor = cv::imread(info.inputColorImage.c_str(), 1);
      if(info.imageHeight != iColor.rows)
	{
	  iColor.release();
	  cout<<"The Height of color image is not the same as 24 Bit Three Channel Range image."<<endl;
	  cout<<"Not using Color image."<<endl;
	}
      if(info.imageWidth != iColor.cols)
	{
	  iColor.release();
	  cout<<"The Width of Colore image is not the same as 24 Bit Three Channel Range image."<<endl;
	  cout<<"Not using Color image."<<endl;
	}
    }

    projection proj(info.imageWidth, info.imageHeight, info.projectionMethod, -1, -1, -1, -1, info.minHorizAngle, info.maxHorizAngle, info.minVertAngle, info.maxVertAngle, false);
    
    //read the panorama and write it to a file
    //get the name of outputscan from the input 24BitThreeChannelRange image
    //images are usualy in the scanXXX-info-image type.file format
    string inputImageName;
    string outputScanName;
    std::size_t found;
    found = info.input24BitThreeChannelRangeImage.find_last_of("/\\");
    inputImageName = info.input24BitThreeChannelRangeImage.substr(found+1);
    found = inputImageName.find("_");
    if (found == std::string::npos)
      {
	found = inputImageName.find(".");
      }
    outputScanName = inputImageName.substr (0, found);
    ofstream os(info.outputDir + outputScanName + ".3d");
    
    for(int h = 0; h < info.imageHeight; h++)
      {
	for(int w = 0; w < info.imageWidth; w++)
	  {
	    float range = 0.0;
	    unsigned char byte;
	    byte = i24BitThreeChannelRange.at<cv::Vec3b>(h,w)[0];
	    range+= byte<<16;
	    byte = i24BitThreeChannelRange.at<cv::Vec3b>(h,w)[1];
	    range+= byte<<8;
	    byte = i24BitThreeChannelRange.at<cv::Vec3b>(h,w)[2];
	    range+= byte;
	    range/= 10000;

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
}
