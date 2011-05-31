/**
 * PanoramaMap.h
 *
 *  Created on: Feb 20, 2010
 *      Author: darko
 *      Maintained : HamidReza Mey 04, 2011
 */

#ifndef PANORAMAMAP_H_
#define PANORAMAMAP_H_

#include "SuperPixel.h"
#include "PolarPointCloud.h"
#include <string>

//for creating panorama images
#define RANGE 1
#define REFELCTANCE 2

//for selecting different projections
#define STANDARD 3 //Equirectangular Projection
#define CYLINDRICAL 4
#define NEW 5 //Z axis projection
#define MERCATOR 6
#define RECTILINEAR 7
#define PANNINI 8

class PanoramaMap 
{
 public:
  //panninid is the d variable in pannini projection equations and panninin is the number of images per scan for pannini projection
  PanoramaMap(PolarPointCloud* cloud, int width, int height, int method, double panninid, int panninin);
  //used in generatesiftfeaturs.cpp
  PanoramaMap(const char* filename); //deserialize
  PanoramaMap();
  
  void serialize(const char* filename);
  void toJpeg(std::string filename, int method);
  
  virtual ~PanoramaMap();
  
 public:

  double mina; //horizontal angle
  double maxa; //horizontal angle
  
  double minb; //vertical angle
  double maxb; //vertical angle
  
  double minz; //for projection
  double maxz; //for projection
  
  std::string scanid;
  SuperPixel **data;
  int width;
  int height;
  int projection;
  double d;//the Pannini projection parameter
  int n;//number of images per scan for pannini
};

#endif /* PANORAMAMAP_H_ */
