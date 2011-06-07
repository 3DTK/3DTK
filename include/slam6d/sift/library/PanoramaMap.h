/**
 * @file PanoramaMap.h
 * @brief Create map files with different projections and also create panorama images
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
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
#define STEREOGRAPHIC 9

class PanoramaMap 
{
 public:
  //panninid is the d variable in pannini projection equations and n is the number of images per scan for pannini projection and stereor is the r parameter of stereographic
  PanoramaMap(PolarPointCloud* cloud, int width, int height, int method, double panninid, int imagen, double stereor);
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
  int n;//number of images per scan 
  double r;//stereoghraphic projection parameter
};

#endif /* PANORAMAMAP_H_ */
