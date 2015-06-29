/*
 * panorama implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file 
 * @brief Implementation of panorama image generation
 * @author Hamidreza Houshiar, Jacobs University Bremen, Germany
 * @author Andreas Nuechter. University of Wuerzburg, Germany.
 * @author Julia Kauer. University of Wuerzburg, Germany.
 * @author Lukas Gradl. University of Wuerzburg, Germany.
 */

#include "slam6d/fbr/panorama.h"
#include <limits.h>

using namespace std;

namespace fbr
{

  panorama::panorama()
  {
    init(3600, 1000, EQUIRECTANGULAR, 1, 0, FARTHEST);
  }

  panorama::panorama(unsigned int width, unsigned int height, projection_method method)
  { 
    double param = 0;
    unsigned int numberOfImages = 1;
    if(method == RECTILINEAR)
      numberOfImages = 3;
    else if(method == PANNINI)
      {
	numberOfImages = 3;
	param = 1;
      } 
    else if (method == STEREOGRAPHIC)
      {
	numberOfImages = 3;
	param = 2;
      }

    init(width, height, method, numberOfImages, param, FARTHEST);
  }

  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages)
  {
    double param = 0;
    if(method == PANNINI)
      param = 1;
    else if (method == STEREOGRAPHIC)
      param = 2;
   
    init(width, height, method, numberOfImages, param, FARTHEST);
  }  
  
  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param)
  { 
    init(width, height, method, numberOfImages, param, FARTHEST);
  }

  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod)
  { 
    init(width, height, method, numberOfImages, param, mMethod);
  }

  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod, float minZ, float maxZ, double minVertAngle, double maxVertAngle)
  { 
    init(width, height, method, numberOfImages, param, mMethod, minZ, maxZ, 0, 360, minVertAngle, maxVertAngle);
  }

  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod, float minZ, float maxZ, double minVertAngle, double maxVertAngle, bool imageSizeOptimization)
  { 
    init(width, height, method, numberOfImages, param, mMethod, minZ, maxZ, 0, 360, minVertAngle, maxVertAngle, imageSizeOptimization);
  }
  
  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod, float minZ, float maxZ, double minVertAngle, double maxVertAngle, bool imageSizeOptimization, bool reflectance, bool range, bool color)
  { 
    init(width, height, method, numberOfImages, param, mMethod, minZ, maxZ, 0, 360, minVertAngle, maxVertAngle, imageSizeOptimization, reflectance, range, color);
  }

  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod, float minZ, float maxZ, double minHorizAngle, double maxHorizAngle, double minVertAngle, double maxVertAngle, bool imageSizeOptimization, bool reflectance, bool range, bool color)
  { 
    init(width, height, method, numberOfImages, param, mMethod, minZ, maxZ, minHorizAngle, maxHorizAngle, minVertAngle, maxVertAngle, imageSizeOptimization, reflectance, range, color);
  }

  void panorama::init(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod, float minZ, float maxZ, double minHorizAngle, double maxHorizAngle, double minVertAngle, double maxVertAngle, bool imageSizeOptimization, bool reflectance, bool range, bool color)
  {  
    projection_ = new projection(width, height, method, numberOfImages, param, minZ, maxZ, minHorizAngle, maxHorizAngle, minVertAngle, maxVertAngle, imageSizeOptimization);
    
    //clear the containers
    clear();

    maxRange_ = 0;
    minRange_ = std::numeric_limits<int>::max();
    mapMethod_ = mMethod;
    reflectance_ = reflectance;
    range_ = range;
    color_ = color;
  }
  
  void panorama::clear()
  {
    //clear the containers
    iReflectance_.release();
    iMap_.release();
    iRange_.release();
    iColor_.release();
    extendedIMap_.clear();
  }

  void panorama::createPanorama(cv::Mat scan) 
  {
    cv::Mat color;
    createPanorama(scan, color);
  }

  void panorama::createPanorama(cv::Mat scan, cv::Mat color)
  {
    initMap();      	

    cv::MatIterator_<cv::Vec4f> it, end; 
    cv::MatIterator_<cv::Vec3f> itColor;
    if(color.empty() == false)
      {
	itColor = color.begin<cv::Vec3f>();
      }

    for( it = scan.begin<cv::Vec4f>(), end = scan.end<cv::Vec4f>(); it != end; ++it)
      {
	int x, y;
	double range;
	projection_->calcPanoramaPositionForAPoint(x, y, it, range);

	//create the iReflectance iRange iolor and map
	map(x, y, it, itColor, range);

	//increase the color
	if(color.empty() == false)
	  {
	    ++itColor;
	  }  
      }
  }
  
  void panorama::createPanoramaFromOctree(PointType pointtype, scanner_type sType, double minReflectance, double maxReflectance)
  {
    initMap();
    
    pointtype_ = pointtype;
    sType_ = sType;
    minReflectance_ = minReflectance;
    maxReflectance_ = maxReflectance;
  }
  
  template <class T>
  void panorama::accept(T *point)
  {
    float x, y, z, reflectance, red, green, blue;
    x = point[0];
    y = point[1];
    z = point[2];

    if(pointtype_.hasReflectance())
      {
	int idx = pointtype_.getReflectance();
	reflectance = point[idx];
      }
    else
      reflectance = 255;

    //normalize the reflectance
    reflectance = fbr::normalizeTheReflectance(reflectance, sType_, minReflectance_, maxReflectance_);

    if(pointtype_.hasColor())
      {
        int idx = pointtype_.getColor();
        blue = ((unsigned char*) &(point[idx]))[0];
        green = ((unsigned char*) &(point[idx]))[1];
        red = ((unsigned char*) &(point[idx]))[2];
      }
    else
      {
	red = 0;
	green = 0;
	blue = 0;
      }
    addPointToPanorama(x, y, z, reflectance, red, green, blue);
  }

  void panorama::addPointToPanorama(float x, float y, float z, float reflectance, float r, float g, float b)
  {
    if(iReflectance_.empty() == false)
      {
	cv::Mat point(1,1,CV_32FC(4),cv::Scalar(x,y,z,reflectance));
	cv::Mat pointColor(1,1,CV_32FC(3),cv::Scalar(r,g,b));
	
	cv::MatIterator_<cv::Vec4f> it;
	it = point.begin<cv::Vec4f>();
	
	cv::MatIterator_<cv::Vec3f> itColor;
	itColor = pointColor.begin<cv::Vec3f>();
	
	int x, y;
	double range;
	projection_->calcPanoramaPositionForAPoint(x, y, it, range);
	
	//create the iReflectance iRange iColor and map
	map(x, y, it, itColor,  range);
      }
    else
      {
	cout<<"Panorma has not been initialized!";
      }
  }

  void panorama::recoverPointCloud(const cv::Mat& rangeImage,
				   cv::Mat& reflectanceImage, vector<cv::Vec4f> &reducedPoints) 
  {
    projection_->recoverPointCloud(rangeImage, reflectanceImage, reducedPoints);
  }

  unsigned int panorama::getImageWidth()
  {
    return projection_->getProjectionWidth();
  }

  unsigned int panorama::getImageHeight()
  {
    return projection_->getProjectionHeight();
  }

  float panorama::getMaxRange()
  {
    return maxRange_;
  }
  
  projection_method panorama::getProjectionMethod()
  {
    return projection_->getProjectionMethod();
  }

  unsigned int panorama::getNumberOfImages()
  {
    return projection_->getProjectionNumberOfImages();
  }
  
  double panorama::getProjectionParam()
  {
    return projection_->getProjectionParam();
  }
  
  panorama_map_method panorama::getMapMethod()
  {
    return mapMethod_;
  }

  void panorama::getDescription()
  {
    cout << "panorama created with width: " << projection_->getProjectionWidth() << ", and height: "
	 << projection_->getProjectionHeight() << ", and projection method: " << projectionMethodToString(projection_->getProjectionMethod())
	 << ", number of images: " << projection_->getProjectionNumberOfImages() << ", projection param: " << projection_->getProjectionParam() << "."
	 << endl;
    cout << endl;
  }

  cv::Mat panorama::getReflectanceImage()
  {
    return iReflectance_;
  }

  cv::Mat panorama::getRangeImage()
  {
    return iRange_;
  }

  cv::Mat panorama::getNormalizedRangeImage()
  {
    cv::Mat normalizediRange;
    iRange_.convertTo(normalizediRange, CV_8UC1, 255.0);
    return normalizediRange;
  }

  cv::Mat panorama::getThreeChannel24BitRangeImage()
  {
    vector<unsigned int> colorMap = getAllRGBSortedByHSL();

    int width = projection_->getProjectionWidth();
    int height = projection_->getProjectionHeight();

    cv::Mat threeChannelRange;
    threeChannelRange.create(height, width, CV_8UC3);
    threeChannelRange = cv::Scalar::all(0);
    for(int h = 0; h < height; h++)
      {
	for(int w = 0; w < width; w++)
	  {
	    //unsigned char bytes[3];
	    unsigned int rangeValue = (int)(iRange_.at<float>(h,w) * 10000);
	    
	    //threeChannelRange.at<cv::Vec3b>(h,w)[0] = (rangeValue >> 16) & 0xFF;
	    //threeChannelRange.at<cv::Vec3b>(h,w)[1] = (rangeValue >> 8) & 0xFF;
	    //threeChannelRange.at<cv::Vec3b>(h,w)[2] = rangeValue & 0xFF;

	    unsigned int R, G, B;
	    unsigned int color = colorMap[rangeValue];

	    colorToRGB8BitBy8Bit(color, R, G, B);

	    threeChannelRange.at<cv::Vec3b>(h,w)[0] = R;
	    threeChannelRange.at<cv::Vec3b>(h,w)[1] = G;
	    threeChannelRange.at<cv::Vec3b>(h,w)[2] = B;
	  }
      }
    
    return threeChannelRange;
  }

  void panorama::getThreeGrayscaleRangeImages(cv::Mat& range1, cv::Mat& range2, cv::Mat& range3)
  {
    vector<unsigned int> colorMap = getAllRGBSortedByHSL();
    
    int width = projection_->getProjectionWidth();
    int height = projection_->getProjectionHeight();

    range1.create(height, width, CV_8UC1);
    range2.create(height, width, CV_8UC1);
    range3.create(height, width, CV_8UC1);
    range1 = cv::Scalar::all(0);
    range2 = cv::Scalar::all(0);
    range3 = cv::Scalar::all(0);
    for(int h = 0; h < height; h++)
      {
	for(int w = 0; w < width; w++)
	  {
	    //unsigned char bytes[3];
	    unsigned int rangeValue = (int)(iRange_.at<float>(h,w) * 10000);
	    
	    //range1.at<uchar>(h,w) = (rangeValue >> 16) & 0xFF;
	    //range2.at<uchar>(h,w) = (rangeValue >> 8) & 0xFF;
	    //range3.at<uchar>(h,w) = rangeValue & 0xFF;

	    unsigned int R, G, B;
	    unsigned int color = colorMap[rangeValue];

	    colorToRGB8BitBy8Bit(color, R, G, B);

	    range1.at<uchar>(h,w) = R;
	    range2.at<uchar>(h,w) = G;
	    range3.at<uchar>(h,w) = B;
	  }
      }
  }
  
  cv::Mat panorama::getColorImage()
  {
    return iColor_;
  }
  
  cv::Mat panorama::getMap()
  {
    return iMap_;
  }

  vector<vector<vector<cv::Vec3f> > > panorama::getExtendedMap()
  {
    return extendedIMap_;
  }


  /////Private functions

  void panorama::initMap()
  {
    int width = projection_->getProjectionWidth();
    int height = projection_->getProjectionHeight();
    if(mapMethod_ == FARTHEST)
      {
	iMap_.create(height, width, CV_32FC(3));
	iMap_ = cv::Scalar::all(0);
      }
    else if(mapMethod_ == EXTENDED)
      {
	extendedIMap_.resize(height);
	for (unsigned int i = 0; i < height; i++)
	  extendedIMap_[i].resize(width);
      }
    //init the compresed map
    else if(mapMethod_ == FULL)
    {	

    }

    //init panorama images
    if(reflectance_ == true)
      {
	iReflectance_.create(height, width, CV_8U);
	iReflectance_ = cv::Scalar::all(0);
      }
    if(range_ == true)
      {
	iRange_.create(height, width, CV_32FC(1));
	iRange_ = cv::Scalar::all(0);
      }
    if(color_ == true)
      {
	iColor_.create(height, width, CV_32FC(3));
	iColor_ = cv::Scalar::all(0);
      }
  }

  void panorama::map(int x, int y, cv::MatIterator_<cv::Vec4f> it, cv::MatIterator_<cv::Vec3f> itColor, double range)
  {    
    if (maxRange_ < (float)range)
      maxRange_ = (float)range;
    if (minRange_ > (float)range)
      minRange_ = (float)range;

    
    if (iRange_.at<float>(y,x) < range) 
      {
	// reflectance
	if(reflectance_ == true)
       	  iReflectance_.at<uchar>(y,x) = (*it)[3]*255;
	// range
	if(range_ == true)
	  iRange_.at<float>(y,x) = (float)range;
	// rgb
	if(color_ == true)
	  {
	    iColor_.at<cv::Vec3f>(y,x)[0] = (*itColor)[0];//r
	    iColor_.at<cv::Vec3f>(y,x)[1] = (*itColor)[1];//g
	    iColor_.at<cv::Vec3f>(y,x)[2] = (*itColor)[2];//b
	  }
      }

    // adding the point with max distance
    if (mapMethod_ == FARTHEST) 
      {
	if (iRange_.at<float>(y,x) < range) 
	  {
	    //adding farthest point
	    iMap_.at<cv::Vec3f>(y,x)[0] = (*it)[0]; // x
	    iMap_.at<cv::Vec3f>(y,x)[1] = (*it)[1]; // y
	    iMap_.at<cv::Vec3f>(y,x)[2] = (*it)[2]; // z
	  }
      }
    //extended map
    else if(mapMethod_ == EXTENDED)
      {
	// adding all the points
	cv::Vec3f point;
	point[0] = (*it)[0]; // x
	point[1] = (*it)[1]; // y
	point[2] = (*it)[2]; // z
	extendedIMap_[y][x].push_back(point);
      }
    //compressed map
    else if(mapMethod_ == FULL)
      {
	
      }
  }
    
}

