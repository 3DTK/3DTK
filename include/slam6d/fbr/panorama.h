/*
 * panorama definition
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file panorama.h
 * @brief create panorama images from 3D scans.
 *        This class is a panorama image container with different projections.
 *        It creates panoramic images with specified resolutions.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 */

#ifndef __PANORAMA_H__
#define __PANORAMA_H__

#include "slam6d/fbr/fbr_global.h"
#include "slam6d/point_type.h"
#include "slam6d/fbr/projection.h"

using namespace std;

namespace fbr{
  /**
   * @class panorama
   * create panorama images with use of projection class [different 
   * projection methods] from input scan files(Mat from scan_cv class) 
   * in opencv Mat format or Octtree files 
   * @param iReflectance_ panorama image from reflectance data             
   * @param iRange_ panorama image from range data             
   * @param iColor_ panorama image from color data             
   * @param iMap_ panorama map of 3D cartesian coordinate of input scan
            (same points as iRange and iReflectance and iColor)
   * @param extendedIMap_ 3D vector as panorama map with all the points
   * @param maxRange_ the maximum range of the scan
   * @param mapMethod_ the method for creating the map [FARTHEST | EXTENDED]
   * @param projection_ pointer to projectionClass Handler
   */
  class panorama {
    
  public:
    /**
     * constructor of class panorama
     * @param width unsigned int the width of the panorama image
     * @param height unsigned int the height of the panorama image
     * @param method projection_method the projection method
     * @param numberOfImages unsigned int number of subsets to create panorama image
     * @param param double special parameter for pannini or stereographic projections
     * @param mapMethod panorama_map_method mapping method for panorama image and 3D points
     * @param minZ float the min value of Z
     * @param maxZ float the max value of Z
     * @param maxAngle double the vertcal max angle of scan
     * @param minAngle double the vertcal min angle of scan
     * @param imageSizeOptimization bool a flag for optimizing the image size
     */
    panorama();
    panorama (unsigned int width,
              unsigned int height,
              projection_method method);
    panorama (unsigned int width,
              unsigned int height,
              projection_method method,
              unsigned int numberOfImages);
    panorama (unsigned int width,
              unsigned int height,
              projection_method method,
              unsigned int numberOfImages,
              double param);
    panorama (unsigned int width,
              unsigned int height,
              projection_method method,
              unsigned int numberOfImages,
              double param,
              panorama_map_method mapMethod);
    panorama (unsigned int width,
              unsigned int height,
              projection_method method,
              unsigned int numberOfImages,
              double param,
              panorama_map_method mapMethod,
	      float minZ,
	      float maxZ,
	      double MINVERTANGLE,
	      double MAXVERTANGLE);
    panorama (unsigned int width,
              unsigned int height,
              projection_method method,
              unsigned int numberOfImages,
              double param,
              panorama_map_method mapMethod,
	      float minZ,
	      float maxZ,
	      double MINVERTANGLE,
	      double MAXVERTANGLE,
	      bool imageSizeOptimization);

    panorama (unsigned int width,
	      unsigned int height,
	      projection_method method,
	      unsigned int numberOfImages,
	      double param,
	      panorama_map_method mapMethod,
	      float minZ,
	      float maxZ,
	      double MINVERTANGLE,
	      double MAXAVERTNGLE,
	      bool imageSizeOptimization,
	      bool reflectance,
	      bool range,
	      bool color);
    
    panorama (unsigned int width,
	      unsigned int height,
	      projection_method method,
	      unsigned int numberOfImages,
	      double param,
	      panorama_map_method mapMethod,
	      float minZ,
	      float maxZ,
	      double MINHORIZANGLE,
	      double MAXAHORIZNGLE,
	      double MINVERTANGLE,
	      double MAXAVERTNGLE,
	      bool imageSizeOptimization,
	      bool reflectance,
	      bool range,
	      bool color);
    
    void init(unsigned int width,
              unsigned int height,
              projection_method method,
              unsigned int numberOfImages,
              double param,
              panorama_map_method mapMethod,
	      float minZ = 0, 
	      float maxZ = 0,
	      double MINHORIZANGLE = 0,
	      double MAXHORIZANGLE = 360,
	      double MINVERTANGLE = -40,
	      double MAXVERTANGLE = 60,
	      bool imageSizeOptimization = false,
	      bool reflectance = false,
	      bool range = true,
	      bool color = false);
    
    void clear();

    /**
     * @brief creates the panorama reflectance image and map and color image.
     */
    //create panorama from cv::Mat
    void createPanorama(cv::Mat scan);
    void createPanorama(cv::Mat scan, cv::Mat color);
    
    //create panorama point by point from octree
    void createPanoramaFromOctree(PointType pointtype, scanner_type sType, double minReflectacne, double maxReflectance);
    //accept points from octree
    template <class T>
      void accept(T *point);
    //add the point from octree to panorama
    void addPointToPanorama(float x, float y, float z, float reflectance, float r, float g, float b);

    /**
     * @brief recovers the point cloud from the panorama image and
              range information
     * @param imageIMAGE - input range image to be converted to point cloud
     * @param reflectanceIMAGE - input reflectance image to be converted to point cloud
     * @param reducedPoints - recoverd point cloud
     */
    void recoverPointCloud(const cv::Mat& rangeImage,
                           cv::Mat& reflectanceImage,
                           vector<cv::Vec4f> &reducedPoints);
    
    //get params
    unsigned int getImageWidth();
    unsigned int getImageHeight();
    float getMaxRange();
    projection_method getProjectionMethod();
    unsigned int getNumberOfImages();
    double getProjectionParam();
    panorama_map_method getMapMethod();
    void getDescription();

    //get panorama and maps
    cv::Mat getReflectanceImage();
    cv::Mat getRangeImage();
    /**
     * Brief this function will normalize the range image to (0-1) 
     */
    cv::Mat getNormalizedRangeImage(); 
    /**
     * Brief this function will put the float range in to 24 bit rgb (3*8) type mat
     */
    cv::Mat getThreeChannel24BitRangeImage();
    /**
     * Brief this function will put the float range in to three grayscale images
     */
    void getThreeGrayscaleRangeImages(cv::Mat& range1, cv::Mat& range2, cv::Mat& range3);
    cv::Mat getColorImage();
    
    cv::Mat getMap();
    vector<vector<vector<cv::Vec3f> > > getExtendedMap();

    
  private:
    /**
     * Brief initializes the containers for map and range, reflectance and color images
     */
    void initMap();
    void map(int x,
             int y,
             cv::MatIterator_<cv::Vec4f> it,
	     cv::MatIterator_<cv::Vec3f> itColor,
             double range);
 
    //flags for panorrama images
    bool reflectance_;
    bool range_;
    bool color_;
    //container and panorama params
    cv::Mat iReflectance_;
    cv::Mat iMap_;
    cv::Mat iRange_;
    cv::Mat iColor_;
    float maxRange_;
    vector<vector<vector<cv::Vec3f> > > extendedIMap_;
    projection* projection_;
    panorama_map_method mapMethod_;
    //parameters for the creation of panoram from octree 
    PointType pointtype_;
    scanner_type sType_;
    double minReflectance_, maxReflectance_;
  };
}
#endif // __PANORAMA_H__
