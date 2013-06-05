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

#include "fbr_global.h"

using namespace std;

namespace fbr{
  /**
   * @class panorama
   *        create panorama images with different projection methods from
            input scan files(Mat from scan_cv class) in opencv Mat format
   * @param iReflectance panorama image from reflectance data             
   * @param iRange panorama image from range data             
   * @param iColor panorama image from color data             
   * @param iMap panorama map of 3D cartesian coordinate of input scan
            (same points as iRange and iReflectance)
   * @param extendedIMap 3D vector as panorama map with all the points
   * @param iWidth width of the panorama image (cols in opencv)
   * @param iHeight height of panorama image (rows in opencv)
   * @param pMethod projection method for panorama creation
   * @param nImage number of images per scan specially for Rectilinear,
            Pannini and Stereographic projections
   * @param pParam special d parameter of Pannini projection
            (see Master Thesis for more info) or special R parameter of
            Stereographic projection (Master Thesis for more info)
	    or phi_s for EQUALAREACYLINDRICAL projection
   */
  class panorama {
    cv::Mat iReflectance;
    cv::Mat iMap;
    cv::Mat iRange;
    cv::Mat iColor;
    float maxRange;
    vector<vector<vector<cv::Vec3f> > > extendedIMap;
    unsigned int iWidth;
    unsigned int iHeight;
    projection_method pMethod;
    unsigned int nImages;
    double pParam;
    panorama_map_method mapMethod;
    float zMin, zMax;
    double MAX_ANGLE, MIN_ANGLE;
    bool iOptimization;


    void map(int x,
             int y,
             cv::MatIterator_<cv::Vec4f> it,
             double range);
    void mapColor(int x, int y, cv::MatIterator_<cv::Vec3f> itColor);
    void initMat();
    void setImageRatio(double xSize, double ySize);

  public:
    /**
     * constructor of class panorama
     * @param width the width of the panorama image
     * @param height the height of the panorama image
     * @param method the projection method
     * @param images number of subsets to crate panorama image
     * @param param special parameter for pannini or stereographic projections
     * @param mapMethod mapping method for panorama image and 3D points
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
	      float min,
	      float max,
	      double MINANGLE,
	      double MAXANGLE);
    panorama (unsigned int width,
              unsigned int height,
              projection_method method,
              unsigned int numberOfImages,
              double param,
              panorama_map_method mapMethod,
	      float min,
	      float max,
	      double MINANGLE,
	      double MAXANGLE,
	      bool imageOptimization);

    void init(unsigned int width,
              unsigned int height,
              projection_method method,
              unsigned int numberOfImages,
              double param,
              panorama_map_method mapMethod,
	      float min = 0, 
	      float max = 0,
	      double MINANGLE = -40,
	      double MAXANGLE = 60,
	      bool imageOptimization = false);

    /**
     * @brief creates the panorama reflectance image and map and color image.
     */
    void createPanorama(cv::Mat scan);
    void createPanorama(cv::Mat scan, cv::Mat color);
    /**
     * @brief recovers the point cloud from the panorama image and
              range information
     * @param image - input range image to be converted to point cloud
     * @param file - destination of .3d file containing the point cloud
     */
    void recoverPointCloud(const cv::Mat& range_image,
                           cv::Mat& reflectance_image,
                           vector<cv::Vec4f> &reduced_points);

    unsigned int getImageWidth();
    unsigned int getImageHeight();
    float getMaxRange();
    projection_method getProjectionMethod();
    unsigned int getNumberOfImages();
    double getProjectionParam();
    cv::Mat getReflectanceImage();
    cv::Mat getMap();
    cv::Mat getRangeImage();
    cv::Mat getColorImage();
    vector<vector<vector<cv::Vec3f> > > getExtendedMap();
    panorama_map_method getMapMethod();
    void getDescription();
  };
}
#endif // __PANORAMA_H__
