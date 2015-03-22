/*
 * projection definition
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file projection.h
 * @brief defines panorama projections for panorama class
 *        This class implements different projections for panorama.
 *        It creates panoramic projections with specified resolutions.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 */

#ifndef __PROJECTION_H__
#define __PROJECTION_H__

#include "fbr_global.h"
#include "slam6d/point_type.h"

using namespace std;

namespace fbr{
  /**
   * @class projection
   *        different projection methods for panorama images
   * @param width_ width of the panorama image/projection (cols in opencv)
   * @param height_ height of panorama image/projection (rows in opencv)
   * @param method_ projection method for panorama creation
   * @param numbrOfImages_ number of images per scan specially for Rectilinear,
            Pannini and Stereographic projections
   * @param param_ special d parameter of Pannini projection
            (see Master Thesis for more info) or special R parameter of
            Stereographic projection (Master Thesis for more info)
	    or phi_s for EQUALAREACYLINDRICAL projection
   * @param minZ_, maxZ_ min and max of Z
   * @param maxVertAngle_, minVertAngle_, maxHorizAngle_, minHorizAngle_ max and min vertical and horizontalangle of scan
   * @param imageSizeOprimization_ flag for image size optimization method
   * @param xSize_, ySize_, xFactor_, yFactor_, withMax_, heightMax_, heightLow_ Panormam projection parameters
   * @param  lat0_, long0_, phi1_, phi2_, n_, c_, rho0_, 
            xMax_, xMin_, yMin_, yMax_ Coninc projection params
   * @param coscRectilinear_ rectilinear projection param
   * @param sPannini_ pannini projection param
   * @param k_ stereographic projetion param
   * @param l0_, p1_, iMinX_, iMaxX_, iMinY_, iMaxY_, interval_, min_, max_ parameters for ptojection with more than one image
   * @param kprime_ azimuthal projection param
   */
  class projection {

      public:
    /**
     * constructor of class projection
     * @param width the width of the panorama image
     * @param height the height of the panorama image
     * @param method the projection method
     * @param numberOfImages number of subsets to crate panorama image
     * @param param special parameter for pannini or stereographic projections
     * @param minZ float the min value of Z
     * @param maxZ float the max value of Z
     * @param maxHorizAngle double the horizontal max angle of scan
     * @param minHorizAngle double the horizontal min angle of scan
     * @param maxVertAngle double the vertcal max angle of scan
     * @param minVertAngle double the vertcal min angle of scan
     * @param imageSizeOptimization bool a flag for optimizing the image size
     */
    projection();
    projection(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, double minZ, double maxZ, double minHorizAngle, double maxHorizAngle, double minVertAngle, double maxVertAngle, bool imageSizeOptimization);

    //init the class
    void init(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, double minZ, double maxZ, double minHorizAngle, double maxHorizAngle, double minVertAngle, double maxVertAngle, bool imageSizeOptimization);

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
    /**
     * @breif caclculates the x and y and z of the 3D point from panorama projection
     * @param x - the x of point
     * @param y - the y of point
     * @param z - the z of point
     * @param row - of the panorama
     * @param col - of the panorama
     * @param range - range of the point
     */
    void calcPointFromPanoramaPosition(double& x, double& y, double& z, int row, int col, double range);
    /**
     * @breif caclculates the x and y of the 3D point on panorama projection
     * @param x - the x on panorama
     * @param y - the y on panorama
     * @param it - the point
     * @param range - range of the point
     */
    void calcPanoramaPositionForAPoint(int &x, int &y, cv::MatIterator_<cv::Vec4f> it, double &range);

    //get params
    unsigned int getProjectionWidth();
    unsigned int getProjectionHeight();
    projection_method getProjectionMethod();
    unsigned int getProjectionNumberOfImages();
    double getProjectionParam();

  private:
    /**
     * @brief sets the image ratio and optimize it if flag is true
     */
    void setImageRatio();

    //input projection params
    unsigned int width_;
    unsigned int height_;
    projection_method method_;
    unsigned int numberOfImages_;
    double param_;
    float minZ_, maxZ_;
    double minVertAngle_, maxVertAngle_, minHorizAngle_, maxHorizAngle_;
    bool imageSizeOptimization_;
    
    //panorama projeciton params
    double xSize_, ySize_;
    double xFactor_, yFactor_;
    int widthMax_, heightMax_;
    double heightLow_;
    //conic params
    double lat0_, long0_;
    double phi1_, phi2_;
    double n_, c_, rho0_;
    double xMax_, xMin_, yMin_, yMax_;
    //rectilinear params
    double coscRectilinear_;
    //pannini params
    double sPannini_;
    //stereographic params
    double k_;
    //projections with more than one image
    double l0_, p1_, iMinX_, iMaxX_, iMinY_, iMaxY_, interval_, min_, max_;
    //azimuthal projection
    double kPrime_;
  };
}
#endif // __PROJECTION_H__
