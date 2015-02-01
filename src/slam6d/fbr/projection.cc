/*
 * projection for panorama implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file 
 * @brief Implementation of panorama image projections
 * @author HamidReza Houshiar, Jacobs University Bremen, Germany
 * @author Andreas Nuechter. University of Wuerzburg, Germany.
 * @author Julia Kauer. University of Wuerzburg, Germany.
 * @author Lukas Gradl. University of Wuerzburg, Germany.
 */

#include "slam6d/fbr/projection.h"

using namespace std;

namespace fbr
{
  projection::projection()
  {
    init(3600, 1000, EQUIRECTANGULAR, 1, 0, 0, 0, -40, 60, false);
  }

  projection::projection(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, double minZ, double maxZ, double minAngle, double maxAngle, bool imageSizeOptimization)
  {
    init(width, height, method, numberOfImages, param, minZ, maxZ, minAngle, maxAngle, imageSizeOptimization);
  }

  void  projection::init(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, double minZ, double maxZ, double minAngle, double maxAngle, bool imageSizeOptimization)
  {
    //set the projection params
    //projection size
    width_ = width;
    height_ = height;
    //optimization of projection size
    imageSizeOptimization_ = imageSizeOptimization;
    //scanner==projection max and min angle
    maxAngle_ = maxAngle;
    minAngle_ = minAngle;
    //min and max of Z from user for zaxis projection
    minZ_ = minZ;
    maxZ_ = maxZ;
    //projction method
    method_ = method;
    //number of images for projection
    numberOfImages_ = numberOfImages;
    //projection specific param
    param_ = param;
    
    //based on projection method init the projection
    //EQUIRECTANGULAR projection
    if(method_ == EQUIRECTANGULAR)
      {
	//adding the longitude to x axis and latitude to y axis
	xSize_ = 2 * M_PI;
	ySize_ =  ((maxAngle_ - minAngle_) / 360.0 * 2.0 * M_PI);
	
	setImageRatio();

	xFactor_ = (double) width_ / xSize_;
	widthMax_ = width_ - 1;
	yFactor_ = (double) height_ / ySize_;      
	//shift all the valuse to positive points on image 
	heightLow_ = (0.0 - minAngle_) / 360.0 * 2.0 * M_PI;
	heightMax_ = height_ - 1;	
      }
    
    //CONIC projection
    if(method_ == CONIC)
      {
	// set up maximum latitude and longitude angles of the robot 
	minVertAngle_ = minAngle_ * M_PI / 180.0; 
	maxVertAngle_ = maxAngle_ * M_PI / 180.0;
	minHorizAngle_ = -M_PI; 
	maxHorizAngle_ = M_PI;
	// set up initial parameters according to MathWorld: http://mathworld.wolfram.com/AlbersEqual-AreaConicProjection.html
	lat0_ = 0.; 
	long0_ = 0.;
	phi1_ = -40. * M_PI / 180.0; 
	phi2_ = 60 * M_PI / 180.0;
	n_ = (sin(phi1_) + sin(phi2_)) / 2.;
	c_ = sqr(cos(phi1_)) + 2 * n_ * sin(phi1_);
	rho0_ = sqrt(c_ - 2 * n_ * sin(lat0_)) / n_;
	// set up max values for x and y and add the longitude to x axis and latitude to y axis
	xMax_ = (1./n_ * sqrt(c_ - 2*n_*sin( minVertAngle_ )) ) * sin(n_ * (maxHorizAngle_ - long0_));
	xMin_ = (1./n_ * sqrt(c_ - 2*n_*sin( minVertAngle_ )) ) * sin(n_ * (minHorizAngle_ - long0_));
	xSize_ =  ( xMax_ - xMin_ );
	
	yMin_ = rho0_ - (1./n_ * sqrt(c_ - 2*n_*sin(minVertAngle_)) ) * cos(n_ * ( 0. - long0_ ));
	yMax_ = rho0_ - (1./n_ * sqrt(c_ - 2*n_*sin(maxVertAngle_)) ) * cos(n_ * (maxHorizAngle_ - long0_ ));
	ySize_ =  ( yMax_ - yMin_ );
	
	setImageRatio();
	
	xFactor_ = (double) width_ / xSize_;
	widthMax_ = width_ - 1;
	yFactor_ = (double) height_ / ySize_;
	//shift all the values to positive points on image 
	heightMax_ = height_ - 1;	
      }
    
    //CYLINDRICAL projection
    if(method_ == CYLINDRICAL)
      {
	//adding the longitude to x and tan(latitude) to y
	xSize_ = 2 * M_PI;
	ySize_ = (tan(maxAngle_ / 360 * 2 * M_PI) - tan(minAngle_ / 360 * 2 * M_PI));
      
	setImageRatio();	
	
	//find the x and y range
	xFactor_ = (double) width_ / xSize_;
	widthMax_ = width_ - 1;
	yFactor_ = (double) height_ / ySize_;
	heightLow_ = (minAngle_) / 360 * 2 * M_PI;
	heightMax_ = height_ - 1;
      }

    //EQUALAREACYLINDRICAL projection
    if(method_ == EQUALAREACYLINDRICAL)
      {
	xSize_ =  (2 * M_PI * cos(param_ / 360 * 2 * M_PI));
	ySize_ =  ((sin(maxAngle_ / 360 * 2 * M_PI) - sin(minAngle_ / 360 * 2 * M_PI)) / cos(param_ / 360 * 2 * M_PI));
      
	setImageRatio();
      
	//find the x and y range
	xFactor_ = (double) width_ / xSize_;
	widthMax_ = width_ - 1;
	yFactor_ = (double) height_ / ySize_;
	heightLow_ = (minAngle_) / 360 * 2 * M_PI;
	heightMax_ = height_ - 1;
      }

    //Mercator Projection
    if(method_ == MERCATOR)
      {
	//find the x and y range
	xSize_ = 2 * M_PI;
	ySize_ =  ( log( tan( maxAngle_ / 360 * 2 * M_PI ) + ( 1 / cos( maxAngle_ / 360 * 2 * M_PI ) ) ) - log ( tan( minAngle_ / 360 * 2 * M_PI) + (1/cos(minAngle_ / 360 * 2 * M_PI) ) ) );
      
	setImageRatio();
      
	xFactor_ = (double) width_ / xSize_;
	widthMax_ = width_ - 1;
	yFactor_ = (double) height_ / ySize_;
	heightLow_ = log(tan(minAngle_ / 360 * 2 * M_PI) + (1/cos(minAngle_ / 360 * 2 * M_PI)));
	heightMax_ = height_ - 1;
      }

    //RECTILINEAR projection
    if(method_ == RECTILINEAR)
      {
	//default value for numberOfImages_
	if(numberOfImages_ < 3) numberOfImages_ = 3;
	cout<<"Number of images per scan is: "<<numberOfImages_<<endl;
	interval_ = 2 * M_PI / numberOfImages_;
	//iMinY_ = -M_PI/9;
	//iMaxY_ = 2*M_PI/9;
	iMinY_ = minAngle_ * M_PI / 180;
	iMaxY_ = maxAngle_ * M_PI / 180;

	//latitude of projection center
	p1_ = 0;

	iMinX_ = 0 * interval_;
	iMaxX_ = (0 + 1) * interval_;
	//the longitude of projection center
	l0_ = iMinX_ + interval_ / 2;
	//finding the min and max of the x direction
	coscRectilinear_ = sin(p1_) * sin(iMaxY_) + cos(p1_) * cos(iMaxY_) * cos(iMaxX_ - l0_);
	max_ = (cos(iMaxY_) * sin(iMaxX_ - l0_) / coscRectilinear_);
	coscRectilinear_ = sin(p1_) * sin(iMinY_) + cos(p1_) * cos(iMinY_) * cos(iMinX_ - l0_);
	min_ = (cos(iMinY_) * sin(iMinX_ - l0_) / coscRectilinear_);
	xSize_ =  (max_ - min_);
	//finding the min and max of y direction
	coscRectilinear_ = sin(p1_) * sin(iMaxY_) + cos(p1_) * cos(iMaxY_) * cos(iMaxX_ - l0_);
	max_ = ( (cos(p1_) * sin(iMaxY_) - sin(p1_) * cos(iMaxY_) * cos(iMaxX_ - l0_) )/ coscRectilinear_);
	coscRectilinear_ = sin(p1_) * sin(iMinY_) + cos(p1_) * cos(iMinY_) * cos(iMinX_ - l0_);
	min_ = ( (cos(p1_) * sin(iMinY_) - sin(p1_) * cos(iMinY_) * cos(iMinX_ - l0_) )/ coscRectilinear_);
	ySize_ = (max_ - min_);

	setImageRatio();
      }

    //PANNINI projection
    if(method_ == PANNINI)
      {

	//default values for numberOfImages_ and dPannini==param_
	if(param_ == 0) param_ = 1;
	if(numberOfImages_ < 2) numberOfImages_ = 2;
	cout << "Parameter d is: " << param_ <<", Horizontal Number of images per scan is: " << numberOfImages_ << endl;
	interval_ = 2 * M_PI / numberOfImages_;
	//iMinY_ = -M_PI/9;
	//iMaxY_ = 2*M_PI/9;
	iMinY_ = minAngle_ * M_PI / 180;
	iMaxY_ = maxAngle_ * M_PI / 180;
	//latitude of projection center
	p1_ = 0;
            
	iMinX_ = 0 * interval_;
	iMaxX_ = (0 + 1) * interval_;
	//the longitude of projection center
	l0_ = iMinX_ + interval_ / 2;
      
	//use the S variable of pannini projection mentioned in the thesis
	//finding the min and max of the x direction
	sPannini_ = (param_ + 1) / (param_ + sin(p1_) * tan(iMaxY_) + cos(p1_) * cos(iMaxX_ - l0_));
	max_ = sPannini_ * (sin(iMaxX_ - l0_));
	sPannini_ = (param_ + 1) / (param_ + sin(p1_) * tan(iMinY_) + cos(p1_) * cos(iMinX_ - l0_));
	min_ = sPannini_ * (sin(iMinX_ - l0_));
	xSize_ = max_ - min_;
	//finding the min and max of y direction
	sPannini_ = (param_ + 1) / (param_ + sin(p1_) * tan(iMaxY_) + cos(p1_) * cos(iMaxX_ - l0_));
	max_ = sPannini_ * (tan(iMaxY_) * (cos(p1_) - sin(p1_) * 1/tan(iMaxY_) * cos(iMaxX_ - l0_)));
	sPannini_ = (param_ + 1) / (param_ + sin(p1_) * tan(iMinY_) + cos(p1_) * cos(iMinX_ - l0_));
	min_ = sPannini_ * (tan(iMinY_) * (cos(p1_) - sin(p1_) * 1/tan(iMinY_) * cos(iMinX_ - l0_)));
	ySize_ = max_ - min_;
      
	setImageRatio();
      }

    //STEREOGRAPHIC projection
    if(method_ == STEREOGRAPHIC)
      {
	//default values for numberOfImages_ and rStereographic==param_
	if(param_ == 0) param_ = 2;
	if(numberOfImages_ < 2) numberOfImages_ = 2;
	cout << "Paremeter R is:" << param_ << ", Number of images per scan is:" << numberOfImages_ << endl;
	// l0_ and p1_ are the center of projection iminx, imaxx, iminy, imaxy are the bounderis of interval_s
	interval_ = 2 * M_PI / numberOfImages_;
	//iMinY_ = -M_PI/9;
	//iMaxY_ = 2*M_PI/9;
	iMinY_ = minAngle_ * M_PI / 180;
	iMaxY_ = maxAngle_ * M_PI / 180;
	//latitude of projection center
	p1_ = 0;
      
	iMinX_ = 0 * interval_;
	iMaxX_ = (0 + 1) * interval_;
	//longitude of projection center
	l0_ = iMinX_ + interval_ / 2;
	//use the R variable of stereographic projection mentioned in the thesis
	//finding the min and max of x direction
	k_ = (2 * param_) / (1 + sin(p1_) * sin(p1_) + cos(p1_) * cos(p1_) * cos(iMaxX_ - l0_));
	max_ = k_ * cos(p1_) * sin (iMaxX_ - l0_);
	k_ = (2 * param_) / (1 + sin (p1_) * sin(p1_) + cos(p1_) * cos(p1_) * cos(iMinX_ -l0_));
	min_ = k_ * cos(p1_) * sin (iMinX_ -l0_);
	xSize_ =  (max_ - min_);
	//finding the min and max of y direction
	k_ = (2 * param_) / (1 + sin(p1_) * sin(iMaxY_) + cos(p1_) * cos(iMaxY_) * cos(iMaxX_ - l0_));
	max_ = k_ * (cos(p1_) * sin(iMaxY_) - sin(p1_) * cos(iMaxY_) * cos(iMaxX_ - l0_));
	k_ = (2 * param_) / (1 + sin(p1_) * sin(iMinY_) + cos(p1_) * cos(iMinY_) * cos(iMinX_ - l0_));
	min_ = k_ * (cos(p1_) * sin(iMinY_) - sin(p1_) * cos(iMinY_) * cos(iMinX_ - l0_));
	ySize_ = (max_ - min_);

	setImageRatio();
      }

    //ZAXIS projection
    if(method_ == ZAXIS)
      {
	//double minZ_ = -100;
	//double maxZ_ = 100;
	//adding the longitude to x axis and latitude to y axis
	xSize_ = 2 * M_PI;
	ySize_ = (maxZ_ - minZ_);

	setImageRatio();

	xFactor_ = (double) width_ / xSize_;
	widthMax_ = width_ - 1;
	cout << "ZMAX= " << maxZ_ << " ZMIN= "<< minZ_ << endl;
	yFactor_ = (double) height_ / ySize_;
	//shift all the valuse to positive points on image 
	heightLow_ = minZ_;
	heightMax_ = height_ - 1;
      }
    
    //AZIMUTHAL projection
    if(method_ == AZIMUTHAL)
      {
	// set up maximum latitude and longitude angles of the robot 
	minVertAngle_ = minAngle_ * M_PI / 180.0;
	maxVertAngle_ = maxAngle_ * M_PI / 180.0;
	minHorizAngle_ = -M_PI; 
	maxHorizAngle_ = M_PI;
	// set up initial parameters according to MathWorld: http://mathworld.wolfram.com/LambertAzimuthalEqual-AreaProjection.html
	long0_ = 0.;
	phi1_ = 0. * M_PI / 180.0;

	// set up max values for x and y and add the longitude to x axis and latitude to y axis
	// sqrt(2/(1+sin(phi1_)*sin(theta)+cos(phi1_)*cos(theta)*cos(phi-long0_)));
	xMax_ =  sqrt(2/(1+sin(phi1_)*sin(maxHorizAngle_)+cos(phi1_)*cos(maxHorizAngle_)*cos((minVertAngle_/2+maxVertAngle_/2)-long0_)))*cos(maxHorizAngle_)*sin((minVertAngle_/2+maxVertAngle_/2)-long0_);
	xMin_ = sqrt(2/(1+sin(phi1_)*sin(minHorizAngle_)+cos(phi1_)*cos(minHorizAngle_)*cos((minVertAngle_/2+maxVertAngle_/2)-long0_)))*cos(minHorizAngle_)*sin((minVertAngle_/2+maxVertAngle_/2)-long0_);
	xSize_ =  ( xMax_ - xMin_ );

	yMin_ = xMin_;     // the thing is supposed to be circular, isn't it?
	yMax_ = xMax_;
	ySize_ =  ( yMax_ - yMin_ );
      
	setImageRatio();

	xSize_=1/2.;
	ySize_=1/2.;
	
	xFactor_ = (double) width_ / 2*xSize_;
	widthMax_ = width_ - 1;
	yFactor_ = (double) height_ / 2*ySize_;
	//shift all the values to positive points on image 
	heightMax_ = height_ - 1;
      }
  }

  void projection::recoverPointCloud(const cv::Mat& rangeImage,
				   cv::Mat& reflectanceImage, vector<cv::Vec4f> &reducedPoints) 
  {
    if (rangeImage.cols != reflectanceImage.cols
	|| rangeImage.rows != reflectanceImage.rows) 
      {
	cerr << "range image and reflectance image have different geometries - using empty range image" << endl;
	reflectanceImage.create(rangeImage.size(), CV_8U);
	reflectanceImage = cv::Scalar::all(0);
      }

    //recover from EQUIRECTANGULAR projection
    if(method_ == EQUIRECTANGULAR) 
      {
	xFactor_ = (double) rangeImage.size().width / 2.0 / M_PI;
	//int widthMax_ = rangeImage.size().width - 1;
	yFactor_ = (double) rangeImage.size().height / ((maxAngle_ - minAngle_) / 360.0 * 2.0 * M_PI);
	heightLow_ = (0.0 - minAngle_) / 360 * 2 * M_PI;
	heightMax_ = rangeImage.size().height - 1;

	bool first_seen = true;
	for (int row = 0; row < rangeImage.size().height; ++row) 
	  {
	    for (int col = 0; col < rangeImage.size().width; ++col) 
	      {
		float range = rangeImage.at<float>(row, col);
		float reflectance = reflectanceImage.at<uchar>(row,col)/255.0;
		float theta = (heightMax_ - row + 0.5) / yFactor_ - heightLow_; 
		float phi = (col + 0.5 ) / xFactor_; 
		phi *= 180.0 / M_PI;
		phi = 360.0 - phi;
		phi *= M_PI / 180.0;
		theta *= 180.0 / M_PI;
		theta *= -1;
		theta += 90.0;
		theta *= M_PI / 180.0;
		double polar[3] = { theta, phi, range }, cartesian[3] = {0., 0., 0.}; 
		toCartesian(polar, cartesian);
         
		if( fabs(cartesian[0]) < 1e-5 && fabs(cartesian[1]) < 1e-5 && fabs(cartesian[2]) < 1e-5) 
		  {
		    if (first_seen) first_seen = false;
		    else continue;
		  }
		reducedPoints.push_back(cv::Vec4f(-100.0*cartesian[1],
						   100.0*cartesian[2],
						   100.0*cartesian[0],
						   reflectance));
	      }
	  }
      }

    //recover from CYLINDRICAL projection
    if(method_ == CYLINDRICAL) 
      {
	xFactor_ = (double) rangeImage.size().width / 2 / M_PI;
	//int widthMax_ = rangeImage.size().width - 1;
	yFactor_ = (double) rangeImage.size().height / (tan(maxAngle_ / 360 * 2 * M_PI) - tan(minAngle_ / 360 * 2 * M_PI));
	heightLow_ = (minAngle_) / 360 * 2 * M_PI;
	//int heightMax_ = rangeImage.size().height - 1;
      
	bool first_seen = true;
	for (int row = 0; row < rangeImage.size().height; ++row) 
	  {
	    for (int col = 0; col < rangeImage.size().width; ++col) 
	      {
		float range = rangeImage.at<float>(row, col);
		float reflectance = reflectanceImage.at<uchar>(row,col)/255.0;
		float theta = atan2(row + 0.5 + yFactor_ * tan(heightLow_), yFactor_);
		float phi = (col + 0.5) / xFactor_; 
		phi *= 180.0 / M_PI;
		phi = 360.0 - phi;
		phi *= M_PI / 180.0;
		theta *= 180.0 / M_PI;
		theta *= -1;
		theta += 90.0;
		theta *= M_PI / 180.0;
		double polar[3] = { theta, phi, range }, cartesian[3] = {0., 0., 0.}; 
		toCartesian(polar, cartesian);

		if( fabs(cartesian[0]) < 1e-5 && fabs(cartesian[1]) < 1e-5 && fabs(cartesian[2]) < 1e-5) 
		  {
		    if (first_seen) first_seen = false;
		    else continue;
		  }
		reducedPoints.push_back(cv::Vec4f(-100.0*cartesian[1],
						   100.0*cartesian[2],
						   100.0*cartesian[0],
						   reflectance));
	      }
	  }
      }

    //recover from MERCATOR projection
    if(method_ == MERCATOR) 
      {
	xFactor_ = (double) rangeImage.size().width / 2 / M_PI;
	yFactor_ = (double) rangeImage.size().height / ( log( tan( maxAngle_ / 360 * 2 * M_PI ) + ( 1 / cos( maxAngle_ / 360 * 2 * M_PI ) ) ) - log ( tan( minAngle_ / 360 * 2 * M_PI) + (1/cos(minAngle_ / 360 * 2 * M_PI) ) ) );
	heightLow_ = log(tan(minAngle_ / 360 * 2 * M_PI) + (1/cos(minAngle_ / 360 * 2 * M_PI)));
	heightMax_ = rangeImage.size().height - 1;
      
	bool first_seen = true;
	for (int row = 0; row < rangeImage.size().height; ++row) 
	  {
	    for (int col = 0; col < rangeImage.size().width; ++col) 
	      {
		float range = rangeImage.at<float>(row, col);
		float reflectance = reflectanceImage.at<uchar>(row,col)/255.0;
		float theta = 2 * atan2(exp((heightMax_ - row + 0.5) / yFactor_ + heightLow_), 1.) - M_PI_2;
		float phi = (col + 0.5) / xFactor_; 
		phi *= 180.0 / M_PI;
		phi = 180.0 - phi;
		phi *= M_PI / 180.0;
		theta *= 180.0 / M_PI;
		theta *= -1;
		theta += 90.0;
		theta *= M_PI / 180.0;
		double polar[3] = { theta, phi, range }, cartesian[3] = {0., 0., 0.}; 
		toCartesian(polar, cartesian);

		if( fabs(cartesian[0]) < 1e-5 && fabs(cartesian[1]) < 1e-5 && fabs(cartesian[2]) < 1e-5) 
		  {
		    if (first_seen) first_seen = false;
		    else continue;
		  }
		reducedPoints.push_back(cv::Vec4f(-100.0*cartesian[1],
						   100.0*cartesian[2],
						   100.0*cartesian[0],
						   reflectance));
	      }
	  }
      }

    //recover from CONIC projection
    if(method_ == CONIC) 
      {
	// set up maximum latitude and longitude angles of the robot 
	minVertAngle_ = minAngle_ * M_PI / 180.0; 
	maxVertAngle_ = maxAngle_ * M_PI / 180.0;
	minHorizAngle_ = -M_PI; 
	maxHorizAngle_ = M_PI;
	// set up initial parameters according to MathWorld: http://mathworld.wolfram.com/AlbersEqual-AreaConicProjection.html
	lat0_ = 0.; 
	long0_ = 0.;
	phi1_ = -40. * M_PI / 180.0; 
	phi2_ = 60 * M_PI / 180.0;
	n_ = (sin(phi1_) + sin(phi2_)) / 2.;
	c_ = sqr(cos(phi1_)) + 2 * n_ * sin(phi1_);
	rho0_ = sqrt(c_ - 2 * n_ * sin(lat0_)) / n_;
	// set up max values for x and y and add the longitude to x axis and latitude to y axis
	xMax_ = (1./n_ * sqrt(c_ - 2*n_*sin( minVertAngle_ )) ) * sin(n_ * (maxHorizAngle_ - long0_));
	xMin_ = (1./n_ * sqrt(c_ - 2*n_*sin( minVertAngle_ )) ) * sin(n_ * (minHorizAngle_ - long0_));
	xFactor_ = (double) rangeImage.size().width / ( xMax_ - xMin_ );
	yMin_ = rho0_ - (1./n_ * sqrt(c_ - 2*n_*sin(minVertAngle_)) ) * cos(n_ * ( 0. - long0_ ));
	yMax_ = rho0_ - (1./n_ * sqrt(c_ - 2*n_*sin(maxVertAngle_)) ) * cos(n_ * (maxHorizAngle_ - long0_ ));
	yFactor_ = (double) rangeImage.size().height / ( yMax_ - yMin_ );
	heightMax_ = rangeImage.size().height - 1;

	bool first_seen = true;
	for (int row = 0; row < rangeImage.size().height; ++row) 
	  {
	    for (int col = 0; col < rangeImage.size().width; ++col) 
	      {
		float range = rangeImage.at<float>(row, col);
		float reflectance = reflectanceImage.at<uchar>(row,col)/255.0;
		float x = col * 1. / xFactor_ - fabs(xMin_);
		float y = (heightMax_ - row) * 1. / yFactor_ - fabs(yMin_);
		float theta = asin((c_ - (x*x + (rho0_ - y) * (rho0_ - y)) * n_ * n_) / (2 * n_));
		float phi = long0_ + (1./n_) * ::atan2(x, (float)rho0_ - y);

		phi *= 180.0 / M_PI;
		phi = 360.0 - phi;
		phi *= M_PI / 180.0;
		theta *= 180.0 / M_PI;
		theta *= -1;
		theta += 90.0;
		theta *= M_PI / 180.0;

		double polar[3] = { theta, phi, range }, cartesian[3] = {0., 0., 0.}; 
		toCartesian(polar, cartesian);

		//if ( std::isnan(cartesian[0]) || std::isnan(cartesian[1]) || std::isnan(cartesian[2]) ) continue;
		if( fabs(cartesian[0]) < 1e-5 && fabs(cartesian[1]) < 1e-5 && fabs(cartesian[2]) < 1e-5) 
		  {
		    if (first_seen) first_seen = false;
		    else continue;
		  }
		reducedPoints.push_back(cv::Vec4f(-100.0*cartesian[1],
						   100.0*cartesian[2],
						   100.0*cartesian[0],
						   reflectance));
	      }
	  }
      }
  }

  void projection::calcPanoramaPositionForAPoint(int &x, int &y, cv::MatIterator_<cv::Vec4f> it, double &range)
  {

    //EQUIRECTANGULAR projection
    if(method_ == EQUIRECTANGULAR)
      {
	double kart[3], polar[3], phi, theta;
	kart[0] = (*it)[2]/100;
	kart[1] = (*it)[0]/-100;
	kart[2] = (*it)[1]/100;
	toPolar(kart, polar);
	//theta == polar[0] == scan [4]
	//phi == polar[1] == scan [5]
	//range == polar[2] == scan [3]
	theta = polar[0] * 180 / M_PI;
	phi = polar[1] * 180 / M_PI;
	range = polar[2];
	//horizantal angle of view of [0:360] and vertical of [-40:60]
	phi = 360.0 - phi;
        phi *= M_PI / 180.0;
	theta -= 90;
	theta *= -1;
	theta *= 2.0 * M_PI / 360.0;
	
	x = (int) ( xFactor_ * phi);
	if (x < 0) x = 0;
	if (x > widthMax_) x = widthMax_;
	y = (int) ( yFactor_ * (theta + heightLow_) );
	y = heightMax_ - y;
	if (y < 0) y = 0;
	if (y > heightMax_) y = heightMax_;	
      }

    //CONIC projection
    if(method_ == CONIC)
      {
	double kart[3], polar[3], phi, theta;
        kart[0] = (*it)[2]/100;
        kart[1] = (*it)[0]/-100;
        kart[2] = (*it)[1]/100;
        toPolar(kart, polar);
        //theta == polar[0] == scan [4]
        //phi == polar[1] == scan [5]
        //range == polar[2] == scan [3]
        theta = polar[0] * 180 / M_PI;
        phi = polar[1] * 180 / M_PI;
        range = polar[2];
        //phi == longitude == horizantal angle of view of [0:360] 
        phi = 180.0 - phi;
        phi *= M_PI / 180.0;
        //theta == latitude == vertical angle of view of [-40:60]
        theta -= 90;
        theta *= -1;
        theta *= M_PI / 180.0;

	// add minimum x position as an offset
        x = (int) ( xFactor_ * (sqrt(c_ - 2 * n_ * sin( theta) ) / n_ * sin(n_ * (phi - long0_)) + fabs(xMin_) ) );
        if (x < 0) x = 0;
        if (x > widthMax_) x = widthMax_;
        
        // add minimum y position as an offset
        y = (int) ( yFactor_ * (rho0_ - (1/n_ * sqrt(c_ - 2 * n_ * sin( theta) ) ) * cos(n_ * (phi - long0_)) + fabs( yMin_ ) ) );
        y = heightMax_ - y;
        if (y < 0) y = 0;
        if (y > heightMax_) y = heightMax_;
      }
    
    //CYLINDRICAL projection
    if(method_ == CYLINDRICAL)
      {
      	double kart[3], polar[3], phi, theta;
	kart[0] = (*it)[2]/100;
	kart[1] = (*it)[0]/-100;
	kart[2] = (*it)[1]/100;
	toPolar(kart, polar);
	//theta == polar[0] == scan [4]
	//phi == polar[1] == scan [5]
	//range == polar[2] == scan [3]
	theta = polar[0] * 180 / M_PI;
	phi = polar[1] * 180 / M_PI;
	range = polar[2];
	//horizantal angle of view of [0:360] and vertical of [-40:60]
	phi = 360.0 - phi;
	phi = phi * 2.0 * M_PI / 360.0;
	theta -= 90;
	theta *= -1;
	theta *= 2.0 * M_PI / 360.0;
	
	x = (int) ( xFactor_ * phi);
	if (x < 0) x = 0;
	if (x > widthMax_) x = widthMax_;
	y = (int) ((double) yFactor_ * (tan(theta) - tan(heightLow_)));
	y = heightMax_ - y;
	if (y < 0) y = 0;
	if (y > heightMax_) y = heightMax_;	
      }



    //EQUALAREACYLINDRICAL projection
    if(method_ == EQUALAREACYLINDRICAL)
      {
      	double kart[3], polar[3], phi, theta;
	kart[0] = (*it)[2]/100;
	kart[1] = (*it)[0]/-100;
	kart[2] = (*it)[1]/100;
	toPolar(kart, polar);
	//theta == polar[0] == scan [4]
	//phi == polar[1] == scan [5]
	//range == polar[2] == scan [3]
	theta = polar[0] * 180 / M_PI;
	phi = polar[1] * 180 / M_PI;
	range = polar[2];
	//horizantal angle of view of [0:360] and vertical of [-40:60]
	phi = 360.0 - phi;
	phi = phi * 2.0 * M_PI / 360.0;
	theta -= 90;
	theta *= -1;
	theta *= 2.0 * M_PI / 360.0;

	x = (int) ( xFactor_ * (phi*cos(param_/360*2*M_PI)));
	if (x < 0) x = 0;
	if (x > widthMax_) x = widthMax_;
	y = (int) ((double) yFactor_ * ((sin(theta) - sin(heightLow_)) / cos(param_/360*2*M_PI)));
	y = heightMax_ - y;
	if (y < 0) y = 0;
	if (y > heightMax_) y = heightMax_;
      }

    
    //Mercator Projection
    if( method_ == MERCATOR)
      {
      	double kart[3], polar[3], phi, theta;
	kart[0] = (*it)[2]/100;
	kart[1] = (*it)[0]/-100;
	kart[2] = (*it)[1]/100;
	toPolar(kart, polar);
	//theta == polar[0] == scan [4]
	//phi == polar[1] == scan [5]
	//range == polar[2] == scan [3]
	theta = polar[0] * 180 / M_PI;
	phi = polar[1] * 180 / M_PI;
	range = polar[2];
	//horizantal angle of view of [0:360] and vertical of [-40:60]
	phi = 360.0 - phi;
	phi = phi * 2.0 * M_PI / 360.0;
	theta -= 90;
	theta *= -1;
	theta *= 2.0 * M_PI / 360.0;

	x = (int) ( xFactor_ * phi);
	if (x < 0) x = 0;
	if (x > widthMax_) x = widthMax_;
	y = (int) ( yFactor_ * (log(tan(theta) + (1/cos(theta))) - heightLow_) );
	y = heightMax_ - y;
	if (y < 0) y = 0;
	if (y > heightMax_) y = heightMax_;	
      }
    
    //RECTILINEAR projection
    if(method_ == RECTILINEAR)
      {
      	double kart[3], polar[3], phi, theta;
	kart[0] = (*it)[2]/100;
	kart[1] = (*it)[0]/-100;
	kart[2] = (*it)[1]/100;
	toPolar(kart, polar);
	//theta == polar[0] == scan [4]
	//phi == polar[1] == scan [5]
	//range == polar[2] == scan [3]
	theta = polar[0] * 180 / M_PI;
	phi = polar[1] * 180 / M_PI;
	range = polar[2];
	//horizantal angle of view of [0:360] and vertical of [-40:60]
	phi = 360.0 - phi;
	phi = phi * 2.0 * M_PI / 360.0;
	theta -= 90;
	theta *= -1;
	theta *= 2.0 * M_PI / 360.0;
	
	for(unsigned int j = 0 ; j < numberOfImages_ ; j++)
	  {
	    iMinX_ = j * interval_;
	    iMaxX_ = (j + 1) * interval_;
	    //check for point in interval_
	    if(phi <= iMaxX_ && phi >= iMinX_)
	      {
		
		//the longitude of projection center
		l0_ = iMinX_ + interval_ / 2;
		//finding the min and max of the x direction
		coscRectilinear_ = sin(p1_) * sin(iMaxY_) + cos(p1_) * cos(iMaxY_) * cos(iMaxX_ - l0_);
		max_ = (cos(iMaxY_) * sin(iMaxX_ - l0_) / coscRectilinear_);
		coscRectilinear_ = sin(p1_) * sin(iMinY_) + cos(p1_) * cos(iMinY_) * cos(iMinX_ - l0_);
		min_ = (cos(iMinY_) * sin(iMinX_ - l0_) / coscRectilinear_);
		xFactor_ = (double) (width_ / numberOfImages_) / (max_ - min_);
		double xlow = min_;
		widthMax_ = (width_ / numberOfImages_) - 1;
		//finding the min and max of y direction
		coscRectilinear_ = sin(p1_) * sin(iMaxY_) + cos(p1_) * cos(iMaxY_) * cos(iMaxX_ - l0_);
		max_ = ( (cos(p1_) * sin(iMaxY_) - sin(p1_) * cos(iMaxY_) * cos(iMaxX_ - l0_) )/ coscRectilinear_);
		coscRectilinear_ = sin(p1_) * sin(iMinY_) + cos(p1_) * cos(iMinY_) * cos(iMinX_ - l0_);
		min_ = ( (cos(p1_) * sin(iMinY_) - sin(p1_) * cos(iMinY_) * cos(iMinX_ - l0_) )/ coscRectilinear_);
		yFactor_ = (double) height_ / (max_ - min_);
		heightLow_ = min_;
		heightMax_ = height_ - 1;
		//project the points and add them to image
		coscRectilinear_ = sin(p1_) * sin(theta) + cos(p1_) * cos(theta) * cos(phi - l0_);
		
		x = (int)(xFactor_) * ((cos(theta) * sin(phi - l0_) / coscRectilinear_) - xlow);
		if (x < 0) x = 0;
		if (x > widthMax_) x = widthMax_;
		x = x + (j * width_ / numberOfImages_);

		y = (int) (yFactor_) * (( (cos(p1_) * sin(theta) - sin(p1_) * cos(theta) * cos(phi - l0_)) / coscRectilinear_) - heightLow_);
		y = heightMax_ - y;
		if (y < 0) y = 0;
		if (y > heightMax_) y = heightMax_;	
	      }
	  }
      }
    
    //PANNINI projection
    if(method_ == PANNINI)
      {
	double kart[3], polar[3], phi, theta;
	kart[0] = (*it)[2]/100;
	kart[1] = (*it)[0]/-100;
	kart[2] = (*it)[1]/100;
	toPolar(kart, polar);
	//theta == polar[0] == scan [4]
	//phi == polar[1] == scan [5]
	//range == polar[2] == scan [3]
	theta = polar[0] * 180 / M_PI;
	phi = polar[1] * 180 / M_PI;
	range = polar[2];
	//horizantal angle of view of [0:360] and vertical of [minAngle_:maxAngle_]
	phi = 360.0 - phi;
	phi = phi * 2.0 * M_PI / 360.0;
	theta -= 90;
	theta *= -1;
	theta *= 2.0 * M_PI / 360.0;

	for(unsigned int i = 0 ; i < numberOfImages_ ; i++)
	  {
	    iMinX_ = i * interval_;
	    iMaxX_ = (i + 1) * interval_;
	    //check for point in interval_
	    if(phi <= (iMaxX_) && phi >= (iMinX_))
	      {
		//the longitude of projection center
		l0_ = iMinX_ + interval_ / 2;
		
		//latitude of projection center
		p1_ = 0;
		
		//use the S variable of pannini projection mentioned in the thesis
		//finding the min and max of the x direction
		sPannini_ = (param_ + 1) / (param_ + sin(p1_) * tan(iMaxY_) + cos(p1_) * cos(iMaxX_ - l0_));
		max_ = sPannini_ * (sin(iMaxX_ - l0_));
		sPannini_ = (param_ + 1) / (param_ + sin(p1_) * tan(iMinY_) + cos(p1_) * cos(iMinX_ - l0_));
		min_ = sPannini_ * (sin(iMinX_ - l0_));
		xFactor_ = (double) (width_ / numberOfImages_) / (max_ - min_);
		double xlow = min_;
		widthMax_ = (width_ / numberOfImages_) - 1;
		//finding the min and max of y direction
		sPannini_ = (param_ + 1) / (param_ + sin(p1_) * tan(iMaxY_) + cos(p1_) * cos(iMaxX_ - l0_));
		max_ = sPannini_ * (tan(iMaxY_) * (cos(p1_) - sin(p1_) * 1/tan(iMaxY_) * cos(iMaxX_ - l0_)));
		sPannini_ = (param_ + 1) / (param_ + sin(p1_) * tan(iMinY_) + cos(p1_) * cos(iMinX_ - l0_));
		min_ = sPannini_ * (tan(iMinY_) * (cos(p1_) - sin(p1_) * 1/tan(iMinY_) * cos(iMinX_ - l0_)));
		yFactor_ = (double) height_ / (max_ - min_);
		heightLow_ = min_;
		heightMax_ = height_ - 1;
		//project the points and add them to image
		sPannini_ = (param_ + 1) / (param_ + sin(p1_) * tan(theta) + cos(p1_) * cos(phi - l0_));
		
		x = (int)(xFactor_) * (sPannini_ * sin(phi - l0_) - xlow);
		if (x < 0) x = 0;
		if (x > widthMax_) x = widthMax_;
		x = x + (i * widthMax_);
		
		y = (int) (yFactor_) * ( (sPannini_ * tan(theta) * (cos(p1_) - sin(p1_) * (1/tan(theta)) * cos(phi - l0_) ) ) - heightLow_ );
		y = heightMax_ - y;		
		if (y < 0) y = 0;
		if (y > heightMax_) y = heightMax_;
	      }
	  }
      }
    
    //STEREOGRAPHIC projection
    if(method_ == STEREOGRAPHIC)
      {
      	double kart[3], polar[3], phi, theta;
	kart[0] = (*it)[2]/100;
	kart[1] = (*it)[0]/-100;
	kart[2] = (*it)[1]/100;
	toPolar(kart, polar);
	//theta == polar[0] == scan [4]
	//phi == polar[1] == scan [5]
	//range == polar[2] == scan [3]
	theta = polar[0] * 180 / M_PI;
	phi = polar[1] * 180 / M_PI;
	range = polar[2];
	//horizantal angle of view of [0:360] and vertical of [-40:60]
	phi = 360.0 - phi;
	phi = phi * 2.0 * M_PI / 360.0;
	theta -= 90;
	theta *= -1;
	theta *= 2.0 * M_PI / 360.0;
	
	for (unsigned int j = 0 ; j < numberOfImages_ ; j++)
	  {
	    iMinX_ = j * interval_;
	    iMaxX_ = (j + 1) * interval_;
	    //check for point in interval_s
	    if(phi <= (iMaxX_) && phi >= (iMinX_))
	      {
		
		//longitude of projection center
		l0_ = iMinX_ + interval_ / 2;
		//use the R variable of stereographic projection mentioned in the thesis
		//finding the min and max of x direction
		k_ = (2 * param_) / (1 + sin(p1_) * sin(p1_) + cos(p1_) * cos(p1_) * cos(iMaxX_ - l0_));
		max_ = k_ * cos(p1_) * sin (iMaxX_ - l0_);
		k_ = (2 * param_) / (1 + sin (p1_) * sin(p1_) + cos(p1_) * cos(p1_) * cos(iMinX_ -l0_));
		min_ = k_ * cos(p1_) * sin (iMinX_ -l0_);
		xFactor_ = (double) (width_ / numberOfImages_) / (max_ - min_);
		double xlow = min_;
		widthMax_ = (width_ / numberOfImages_) - 1;
		//finding the min and max of y direction
		k_ = (2 * param_) / (1 + sin(p1_) * sin(iMaxY_) + cos(p1_) * cos(iMaxY_) * cos(iMaxX_ - l0_));
		max_ = k_ * (cos(p1_) * sin(iMaxY_) - sin(p1_) * cos(iMaxY_) * cos(iMaxX_ - l0_));
		k_ = (2 * param_) / (1 + sin(p1_) * sin(iMinY_) + cos(p1_) * cos(iMinY_) * cos(iMinX_ - l0_));
		min_ = k_ * (cos(p1_) * sin(iMinY_) - sin(p1_) * cos(iMinY_) * cos(iMinX_ - l0_));
		yFactor_ = (double) height_ / (max_ - min_);
		heightLow_ = min_;
		heightMax_ = height_ - 1;
		//project the points and add them to image
		k_ = (2 * param_) / (1 + sin(p1_) * sin(theta) + cos(p1_) * cos(theta) * cos(phi - l0_));

		x = (int) (xFactor_) * (k_ * cos(theta) * sin(phi - l0_) - xlow);
		if (x < 0) x = 0;
		if (x > widthMax_) x = widthMax_;
		x = x + (j * width_ / numberOfImages_);
		
		y = (int) (yFactor_) * (k_ * ( cos(p1_) * sin(theta) - sin(p1_) * cos(theta) * cos(phi - l0_) ) - heightLow_);
		y = heightMax_ - y;
		if (y < 0) y = 0;
		if (y > heightMax_) y = heightMax_;
	      }
	  }
      }
    
    //ZAXIS projection
    if(method_ == ZAXIS)
      {
      	double kart[3], polar[3], phi, theta;
	kart[0] = (*it)[2]/100;
	kart[1] = (*it)[0]/-100;
	kart[2] = (*it)[1]/100;
	toPolar(kart, polar);
	//theta == polar[0] == scan [4]
	//phi == polar[1] == scan [5]
	//range == polar[2] == scan [3]
	theta = polar[0] * 180 / M_PI;
	phi = polar[1] * 180 / M_PI;
	range = polar[2];
	//horizantal angle of view of [0:360] and vertical of [-40:60]
	phi = 360.0 - phi;
	phi = phi * 2.0 * M_PI / 360.0;
	theta -= 90;
	theta *= -1;
	theta *= 2.0 * M_PI / 360.0;

	x = (int) ( xFactor_ * phi);
	if (x < 0) x = 0;
	if (x > widthMax_) x = widthMax_;
	
	///////////////////check this
	y = (int) ( yFactor_ * ((*it)[1] - heightLow_) );
	y = heightMax_ - y;
	if (y < 0) y = 0;
	if (y > heightMax_) y = heightMax_;
      }

    //AZIMUTHAL projection
    if(method_ == AZIMUTHAL)
      {
	double kart[3], polar[3], phi, theta;
	kart[0] = (*it)[2]/100;
	kart[1] = (*it)[0]/-100;
	kart[2] = (*it)[1]/100;
	toPolar(kart, polar);
	//theta == polar[0] == scan [4]
	//phi == polar[1] == scan [5]
	//range == polar[2] == scan [3]
	theta = polar[0] * 180 / M_PI;
	phi = polar[1] * 180 / M_PI;
	range = polar[2];
	//phi == longitude == horizantal angle of view of [0:360] 
	phi = 180.0 - phi;
	phi *= M_PI / 180.0;
	//theta == latitude == vertical angle of view of [-40:60]
	theta -= 90;
	theta *= -1;
	theta *= M_PI / 180.0;

	//calculate kPrime_ according to Mathworld article
	kPrime_ = sqrt(2/(1+sin(phi1_)*sin(theta)+cos(phi1_)*cos(theta)*cos(phi-long0_)));
	
	// add minimum x position as an offset
	x = (int) xFactor_*kPrime_*cos(theta)*sin(phi-long0_);
	x = widthMax_/2 + x;
	if (x < 0) x = 0;
	if (x > widthMax_) x = widthMax_;
        
	// add minimum y position as an offset
	y = (int) yFactor_*kPrime_*(cos(phi1_)*sin(theta)-sin(phi1_)*cos(theta)*cos(phi-long0_));
	y = -y + heightMax_/2;
	if (y < 0) y = 0;
	if (y > heightMax_) y = heightMax_;
      }
  }

  unsigned int projection::getProjectionWidth()
  {
    return width_;
  }

  unsigned int projection::getProjectionHeight()
  {
    return height_;
  }

  projection_method projection::getProjectionMethod()
  {
    return method_;
  }

  unsigned int projection::getProjectionNumberOfImages()
  {
    return numberOfImages_;
  }
  
  double projection::getProjectionParam()
  {
    return param_;
  }

  //private functions
  
  void projection::setImageRatio()
  {
    if((xSize_/ySize_) != ((double)width_/height_))
      {
	cout<<endl;
	cout<<"!!Best Image Ratio (x/y) for this Projection is: "<<xSize_/ySize_<<endl;
	cout<<"width_/height_: "<<(double)width_/height_<<endl;
	if(imageSizeOptimization_ == true)
	  {
	    double tWidth, tHeight;
	    int imageNumber = 1;
	    if(method_ == RECTILINEAR || method_ == PANNINI || method_ == STEREOGRAPHIC)
	      {
		imageNumber = numberOfImages_;
	      }
	    tWidth = imageNumber * height_ * xSize_ / ySize_;
	    tHeight = width_ * ySize_ / xSize_;
	    cout<<"tWidth: "<<tWidth<<endl;
	    cout<<"tHeight: "<<tHeight<<endl;

	    if((double)(width_/height_) >= 1)
	      {
		if((double)(xSize_/ySize_) >= 1)
		  {
		    //height_ stays the same
		    if((double)(tWidth/height_) >= 1)
		      width_ = tWidth;
		    //width_ stays the same
		    else if((double)(width_/tHeight) >= 1)
		      height_ = tHeight;
		  }
	      }
	    else
	      {
		if((double)(xSize_/ySize_) < 1)
		  {
		    //width_ stays the same
		    if((double)(width_/tHeight) <= 1)
		      height_ = tHeight;		
		    //height_ stays the same
		    else if((double)(tWidth/height_) <= 1)
		      width_ = tWidth;		
		  }
	      }
	    cout<<"New Panorama Size is: "<<width_<<"X"<<height_<<endl;
	    cout<<endl;
	  }
      } 
  }
  
}
