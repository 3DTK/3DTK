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
 * @author Hamidreza Houshiar, Jacobs University Bremen, Germany
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
    init(3600, 1000, EQUIRECTANGULAR, 1, 0, 0, 0, 0, 360, -40, 60, false);
  }

  projection::projection(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, double minZ, double maxZ, double minHorizAngle, double maxHorizAngle, double minVertAngle, double maxVertAngle, bool imageSizeOptimization)
  {
    init(width, height, method, numberOfImages, param, minZ, maxZ, minHorizAngle, maxHorizAngle, minVertAngle, maxVertAngle, imageSizeOptimization);
  }

  void  projection::init(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, double minZ, double maxZ, double minHorizAngle, double maxHorizAngle, double minVertAngle, double maxVertAngle, bool imageSizeOptimization)
  {
    //set the projection params
    //projection size
    width_ = width;
    height_ = height;
    //optimization of projection size
    imageSizeOptimization_ = imageSizeOptimization;
    //min and max of Z from user for zaxis projection
    minZ_ = minZ;
    maxZ_ = maxZ;
    //projction method
    method_ = method;
    //number of images for projection
    numberOfImages_ = numberOfImages;
    //projection specific param
    param_ = param;

    //scanner==projection max and min angle
    //change the max, min of horizontal and vertical from degree to rad    
    maxHorizAngle_ = maxHorizAngle / 360.0 * 2.0 * M_PI;
    minHorizAngle_ = minHorizAngle / 360.0 * 2.0 * M_PI;
    maxVertAngle_ = maxVertAngle / 360.0 * 2.0 * M_PI;
    minVertAngle_ = minVertAngle / 360.0 * 2.0 * M_PI;
    
    //based on projection method init the projection
    //EQUIRECTANGULAR projection
    if(method_ == EQUIRECTANGULAR)
      {
	//adding the longitude to x axis and latitude to y axis
	xSize_ = maxHorizAngle_ - minHorizAngle_;
	ySize_ = maxVertAngle_ - minVertAngle_;

	setImageRatio();

	xFactor_ = (double) width_ / xSize_;
	widthMax_ = width_ - 1;
	yFactor_ = (double) height_ / ySize_;      
	//shift all the valuse to positive points on image 
	heightLow_ = minVertAngle_;
	heightMax_ = height_ - 1;	
      }
    
    //CONIC projection
    if(method_ == CONIC)
      {	
	// set up initial parameters according to MathWorld: http://mathworld.wolfram.com/AlbersEqual-AreaConicProjection.html
	lat0_ = (minVertAngle_ + maxVertAngle_) / 2;
	long0_ = (minHorizAngle_ + maxHorizAngle_) / 2;
	phi1_ = minVertAngle_;
	phi2_ = maxVertAngle_;
	n_ = (sin(phi1_) + sin(phi2_)) / 2.;
	c_ = sqr(cos(phi1_)) + 2 * n_ * sin(phi1_);
	rho0_ = sqrt(c_ - 2 * n_ * sin(lat0_)) / n_;
	// set up max values for x and y and add the longitude to x axis and latitude to y axis
	xMax_ = (1./n_ * sqrt(c_ - 2*n_*sin( minVertAngle_ )) ) * sin(n_ * (maxHorizAngle_ - long0_));
	xMin_ = (1./n_ * sqrt(c_ - 2*n_*sin( minVertAngle_ )) ) * sin(n_ * (minHorizAngle_ - long0_));
	xSize_ =  ( xMax_ - xMin_ );

	yMax_ = rho0_ - (1./n_ * sqrt(c_ - 2*n_*sin(maxVertAngle_)) ) * cos(n_ * (maxHorizAngle_ - long0_ ));
	yMin_ = rho0_ - (1./n_ * sqrt(c_ - 2*n_*sin(minVertAngle_)) ) * cos(n_ * ((minHorizAngle_ + maxHorizAngle_)/2 - long0_ ));
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
	xSize_ = maxHorizAngle_ - minHorizAngle_;
	ySize_ = tan(maxVertAngle_) - tan(minVertAngle_);
      
	setImageRatio();	
	
	//find the x and y range
	xFactor_ = (double) width_ / xSize_;
	widthMax_ = width_ - 1;
	yFactor_ = (double) height_ / ySize_;
	heightLow_ = minVertAngle_;
	heightMax_ = height_ - 1;
      }

    //EQUALAREACYLINDRICAL projection
    if(method_ == EQUALAREACYLINDRICAL)
      {
	//change the param==phi_s to rad
	param_ = param_ / 360.0 * 2 * M_PI;
	xSize_ =  (maxHorizAngle_ - minHorizAngle_) * cos(param_);
	ySize_ =  ((sin(maxVertAngle_) - sin(minVertAngle_)) / cos(param_));
      
	setImageRatio();
      
	//find the x and y range
	xFactor_ = (double) width_ / xSize_;
	widthMax_ = width_ - 1;
	yFactor_ = (double) height_ / ySize_;
	heightLow_ = minVertAngle_;
	heightMax_ = height_ - 1;
      }

    //Mercator Projection
    if(method_ == MERCATOR)
      {
	//find the x and y range
	xSize_ = maxHorizAngle_ - minHorizAngle_;
	ySize_ =  ( log( tan( maxVertAngle_) + ( 1 / cos( maxVertAngle_) ) ) - log ( tan( minVertAngle_) + (1 / cos(minVertAngle_) ) ) );
      
	setImageRatio();
      
	xFactor_ = (double) width_ / xSize_;
	widthMax_ = width_ - 1;
	yFactor_ = (double) height_ / ySize_;
	heightLow_ = log(tan(minVertAngle_) + (1/cos(minVertAngle_)));
	heightMax_ = height_ - 1;
      }

    //RECTILINEAR projection
    if(method_ == RECTILINEAR)
      {
	//default value for numberOfImages_
	if(numberOfImages_ < 3) numberOfImages_ = 3;
	cout<<"Number of images per scan is: "<<numberOfImages_<<endl;
	interval_ = (maxHorizAngle_ - minHorizAngle_) / numberOfImages_;
	iMinY_ = minVertAngle_;
	iMaxY_ = maxVertAngle_;

	//latitude of projection center
	p1_ = 0;

	iMinX_ = minHorizAngle_ + (0 * interval_);
	iMaxX_ = minHorizAngle_ + ((0 + 1) * interval_);
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
	interval_ = (maxHorizAngle_ - minHorizAngle_) / numberOfImages_;
	iMinY_ = minVertAngle_;
	iMaxY_ = maxVertAngle_;
	//latitude of projection center
	p1_ = 0;
            
	iMinX_ = minHorizAngle_ + (0 * interval_);
	iMaxX_ = minHorizAngle_ + ((0 + 1) * interval_);
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
	interval_ = (maxHorizAngle_ - minHorizAngle_) / numberOfImages_;
	iMinY_ = minVertAngle_;
	iMaxY_ = maxVertAngle_;
	//latitude of projection center
	p1_ = 0;
      
	iMinX_ = minHorizAngle_ + (0 * interval_);
	iMaxX_ = minHorizAngle_ + ((0 + 1) * interval_);
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
	xSize_ = maxHorizAngle_ - minHorizAngle_;
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
	// set up initial parameters according to MathWorld: http://mathworld.wolfram.com/LambertAzimuthalEqual-AreaProjection.html
	long0_ = (minHorizAngle_ + maxHorizAngle_) / 2;
	phi1_ = (minVertAngle_ + maxVertAngle_) / 2;

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
    if(rangeImage.cols != width_ || rangeImage.rows != height_)
      {
	cout<<"rnage image size is different from input size."<<endl;
	cout<<"init the class with new params."<<endl;
	init(rangeImage.cols, rangeImage.rows, method_, numberOfImages_, param_, minZ_, maxZ_, minHorizAngle_, maxHorizAngle_, minVertAngle_, maxVertAngle_, imageSizeOptimization_);
	//return;
      }

    if (rangeImage.cols != reflectanceImage.cols
	|| rangeImage.rows != reflectanceImage.rows) 
      {
	cerr << "range image and reflectance image have different geometries - using empty reflectance image" << endl;
	reflectanceImage.create(rangeImage.size(), CV_8U);
	reflectanceImage = cv::Scalar::all(0);
      }

    for (int row = 0; row < rangeImage.size().height; ++row) 
      {
	for (int col = 0; col < rangeImage.size().width; ++col) 
	  {
	    double range,reflectance, x, y, z;
	    range = rangeImage.at<float>(row, col);
	    reflectance = reflectanceImage.at<uchar>(row,col)/255.0;
	    
	    calcPointFromPanoramaPosition(x, y, z, row, col, range);
	    
	    if( x < 1e-5 && y < 1e-5 && z < 1e-5) 
	      {
		continue;
	      }
	    reducedPoints.push_back(cv::Vec4f(x, y, z, reflectance));
	  }
      }
  }

  void projection::calcPointFromPanoramaPosition(double& x, double& y, double& z, int row, int col, double range)
  {
    double theta, phi;
    //get the theta and phi based on the projection 
    if(method_ == EQUIRECTANGULAR)
      {
	theta = (heightMax_ - row + 0.5) / yFactor_ + heightLow_; 
	phi = (col + 0.5 ) / xFactor_;
      }
    if(method_ == CYLINDRICAL)
      {
	theta = atan2(row + 0.5 + yFactor_ * tan(heightLow_), yFactor_);
	phi = (col + 0.5) / xFactor_; 
      }
    if(method_ == MERCATOR)
      {
	theta = 2 * atan2(exp((heightMax_ - row + 0.5) / yFactor_ + heightLow_), 1.) - M_PI_2;
	phi = (col + 0.5) / xFactor_;
      }
    if(method_ == CONIC)
      {
	float X = col * 1. / xFactor_ - fabs(xMin_);
	float Y = (heightMax_ - row) * 1. / yFactor_ - fabs(yMin_);
	theta = asin( (c_ - (X*X + (rho0_ - Y) * (rho0_ - Y)) * n_ * n_) / (2 * n_) );
	phi = long0_ + (1./n_) * ::atan2(X, (float)rho0_ - Y);
      }    
    //other projections
    
    phi = (2 * M_PI) - phi;
    theta *= -1;
    theta += M_PI/2.0;
    
    
    double polar[3] = { theta, phi, range }, cartesian[3] = {0., 0., 0.}; 
    toCartesian(polar, cartesian);

    x = -100.0 * cartesian[1];
    y = 100.0 * cartesian[2];
    z = 100.0 * cartesian[0];
  }
  
  void projection::calcPanoramaPositionForAPoint(int &x, int &y, cv::MatIterator_<cv::Vec4f> it, double &range)
  {
	double kart[3], polar[3], phi, theta;
	//get the x,y,z in right handed coordinate system in meters
	kart[0] = (*it)[2]/100;
	kart[1] = (*it)[0]/-100;
	kart[2] = (*it)[1]/100;
	//get the polar coordinte of x,y,z this is in rad
	if(kart[0] != 0 && kart[1] != 0 && kart[2] != 0)
	  toPolar(kart, polar);
	else
	  {
	    x = 0;
	    y = 0;
	    return;
	  }
	  
	//theta == polar[0] == scan [4]
	//phi == polar[1] == scan [5]
	//range == polar[2] == scan [3]
	theta = polar[0];
	phi = polar[1];
	range = polar[2];
	//horizantal angle of view of [0:360][minHorzAngle_:maxHorzAngle_] and vertical of [-40:60][minVertAngle_:maxVertAngle]
        //phi == longitude == horizantal angle of view of [0:360] 
	//shift it to clockwise instead of counter clockwise 
	phi = (2 * M_PI) - phi;
	//theta == latitude == vertical angle of view of [-40:60]
	//shift the vertical angle instead of -90:90 to 0:180 from north to south pole
	theta -= M_PI/2.0;
	theta *= -1;
    
    //EQUIRECTANGULAR projection
    if(method_ == EQUIRECTANGULAR)
      {
	x = (int) ( xFactor_ * phi);
	if (x < 0) x = 0;
	if (x > widthMax_) x = widthMax_;
	y = (int) ( yFactor_ * (theta - heightLow_) );
	y = heightMax_ - y;
	if (y < 0) y = 0;
	if (y > heightMax_) y = heightMax_;	
      }

    //CONIC projection
    if(method_ == CONIC)
      {
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
	x = (int) ( xFactor_ * (phi*cos(param_)));
	if (x < 0) x = 0;
	if (x > widthMax_) x = widthMax_;
	y = (int) ((double) yFactor_ * ((sin(theta) - sin(heightLow_)) / cos(param_)));
	y = heightMax_ - y;
	if (y < 0) y = 0;
	if (y > heightMax_) y = heightMax_;
      }
    
    //Mercator Projection
    if( method_ == MERCATOR)
      {
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
	for(unsigned int j = 0 ; j < numberOfImages_ ; j++)
	  {
	    iMinX_ = minHorizAngle_ + (j * interval_);
	    iMaxX_ = minHorizAngle_ + ((j + 1) * interval_);
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
	for(unsigned int i = 0 ; i < numberOfImages_ ; i++)
	  {
	    iMinX_ = minHorizAngle_ + (i * interval_);
	    iMaxX_ = minHorizAngle_ + ((i + 1) * interval_);
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
	for (unsigned int j = 0 ; j < numberOfImages_ ; j++)
	  {
	    iMinX_ = minHorizAngle_ + (j * interval_);
	    iMaxX_ = minHorizAngle_ + ((j + 1) * interval_);
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
	if(imageSizeOptimization_ == true)
	  {
	    cout<<endl;
	    cout<<"!!Best Image Ratio (x/y) for this Projection is: "<<xSize_/ySize_<<endl;
	    cout<<"width_/height_: "<<(double)width_/height_<<endl;

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
