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
 * @author HamidReza Houshiar, Jacobs University Bremen, Germany
 * @author Andreas Nuechter. University of Wuerzburg, Germany.
 * @author Julia Kauer. University of Wuerzburg, Germany.
 * @author Lukas Gradl. University of Wuerzburg, Germany.
 */

#include "slam6d/fbr/panorama.h"

using namespace std;

namespace fbr
{
  
  void panorama::init(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod, float min, float max, double MINANGLE, double MAXANGLE, bool imageOptimization)
  {
    
    //optimization of panorama size
    iOptimization = imageOptimization;
    //scanner max and min angle
    MAX_ANGLE = MAXANGLE;
    MIN_ANGLE = MINANGLE;
    //z Min and Max
    zMin = min;
    zMax = max;
    //clear the containers
    iReflectance.release();
    iMap.release();
    iRange.release();
    iColor.release();
    extendedIMap.clear();
    //set the data
    iWidth = width;
    iHeight = height;
    pMethod = method;
    nImages = numberOfImages;
    pParam = param;
    maxRange = 0;
    mapMethod = mMethod;

    //EQUIRECTANGULAR projection
    if(pMethod == EQUIRECTANGULAR)
      {
	//adding the longitude to x axis and latitude to y axis
	xSize = 2 * M_PI;
	ySize =  ((MAX_ANGLE - MIN_ANGLE) / 360.0 * 2.0 * M_PI);
	
	setImageRatio(xSize, ySize);

	xFactor = (double) iWidth / xSize;
	widthMax = iWidth - 1;
	yFactor = (double) iHeight / ySize;      
	//shift all the valuse to positive points on image 
	heightLow =(0.0 - MIN_ANGLE) / 360.0 * 2.0 * M_PI;
	heightMax = iHeight - 1;
	
      }
    
    //CONIC projection
    if(pMethod == CONIC)
      {
	// set up maximum latitude and longitude angles of the robot 
	MIN_VERT_ANGLE = MIN_ANGLE * M_PI / 180.0; 
	MAX_VERT_ANGLE = MAX_ANGLE * M_PI / 180.0;
	MIN_HORIZ_ANGLE = -M_PI; 
	MAX_HORIZ_ANGLE = M_PI;
	// set up initial parameters according to MathWorld: http://mathworld.wolfram.com/AlbersEqual-AreaConicProjection.html
	Lat0 = 0.; 
	Long0 = 0.;
	Phi1 = -40. * M_PI / 180.0; 
	Phi2 = 60 * M_PI / 180.0;
	n = (sin(Phi1) + sin(Phi2)) / 2.;
	C = sqr(cos(Phi1)) + 2 * n * sin(Phi1);
	Rho0 = sqrt(C - 2 * n * sin(Lat0)) / n;
	// set up max values for x and y and add the longitude to x axis and latitude to y axis
	xmax = (1./n * sqrt(C - 2*n*sin( MIN_VERT_ANGLE )) ) * sin(n * (MAX_HORIZ_ANGLE - Long0));
	xmin = (1./n * sqrt(C - 2*n*sin( MIN_VERT_ANGLE )) ) * sin(n * (MIN_HORIZ_ANGLE - Long0));
	xSize =  ( xmax - xmin );
	
	ymin = Rho0 - (1./n * sqrt(C - 2*n*sin(MIN_VERT_ANGLE)) ) * cos(n * ( 0. - Long0 ));
	ymax = Rho0 - (1./n * sqrt(C - 2*n*sin(MAX_VERT_ANGLE)) ) * cos(n * (MAX_HORIZ_ANGLE - Long0 ));
	ySize =  ( ymax - ymin );
	
	setImageRatio(xSize, ySize);
	
	xFactor = (double) iWidth / xSize;
	widthMax = iWidth - 1;
	yFactor = (double) iHeight / ySize;
	//shift all the values to positive points on image 
	heightMax = iHeight - 1;	
      }
    
    //CYLINDRICAL projection
    if(pMethod == CYLINDRICAL)
      {
	//adding the longitude to x and tan(latitude) to y
	xSize = 2 * M_PI;
	ySize = (tan(MAX_ANGLE / 360 * 2 * M_PI) - tan(MIN_ANGLE / 360 * 2 * M_PI));
      
	setImageRatio(xSize, ySize);	
	
	//find the x and y range
	xFactor = (double) iWidth / xSize;
	widthMax = iWidth - 1;
	yFactor = (double) iHeight / ySize;
	heightLow = (MIN_ANGLE) / 360 * 2 * M_PI;
	heightMax = iHeight - 1;
      }

    //EQUALAREACYLINDRICAL projection
    if(pMethod == EQUALAREACYLINDRICAL)
      {
	xSize =  (2 * M_PI * cos(pParam / 360 * 2 * M_PI));
	ySize =  ((sin(MAX_ANGLE / 360 * 2 * M_PI) - sin(MIN_ANGLE / 360 * 2 * M_PI)) / cos(pParam / 360 * 2 * M_PI));
      
	setImageRatio(xSize, ySize);
      
	//find the x and y range
	xFactor = (double) iWidth / xSize;
	widthMax = iWidth - 1;
	yFactor = (double) iHeight / ySize;
	heightLow = (MIN_ANGLE) / 360 * 2 * M_PI;
	heightMax = iHeight - 1;
      }

    //Mercator Projection
    if( pMethod == MERCATOR)
      {
	//find the x and y range
	xSize = 2 * M_PI;
	ySize =  ( log( tan( MAX_ANGLE / 360 * 2 * M_PI ) + ( 1 / cos( MAX_ANGLE / 360 * 2 * M_PI ) ) ) - log ( tan( MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI) ) ) );
      
	setImageRatio(xSize, ySize);
      
	xFactor = (double) iWidth / xSize;
	widthMax = iWidth - 1;
	yFactor = (double) iHeight / ySize;
	heightLow = log(tan(MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI)));
	heightMax = iHeight - 1;
      }

    //RECTILINEAR projection
    if(pMethod == RECTILINEAR)
      {
	//default value for nImages
	if(nImages < 3) nImages = 3;
	cout<<"Number of images per scan is: "<<nImages<<endl;
	interval = 2 * M_PI / nImages;
	//iMiny = -M_PI/9;
	//iMaxy = 2*M_PI/9;
	iMiny = MIN_ANGLE * M_PI / 180;
	iMaxy = MAX_ANGLE * M_PI / 180;

	//latitude of projection center
	p1 = 0;

	iMinx = 0 * interval;
	iMaxx = (0 + 1) * interval;
	//the longitude of projection center
	l0 = iMinx + interval / 2;
	//finding the min and max of the x direction
	coscRectilinear = sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0);
	max = (cos(iMaxy) * sin(iMaxx - l0) / coscRectilinear);
	coscRectilinear = sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0);
	min = (cos(iMiny) * sin(iMinx - l0) / coscRectilinear);
	xSize =  (max - min);
	//finding the min and max of y direction
	coscRectilinear = sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0);
	max = ( (cos(p1) * sin(iMaxy) - sin(p1) * cos(iMaxy) * cos(iMaxx - l0) )/ coscRectilinear);
	coscRectilinear = sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0);
	min = ( (cos(p1) * sin(iMiny) - sin(p1) * cos(iMiny) * cos(iMinx - l0) )/ coscRectilinear);
	ySize = (max - min);

	setImageRatio(xSize, ySize);
      }

    //PANNINI projection
    if(pMethod == PANNINI)
      {

	//default values for nImages and dPannini==pParam
	if(pParam == 0) pParam = 1;
	if(nImages < 2) nImages = 2;
	cout << "Parameter d is: " << pParam <<", Horizontal Number of images per scan is: " << nImages << endl;
	interval = 2 * M_PI / nImages;
	//iMiny = -M_PI/9;
	//iMaxy = 2*M_PI/9;
	iMiny = MIN_ANGLE * M_PI / 180;
	iMaxy = MAX_ANGLE * M_PI / 180;
	//latitude of projection center
	p1 = 0;
            
	iMinx = 0 * interval;
	iMaxx = (0 + 1) * interval;
	//the longitude of projection center
	l0 = iMinx + interval / 2;
      
	//use the S variable of pannini projection mentioned in the thesis
	//finding the min and max of the x direction
	sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMaxy) + cos(p1) * cos(iMaxx - l0));
	max = sPannini * (sin(iMaxx - l0));
	sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMiny) + cos(p1) * cos(iMinx - l0));
	min = sPannini * (sin(iMinx - l0));
	xSize = max - min;
	//finding the min and max of y direction
	sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMaxy) + cos(p1) * cos(iMaxx - l0));
	max = sPannini * (tan(iMaxy) * (cos(p1) - sin(p1) * 1/tan(iMaxy) * cos(iMaxx - l0)));
	sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMiny) + cos(p1) * cos(iMinx - l0));
	min = sPannini * (tan(iMiny) * (cos(p1) - sin(p1) * 1/tan(iMiny) * cos(iMinx - l0)));
	ySize = max - min;
      
	setImageRatio(xSize, ySize);
      }

    //STEREOGRAPHIC projection
    if(pMethod == STEREOGRAPHIC)
      {
	//default values for nImages and rStereographic==pParam
	if(pParam == 0) pParam = 2;
	if(nImages < 2) nImages = 2;
	cout << "Paremeter R is:" << pParam << ", Number of images per scan is:" << nImages << endl;
	// l0 and p1 are the center of projection iminx, imaxx, iminy, imaxy are the bounderis of intervals
	interval = 2 * M_PI / nImages;
	//iMiny = -M_PI/9;
	//iMaxy = 2*M_PI/9;
	iMiny = MIN_ANGLE * M_PI / 180;
	iMaxy = MAX_ANGLE * M_PI / 180;
	//latitude of projection center
	p1 = 0;
      
	iMinx = 0 * interval;
	iMaxx = (0 + 1) * interval;
	//longitude of projection center
	l0 = iMinx + interval / 2;
	//use the R variable of stereographic projection mentioned in the thesis
	//finding the min and max of x direction
	k = (2 * pParam) / (1 + sin(p1) * sin(p1) + cos(p1) * cos(p1) * cos(iMaxx - l0));
	max = k * cos(p1) * sin (iMaxx - l0);
	k = (2 * pParam) / (1 + sin (p1) * sin(p1) + cos(p1) * cos(p1) * cos(iMinx -l0));
	min = k * cos(p1) * sin (iMinx -l0);
	xSize =  (max - min);
	//finding the min and max of y direction
	k = (2 * pParam) / (1 + sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0));
	max = k * (cos(p1) * sin(iMaxy) - sin(p1) * cos(iMaxy) * cos(iMaxx - l0));
	k = (2 * pParam) / (1 + sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0));
	min = k * (cos(p1) * sin(iMiny) - sin(p1) * cos(iMiny) * cos(iMinx - l0));
	ySize = (max - min);

	setImageRatio(xSize, ySize);
      }

    //ZAXIS projection
    if(pMethod == ZAXIS)
      {
	//double zMin = -100;
	//double zMax = 100;
	//adding the longitude to x axis and latitude to y axis
	xSize = 2 * M_PI;
	ySize = (zMax - zMin);

	setImageRatio(xSize, ySize);

	xFactor = (double) iWidth / xSize;
	widthMax = iWidth - 1;
	cout << "ZMAX= " << zMax << " ZMIN= "<< zMin << endl;
	yFactor = (double) iHeight / ySize;
	//shift all the valuse to positive points on image 
	heightLow = zMin;
	heightMax = iHeight - 1;
      }
    
    //AZIMUTHAL projection
    if(pMethod == AZIMUTHAL)
      {
	// set up maximum latitude and longitude angles of the robot 
	MIN_VERT_ANGLE = MIN_ANGLE * M_PI / 180.0;
	MAX_VERT_ANGLE = MAX_ANGLE * M_PI / 180.0;
	MIN_HORIZ_ANGLE = -M_PI; 
	MAX_HORIZ_ANGLE = M_PI;
	// set up initial parameters according to MathWorld: http://mathworld.wolfram.com/LambertAzimuthalEqual-AreaProjection.html
	Long0 = 0.;
	Phi1 = 0. * M_PI / 180.0;

	// set up max values for x and y and add the longitude to x axis and latitude to y axis
	// sqrt(2/(1+sin(Phi1)*sin(theta)+cos(Phi1)*cos(theta)*cos(phi-Long0)));
	xmax =  sqrt(2/(1+sin(Phi1)*sin(MAX_HORIZ_ANGLE)+cos(Phi1)*cos(MAX_HORIZ_ANGLE)*cos((MIN_VERT_ANGLE/2+MAX_VERT_ANGLE/2)-Long0)))*cos(MAX_HORIZ_ANGLE)*sin((MIN_VERT_ANGLE/2+MAX_VERT_ANGLE/2)-Long0);
	xmin = sqrt(2/(1+sin(Phi1)*sin(MIN_HORIZ_ANGLE)+cos(Phi1)*cos(MIN_HORIZ_ANGLE)*cos((MIN_VERT_ANGLE/2+MAX_VERT_ANGLE/2)-Long0)))*cos(MIN_HORIZ_ANGLE)*sin((MIN_VERT_ANGLE/2+MAX_VERT_ANGLE/2)-Long0);
	xSize =  ( xmax - xmin );

	ymin = xmin;     // the thing is supposed to be circular, isn't it?
	ymax = xmax;
	ySize =  ( ymax - ymin );
      
	setImageRatio(xSize, ySize);

	xSize=1/2.;
	ySize=1/2.;
	
	xFactor = (double) iWidth / 2*xSize;
	widthMax = iWidth - 1;
	yFactor = (double) iHeight / 2*ySize;
	//shift all the values to positive points on image 
	heightMax = iHeight - 1;
      }

  }
  
  void panorama::clear()
  {
    //clear the containers
    iReflectance.release();
    iMap.release();
    iRange.release();
    iColor.release();
    extendedIMap.clear();
  }

  panorama::panorama()
  {
    init(3600, 1000, EQUIRECTANGULAR, 1, 0, FARTHEST);
  }

  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod, float min, float max, double MINANGLE, double MAXANGLE, bool imageOptimization)
  { 
    init(width, height, method, numberOfImages, param, mMethod, min, max, MINANGLE, MAXANGLE, imageOptimization);
  }

  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod, float min, float max, double MINANGLE, double MAXANGLE)
  { 
    init(width, height, method, numberOfImages, param, mMethod, min, max, MINANGLE, MAXANGLE);
  }

  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod)
  { 
    init(width, height, method, numberOfImages, param, mMethod);
  }
  
  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param)
  { 
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
  
  void panorama::map(int x, int y, cv::MatIterator_<cv::Vec4f> it, double range)
  {
    
    if (maxRange < (float)range)
      maxRange = (float)range;

    // adding the point with max distance
    if (mapMethod == FARTHEST) 
      {
	if (iRange.at<float>(y,x) < range) 
	  {
	    // reflectance
	    iReflectance.at<uchar>(y,x) = (*it)[3]*255;
	
	    // range
	    iRange.at<float>(y,x) = (float)range;

	    iMap.at<cv::Vec3f>(y,x)[0] = (*it)[0]; // x
	    iMap.at<cv::Vec3f>(y,x)[1] = (*it)[1]; // y
	    iMap.at<cv::Vec3f>(y,x)[2] = (*it)[2]; // z
	  }
      } 
    else if(mapMethod == EXTENDED)
      {
	// reflectance
	iReflectance.at<uchar>(y,x) = (*it)[3]*255;
      
	// range
	iRange.at<float>(y,x) = (float)range;
      
	// adding all the points
	cv::Vec3f point;
	point[0] = (*it)[0]; // x
	point[1] = (*it)[1]; // y
	point[2] = (*it)[2]; // z
	extendedIMap[y][x].push_back(point);
      }
  }

  void panorama::mapColor(int x, int y, cv::MatIterator_<cv::Vec3f> itColor)
  {
    // rgb
    iColor.at<cv::Vec3f>(y,x)[0] = (*itColor)[0];//r
    iColor.at<cv::Vec3f>(y,x)[1] = (*itColor)[1];//g
    iColor.at<cv::Vec3f>(y,x)[2] = (*itColor)[2];//b
  }
  
  void panorama::initMat()
  {
    if(mapMethod == FARTHEST)
      {
	iMap.create(iHeight, iWidth, CV_32FC(3));
	iMap = cv::Scalar::all(0);
      }
    else if(mapMethod == EXTENDED)
      {
	extendedIMap.resize(iHeight);
	for (unsigned int i = 0; i < iHeight; i++)
	  extendedIMap[i].resize(iWidth);
      }
    iReflectance.create(iHeight, iWidth, CV_8U);
    iReflectance = cv::Scalar::all(0);
    iRange.create(iHeight, iWidth, CV_32FC(1));
    iRange = cv::Scalar::all(0);
    iColor.create(iHeight, iWidth, CV_32FC(3));
    iColor = cv::Scalar::all(0);
  }

  void panorama::setImageRatio(double xSize, double ySize)
  {
    if((xSize/ySize) != ((double)iWidth/iHeight))
      {
	cout<<endl;
	cout<<"!!Best Image Ratio (x/y) for this Projection is: "<<xSize/ySize<<endl;
	cout<<"iWidth/iHeight: "<<(double)iWidth/iHeight<<endl;
	if(iOptimization == true)
	  {
	    double tWidth, tHeight;
	    int imageNumber = 1;
	    if(pMethod == RECTILINEAR || pMethod == PANNINI || pMethod == STEREOGRAPHIC)
	      {
		imageNumber = nImages;
	      }
	    tWidth = imageNumber * iHeight * xSize / ySize;
	    tHeight = iWidth * ySize / xSize;
	    cout<<"tWidth: "<<tWidth<<endl;
	    cout<<"tHeight: "<<tHeight<<endl;

	    if((double)(iWidth/iHeight) >= 1)
	      {
		if((double)(xSize/ySize) >= 1)
		  {
		    //iHeight stays the same
		    if((double)(tWidth/iHeight) >= 1)
		      iWidth = tWidth;
		    //iWidth stays the same
		    else if((double)(iWidth/tHeight) >= 1)
		      iHeight = tHeight;
		  }
	      }
	    else
	      {
		if((double)(xSize/ySize) < 1)
		  {
		    //iWidth stays the same
		    if((double)(iWidth/tHeight) <= 1)
		      iHeight = tHeight;		
		    //iHeight stays the same
		    else if((double)(tWidth/iHeight) <= 1)
		      iWidth = tWidth;		
		  }
	      }
	    cout<<"New Panorama Size is: "<<iWidth<<"X"<<iHeight<<endl;
	    cout<<endl;
	  }
      } 
  }

  void panorama::createPanorama(cv::Mat scan, cv::Mat color)
  {

    initMat();      	

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
	calcPanoramaPositionForAPoint(x, y, it, range);
	    
	//create the iReflectance iRange and map
	map(x, y, it, range);
	//create the iColor
	if(color.empty() == false)
	  {
	    mapColor(x, y, itColor);
	    ++itColor;
	  }  
      }

  }

  void panorama::createPanorama(cv::Mat scan) 
  {
    cv::Mat color;
    createPanorama(scan, color);
  }
  
  void panorama::createPanoramaFromOctree(PointType pointtype_, scanner_type sType_, double minReflectance_, double maxReflectance_)
  {
    initMat();
    
    pointtype = pointtype_;
    sType = sType_;
    minReflectance = minReflectance_;
    maxReflectance = maxReflectance_;
  }
  
  template <class T>
  void panorama::accept(T *point)
  {
    float x, y, z, reflectance, red, green, blue;
    x = point[0];
    y = point[1];
    z = point[2];

    if(pointtype.hasReflectance())
      {
	int idx = pointtype.getReflectance();
	reflectance = point[idx];
      }
    else
      reflectance = 255;

    //normalize the reflectance
    reflectance = fbr::normalizeTheReflectance(reflectance, sType, minReflectance, maxReflectance);

    if(pointtype.hasColor())
      {
        int idx = pointtype.getColor();
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
    if(iReflectance.empty() == false)
      {
	cv::Mat point(1,1,CV_32FC(4),cv::Scalar(x,y,z,reflectance));
	cv::Mat pointColor(1,1,CV_32FC(3),cv::Scalar(r,g,b));
	
	cv::MatIterator_<cv::Vec4f> it;
	it = point.begin<cv::Vec4f>();
	
	cv::MatIterator_<cv::Vec3f> itColor;
	itColor = pointColor.begin<cv::Vec3f>();
	
	int x, y;
	double range;
	calcPanoramaPositionForAPoint(x, y, it, range);
	
	//create the iReflectance iRange and map
	map(x, y, it, range);
	//create the iColor
	mapColor(x, y, itColor);
      }
    else
      {
	cout<<"Panorma has not been initialized!";
      }
  }

  void panorama::calcPanoramaPositionForAPoint(int &x, int &y, cv::MatIterator_<cv::Vec4f> it, double &range)
  {

    //EQUIRECTANGULAR projection
    if(pMethod == EQUIRECTANGULAR)
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
	
	x = (int) ( xFactor * phi);
	if (x < 0) x = 0;
	if (x > widthMax) x = widthMax;
	y = (int) ( yFactor * (theta + heightLow) );
	y = heightMax - y;
	if (y < 0) y = 0;
	if (y > heightMax) y = heightMax;	
      }

    //CONIC projection
    if(pMethod == CONIC)
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
        x = (int) ( xFactor * (sqrt(C - 2 * n * sin( theta) ) / n * sin(n * (phi - Long0)) + fabs(xmin) ) );
        if (x < 0) x = 0;
        if (x > widthMax) x = widthMax;
        
        // add minimum y position as an offset
        y = (int) ( yFactor * (Rho0 - (1/n * sqrt(C - 2 * n * sin( theta) ) ) * cos(n * (phi - Long0)) + fabs( ymin ) ) );
        y = heightMax - y;
        if (y < 0) y = 0;
        if (y > heightMax) y = heightMax;
      }
    
    //CYLINDRICAL projection
    if(pMethod == CYLINDRICAL)
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
	
	x = (int) ( xFactor * phi);
	if (x < 0) x = 0;
	if (x > widthMax) x = widthMax;
	y = (int) ((double) yFactor * (tan(theta) - tan(heightLow)));
	y = heightMax - y;
	if (y < 0) y = 0;
	if (y > heightMax) y = heightMax;	
      }



    //EQUALAREACYLINDRICAL projection
    if(pMethod == EQUALAREACYLINDRICAL)
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

	x = (int) ( xFactor * (phi*cos(pParam/360*2*M_PI)));
	if (x < 0) x = 0;
	if (x > widthMax) x = widthMax;
	y = (int) ((double) yFactor * ((sin(theta) - sin(heightLow)) / cos(pParam/360*2*M_PI)));
	y = heightMax - y;
	if (y < 0) y = 0;
	if (y > heightMax) y = heightMax;
      }

    
    //Mercator Projection
    if( pMethod == MERCATOR)
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

	x = (int) ( xFactor * phi);
	if (x < 0) x = 0;
	if (x > widthMax) x = widthMax;
	y = (int) ( yFactor * (log(tan(theta) + (1/cos(theta))) - heightLow) );
	y = heightMax - y;
	if (y < 0) y = 0;
	if (y > heightMax) y = heightMax;	
      }
    
    //RECTILINEAR projection
    if(pMethod == RECTILINEAR)
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
	
	for(unsigned int j = 0 ; j < nImages ; j++)
	  {
	    iMinx = j * interval;
	    iMaxx = (j + 1) * interval;
	    //check for point in interval
	    if(phi <= iMaxx && phi >= iMinx)
	      {
		
		//the longitude of projection center
		l0 = iMinx + interval / 2;
		//finding the min and max of the x direction
		coscRectilinear = sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0);
		max = (cos(iMaxy) * sin(iMaxx - l0) / coscRectilinear);
		coscRectilinear = sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0);
		min = (cos(iMiny) * sin(iMinx - l0) / coscRectilinear);
		xFactor = (double) (iWidth / nImages) / (max - min);
		double xlow = min;
		widthMax = (iWidth / nImages) - 1;
		//finding the min and max of y direction
		coscRectilinear = sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0);
		max = ( (cos(p1) * sin(iMaxy) - sin(p1) * cos(iMaxy) * cos(iMaxx - l0) )/ coscRectilinear);
		coscRectilinear = sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0);
		min = ( (cos(p1) * sin(iMiny) - sin(p1) * cos(iMiny) * cos(iMinx - l0) )/ coscRectilinear);
		yFactor = (double) iHeight / (max - min);
		heightLow = min;
		heightMax = iHeight - 1;
		//project the points and add them to image
		coscRectilinear = sin(p1) * sin(theta) + cos(p1) * cos(theta) * cos(phi - l0);
		
		x = (int)(xFactor) * ((cos(theta) * sin(phi - l0) / coscRectilinear) - xlow);
		if (x < 0) x = 0;
		if (x > widthMax) x = widthMax;
		x = x + (j * iWidth / nImages);

		y = (int) (yFactor) * (( (cos(p1) * sin(theta) - sin(p1) * cos(theta) * cos(phi - l0)) / coscRectilinear) - heightLow);
		y = heightMax - y;
		if (y < 0) y = 0;
		if (y > heightMax) y = heightMax;	
	      }
	  }
      }
    
    //PANNINI projection
    if(pMethod == PANNINI)
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
	//horizantal angle of view of [0:360] and vertical of [MIN_ANGLE:MAX_ANGLE]
	phi = 360.0 - phi;
	phi = phi * 2.0 * M_PI / 360.0;
	theta -= 90;
	theta *= -1;
	theta *= 2.0 * M_PI / 360.0;

	for(unsigned int i = 0 ; i < nImages ; i++)
	  {
	    iMinx = i * interval;
	    iMaxx = (i + 1) * interval;
	    //check for point in interval
	    if(phi <= (iMaxx) && phi >= (iMinx))
	      {
		//the longitude of projection center
		l0 = iMinx + interval / 2;
		
		//latitude of projection center
		p1 = 0;
		
		//use the S variable of pannini projection mentioned in the thesis
		//finding the min and max of the x direction
		sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMaxy) + cos(p1) * cos(iMaxx - l0));
		max = sPannini * (sin(iMaxx - l0));
		sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMiny) + cos(p1) * cos(iMinx - l0));
		min = sPannini * (sin(iMinx - l0));
		xFactor = (double) (iWidth / nImages) / (max - min);
		double xlow = min;
		widthMax = (iWidth / nImages) - 1;
		//finding the min and max of y direction
		sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMaxy) + cos(p1) * cos(iMaxx - l0));
		max = sPannini * (tan(iMaxy) * (cos(p1) - sin(p1) * 1/tan(iMaxy) * cos(iMaxx - l0)));
		sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMiny) + cos(p1) * cos(iMinx - l0));
		min = sPannini * (tan(iMiny) * (cos(p1) - sin(p1) * 1/tan(iMiny) * cos(iMinx - l0)));
		yFactor = (double) iHeight / (max - min);
		heightLow = min;
		heightMax = iHeight - 1;
		//project the points and add them to image
		sPannini = (pParam + 1) / (pParam + sin(p1) * tan(theta) + cos(p1) * cos(phi - l0));
		
		x = (int)(xFactor) * (sPannini * sin(phi - l0) - xlow);
		if (x < 0) x = 0;
		if (x > widthMax) x = widthMax;
		x = x + (i * widthMax);
		
		y = (int) (yFactor) * ( (sPannini * tan(theta) * (cos(p1) - sin(p1) * (1/tan(theta)) * cos(phi - l0) ) ) - heightLow );
		y = heightMax - y;		
		if (y < 0) y = 0;
		if (y > heightMax) y = heightMax;
	      }
	  }
      }
    
    //STEREOGRAPHIC projection
    if(pMethod == STEREOGRAPHIC)
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
	
	for (unsigned int j = 0 ; j < nImages ; j++)
	  {
	    iMinx = j * interval;
	    iMaxx = (j + 1) * interval;
	    //check for point in intervals
	    if(phi <= (iMaxx) && phi >= (iMinx))
	      {
		
		//longitude of projection center
		l0 = iMinx + interval / 2;
		//use the R variable of stereographic projection mentioned in the thesis
		//finding the min and max of x direction
		k = (2 * pParam) / (1 + sin(p1) * sin(p1) + cos(p1) * cos(p1) * cos(iMaxx - l0));
		max = k * cos(p1) * sin (iMaxx - l0);
		k = (2 * pParam) / (1 + sin (p1) * sin(p1) + cos(p1) * cos(p1) * cos(iMinx -l0));
		min = k * cos(p1) * sin (iMinx -l0);
		xFactor = (double) (iWidth / nImages) / (max - min);
		double xlow = min;
		widthMax = (iWidth / nImages) - 1;
		//finding the min and max of y direction
		k = (2 * pParam) / (1 + sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0));
		max = k * (cos(p1) * sin(iMaxy) - sin(p1) * cos(iMaxy) * cos(iMaxx - l0));
		k = (2 * pParam) / (1 + sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0));
		min = k * (cos(p1) * sin(iMiny) - sin(p1) * cos(iMiny) * cos(iMinx - l0));
		yFactor = (double) iHeight / (max - min);
		heightLow = min;
		heightMax = iHeight - 1;
		//project the points and add them to image
		k = (2 * pParam) / (1 + sin(p1) * sin(theta) + cos(p1) * cos(theta) * cos(phi - l0));

		x = (int) (xFactor) * (k * cos(theta) * sin(phi - l0) - xlow);
		if (x < 0) x = 0;
		if (x > widthMax) x = widthMax;
		x = x + (j * iWidth / nImages);
		
		y = (int) (yFactor) * (k * ( cos(p1) * sin(theta) - sin(p1) * cos(theta) * cos(phi - l0) ) - heightLow);
		y = heightMax - y;
		if (y < 0) y = 0;
		if (y > heightMax) y = heightMax;
	      }
	  }
      }
    
    //ZAXIS projection
    if(pMethod == ZAXIS)
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

	x = (int) ( xFactor * phi);
	if (x < 0) x = 0;
	if (x > widthMax) x = widthMax;
	
	///////////////////check this
	y = (int) ( yFactor * ((*it)[1] - heightLow) );
	y = heightMax - y;
	if (y < 0) y = 0;
	if (y > heightMax) y = heightMax;
      }

    //AZIMUTHAL projection
    if(pMethod == AZIMUTHAL)
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

	//calculate kprime according to Mathworld article
	kprime = sqrt(2/(1+sin(Phi1)*sin(theta)+cos(Phi1)*cos(theta)*cos(phi-Long0)));
	
	// add minimum x position as an offset
	x = (int) xFactor*kprime*cos(theta)*sin(phi-Long0);
	x = widthMax/2 + x;
	if (x < 0) x = 0;
	if (x > widthMax) x = widthMax;
        
	// add minimum y position as an offset
	y = (int) yFactor*kprime*(cos(Phi1)*sin(theta)-sin(Phi1)*cos(theta)*cos(phi-Long0));
	y = -y + heightMax/2;
	if (y < 0) y = 0;
	if (y > heightMax) y = heightMax;
      }
  }

  void panorama::recoverPointCloud(const cv::Mat& range_image,
				   cv::Mat& reflectance_image, vector<cv::Vec4f> &reduced_points) 
  {
    if (range_image.cols != reflectance_image.cols
	|| range_image.rows != reflectance_image.rows) 
      {
	cerr << "range image and reflectance image have different geometries - using empty range image" << endl;
	reflectance_image.create(range_image.size(), CV_8U);
	reflectance_image = cv::Scalar::all(0);
      }

    //recover from EQUIRECTANGULAR projection
    if(pMethod == EQUIRECTANGULAR) 
      {
	xFactor = (double) range_image.size().width / 2.0 / M_PI;
	//int widthMax = range_image.size().width - 1;
	yFactor = (double) range_image.size().height / ((MAX_ANGLE - MIN_ANGLE) / 360.0 * 2.0 * M_PI);
	heightLow = (0.0 - MIN_ANGLE) / 360 * 2 * M_PI;
	heightMax = range_image.size().height - 1;

	bool first_seen = true;
	for (int row = 0; row < range_image.size().height; ++row) 
	  {
	    for (int col = 0; col < range_image.size().width; ++col) 
	      {
		float range = range_image.at<float>(row, col);
		float reflectance = reflectance_image.at<uchar>(row,col)/255.0;
		float theta = (heightMax - row + 0.5) / yFactor - heightLow; 
		float phi = (col + 0.5 ) / xFactor; 
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
		reduced_points.push_back(cv::Vec4f(-100.0*cartesian[1],
						   100.0*cartesian[2],
						   100.0*cartesian[0],
						   reflectance));
	      }
	  }
      }

    //recover from CYLINDRICAL projection
    if(pMethod == CYLINDRICAL) 
      {
	xFactor = (double) range_image.size().width / 2 / M_PI;
	//int widthMax = range_image.size().width - 1;
	yFactor = (double) range_image.size().height / (tan(MAX_ANGLE / 360 * 2 * M_PI) - tan(MIN_ANGLE / 360 * 2 * M_PI));
	heightLow = (MIN_ANGLE) / 360 * 2 * M_PI;
	//int heightMax = range_image.size().height - 1;
      
	bool first_seen = true;
	for (int row = 0; row < range_image.size().height; ++row) 
	  {
	    for (int col = 0; col < range_image.size().width; ++col) 
	      {
		float range = range_image.at<float>(row, col);
		float reflectance = reflectance_image.at<uchar>(row,col)/255.0;
		float theta = atan2(row + 0.5 + yFactor * tan(heightLow), yFactor);
		float phi = (col + 0.5) / xFactor; 
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
		reduced_points.push_back(cv::Vec4f(-100.0*cartesian[1],
						   100.0*cartesian[2],
						   100.0*cartesian[0],
						   reflectance));
	      }
	  }
      }

    //recover from MERCATOR projection
    if(pMethod == MERCATOR) 
      {
	xFactor = (double) range_image.size().width / 2 / M_PI;
	yFactor = (double) range_image.size().height / ( log( tan( MAX_ANGLE / 360 * 2 * M_PI ) + ( 1 / cos( MAX_ANGLE / 360 * 2 * M_PI ) ) ) - log ( tan( MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI) ) ) );
	heightLow = log(tan(MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI)));
	heightMax = range_image.size().height - 1;
      
	bool first_seen = true;
	for (int row = 0; row < range_image.size().height; ++row) 
	  {
	    for (int col = 0; col < range_image.size().width; ++col) 
	      {
		float range = range_image.at<float>(row, col);
		float reflectance = reflectance_image.at<uchar>(row,col)/255.0;
		float theta = 2 * atan2(exp((heightMax - row + 0.5) / yFactor + heightLow), 1.) - M_PI_2;
		float phi = (col + 0.5) / xFactor; 
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
		reduced_points.push_back(cv::Vec4f(-100.0*cartesian[1],
						   100.0*cartesian[2],
						   100.0*cartesian[0],
						   reflectance));
	      }
	  }
      }

    //recover from CONIC projection
    if(pMethod == CONIC) 
      {
	// set up maximum latitude and longitude angles of the robot 
	MIN_VERT_ANGLE = MIN_ANGLE * M_PI / 180.0; 
	MAX_VERT_ANGLE = MAX_ANGLE * M_PI / 180.0;
	MIN_HORIZ_ANGLE = -M_PI; 
	MAX_HORIZ_ANGLE = M_PI;
	// set up initial parameters according to MathWorld: http://mathworld.wolfram.com/AlbersEqual-AreaConicProjection.html
	Lat0 = 0.; 
	Long0 = 0.;
	Phi1 = -40. * M_PI / 180.0; 
	Phi2 = 60 * M_PI / 180.0;
	n = (sin(Phi1) + sin(Phi2)) / 2.;
	C = sqr(cos(Phi1)) + 2 * n * sin(Phi1);
	Rho0 = sqrt(C - 2 * n * sin(Lat0)) / n;
	// set up max values for x and y and add the longitude to x axis and latitude to y axis
	xmax = (1./n * sqrt(C - 2*n*sin( MIN_VERT_ANGLE )) ) * sin(n * (MAX_HORIZ_ANGLE - Long0));
	xmin = (1./n * sqrt(C - 2*n*sin( MIN_VERT_ANGLE )) ) * sin(n * (MIN_HORIZ_ANGLE - Long0));
	xFactor = (double) range_image.size().width / ( xmax - xmin );
	ymin = Rho0 - (1./n * sqrt(C - 2*n*sin(MIN_VERT_ANGLE)) ) * cos(n * ( 0. - Long0 ));
	ymax = Rho0 - (1./n * sqrt(C - 2*n*sin(MAX_VERT_ANGLE)) ) * cos(n * (MAX_HORIZ_ANGLE - Long0 ));
	yFactor = (double) range_image.size().height / ( ymax - ymin );
	heightMax = range_image.size().height - 1;

	bool first_seen = true;
	for (int row = 0; row < range_image.size().height; ++row) 
	  {
	    for (int col = 0; col < range_image.size().width; ++col) 
	      {
		float range = range_image.at<float>(row, col);
		float reflectance = reflectance_image.at<uchar>(row,col)/255.0;
		float x = col * 1. / xFactor - fabs(xmin);
		float y = (heightMax - row) * 1. / yFactor - fabs(ymin);
		float theta = asin((C - (x*x + (Rho0 - y) * (Rho0 - y)) * n * n) / (2 * n));
		float phi = Long0 + (1./n) * ::atan2(x, (float)Rho0 - y);

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
		reduced_points.push_back(cv::Vec4f(-100.0*cartesian[1],
						   100.0*cartesian[2],
						   100.0*cartesian[0],
						   reflectance));
	      }
	  }
      }
  }

  unsigned int panorama::getImageWidth()
  {
    return iWidth;
  }

  unsigned int panorama::getImageHeight()
  {
    return iHeight;
  }

  float panorama::getMaxRange()
  {
    return maxRange;
  }

  projection_method panorama::getProjectionMethod()
  {
    return pMethod;
  }

  unsigned int panorama::getNumberOfImages()
  {
    return nImages;
  }
  
  double panorama::getProjectionParam()
  {
    return pParam;
  }
  
  cv::Mat panorama::getReflectanceImage()
  {
    return iReflectance;
  }

  cv::Mat panorama::getMap()
  {
    return iMap;
  }

  cv::Mat panorama::getRangeImage()
  {
    return iRange;
  }

  cv::Mat panorama::getColorImage()
  {
    return iColor;
  }
  
  vector<vector<vector<cv::Vec3f> > > panorama::getExtendedMap()
  {
    return extendedIMap;
  }

  panorama_map_method panorama::getMapMethod()
  {
    return mapMethod;
  }

  void panorama::getDescription()
  {
    cout << "panorama created with width: " << iWidth << ", and height: "
	 << iHeight << ", and projection method: " << projectionMethodToString(pMethod)
	 << ", number of images: " << nImages << ", projection param: " << pParam << "."
	 << endl;
    cout << endl;
  }

}

