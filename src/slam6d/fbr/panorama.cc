/*
 * panorama implementation
 *
 * Copyright (C) HamidReza Houshiar
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/fbr/panorama.h"

using namespace std;

namespace fbr{
  
  void panorama::init(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod){
    iWidth = width;
    iHeight = height;
    pMethod = method;
    nImages = numberOfImages;
    pParam = param;
    if(mMethod == FARTHEST){
      iMap.create(iHeight, iWidth, CV_32FC(3));
      iMap = cv::Scalar::all(0);
    }
    else if(mMethod == EXTENDED){
      extendedIMap.resize(iHeight);
      for (unsigned int i = 0; i < iHeight; i++)
	extendedIMap[i].resize(iWidth);
    }
    iReflectance.create(iHeight, iWidth, CV_8U);
    iReflectance = cv::Scalar::all(0);
    iRange.create(iHeight, iWidth, CV_32FC(1));
    iRange = cv::Scalar::all(0);
    mapMethod = mMethod;
  }

  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param, panorama_map_method mMethod){ 
    init(width, height, method, numberOfImages, param, mMethod);
  }
  
  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages, double param){ 
    init(width, height, method, numberOfImages, param, FARTHEST);
  }
  
  panorama::panorama(unsigned int width, unsigned int height, projection_method method, unsigned int numberOfImages){
    double param = 0;
    if(method == PANNINI)
      param = 1;
    else if (method == STEREOGRAPHIC)
      param = 2;
   
    init(width, height, method, numberOfImages, param, FARTHEST);
  }  

  panorama::panorama(unsigned int width, unsigned int height, projection_method method){ 
    double param = 0;
    unsigned int numberOfImages = 1;
    if(method == RECTILINEAR)
      numberOfImages = 3;
    else if(method == PANNINI){
      numberOfImages = 3;
      param = 1;
    } else if (method == STEREOGRAPHIC){
      numberOfImages = 3;
      param = 2;
    }

    init(width, height, method, numberOfImages, param, FARTHEST);
  }
  
  void panorama::map(int x, int y, cv::MatIterator_<cv::Vec4f> it, double range){
    iReflectance.at<uchar>(y,x) = (*it)[3]*255;//reflectance
    iRange.at<float>(y,x) = range;//range
    if(mapMethod == FARTHEST){
      //adding the point with max distance
      if( iRange.at<float>(y,x) < range ){
        iMap.at<cv::Vec3f>(y,x)[0] = (*it)[0];//x
        iMap.at<cv::Vec3f>(y,x)[1] = (*it)[1];//y
        iMap.at<cv::Vec3f>(y,x)[2] = (*it)[2];//z
      }
    }else if(mapMethod == EXTENDED){
      //adding all the points
      cv::Vec3f point;
      point[0] = (*it)[0];//x
      point[1] = (*it)[1];//y
      point[2] = (*it)[2];//z
      extendedIMap[y][x].push_back(point);
    }
  }

  void panorama::createPanorama(cv::Mat scan){

    //EQUIRECTANGULAR projection
    if(pMethod == EQUIRECTANGULAR){
      //adding the longitude to x axis and latitude to y axis
      double xFactor = (double) iWidth / 2 / M_PI;
      int widthMax = iWidth - 1;
      double yFactor = (double) iHeight / ((MAX_ANGLE - MIN_ANGLE) / 360 * 2 * M_PI);
      //shift all the valuse to positive points on image 
      double heightLow =(0 - MIN_ANGLE) / 360 * 2 * M_PI;
      int heightMax = iHeight - 1;
      
      cv::MatIterator_<cv::Vec4f> it, end; 
      
      for( it = scan.begin<cv::Vec4f>(), end = scan.end<cv::Vec4f>(); it != end; ++it){
	double kart[3], polar[3], phi, theta, range;
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
	int x = (int) ( xFactor * phi);
	if (x < 0) x = 0;
	if (x > widthMax) x = widthMax;
	int y = (int) ( yFactor * (theta + heightLow) );
	y = heightMax - y;
	if (y < 0) y = 0;
	if (y > heightMax) y = heightMax;
	
	//create the iReflectance iRange and map
	map(x, y, it, range);
      }
    }

    //CONIC projection
    if(pMethod == CONIC){
      // set up maximum latitude and longitude angles of the robot 
			double MIN_VERT_ANGLE = MIN_ANGLE * M_PI / 180.0, MAX_VERT_ANGLE = MAX_ANGLE * M_PI / 180.0,
             MIN_HORIZ_ANGLE = -M_PI, MAX_HORIZ_ANGLE = M_PI;
      // set up initial parameters according to MathWorld: http://mathworld.wolfram.com/AlbersEqual-AreaConicProjection.html
      double Lat0 = 0., Long0 = 0.;
      double Phi1 = -40. * M_PI / 180.0, Phi2 = 60 * M_PI / 180.0;
      double n = (sin(Phi1) + sin(Phi2)) / 2.;
      double C = sqr(cos(Phi1)) + 2 * n * sin(Phi1);
      double Rho0 = sqrt(C - 2 * n * sin(Lat0)) / n;
			// set up max values for x and y and add the longitude to x axis and latitude to y axis
			double xmax = (1./n * sqrt(C - 2*n*sin( MIN_VERT_ANGLE )) ) * sin(n * (MAX_HORIZ_ANGLE - Long0));
			double xmin = (1./n * sqrt(C - 2*n*sin( MIN_VERT_ANGLE )) ) * sin(n * (MIN_HORIZ_ANGLE - Long0));
      double xFactor = (double) iWidth / ( xmax - xmin );
      int widthMax = iWidth - 1;
			double ymin = Rho0 - (1./n * sqrt(C - 2*n*sin(MIN_VERT_ANGLE)) ) * cos(n * ( 0. - Long0 ));
			double ymax = Rho0 - (1./n * sqrt(C - 2*n*sin(MAX_VERT_ANGLE)) ) * cos(n * (MAX_HORIZ_ANGLE - Long0 ));
      double yFactor = (double) iHeight / ( ymax - ymin );
      //shift all the values to positive points on image 
      int heightMax = iHeight - 1;
      cv::MatIterator_<cv::Vec4f> it, end; 

      for( it = scan.begin<cv::Vec4f>(), end = scan.end<cv::Vec4f>(); it != end; ++it){
        double kart[3], polar[3], phi, theta, range;
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
        int x = (int) ( xFactor * (sqrt(C - 2 * n * sin( theta) ) / n * sin(n * (phi - Long0)) + fabs(xmin) ) );
        if (x < 0) x = 0;
        if (x > widthMax) x = widthMax;
        
        // add minimum y position as an offset
        int y = (int) ( yFactor * (Rho0 - (1/n * sqrt(C - 2 * n * sin( theta) ) ) * cos(n * (phi - Long0)) + fabs( ymin ) ) );
        y = heightMax - y;
        if (y < 0) y = 0;
        if (y > heightMax) y = heightMax;
        //create the iReflectance iRange and map
        map(x, y, it, range);
      }
    }
    
    //CYLINDRICAL projection
    if(pMethod == CYLINDRICAL){
      //adding the longitude to x and tan(latitude) to y
      //find the x and y range
      double xFactor = (double) iWidth / 2 / M_PI;
      int widthMax = iWidth - 1;
      double yFactor = (double) iHeight / (tan(MAX_ANGLE / 360 * 2 * M_PI) - tan(MIN_ANGLE / 360 * 2 * M_PI));
      double heightLow = (MIN_ANGLE) / 360 * 2 * M_PI;
      int heightMax = iHeight - 1;
      
      cv::MatIterator_<cv::Vec4f> it, end; 
      
      for( it = scan.begin<cv::Vec4f>(), end = scan.end<cv::Vec4f>(); it != end; ++it){
	double kart[3], polar[3], phi, theta, range;
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
	int x = (int) ( xFactor * phi);
	if (x < 0) x = 0;
	if (x > widthMax) x = widthMax;
	int y = (int) ((double) yFactor * (tan(theta) - tan(heightLow)));
	y = heightMax - y;
	if (y < 0) y = 0;
	if (y > heightMax) y = heightMax;
	
	//create the iReflectance iRange and map
	map(x, y, it, range);
      }
    }
    
    //Mercator Projection
    if( pMethod == MERCATOR){
      //find the x and y range
      double xFactor = (double) iWidth / 2 / M_PI;
      int widthMax = iWidth - 1;
      double yFactor = (double) iHeight / ( log( tan( MAX_ANGLE / 360 * 2 * M_PI ) + ( 1 / cos( MAX_ANGLE / 360 * 2 * M_PI ) ) ) - log ( tan( MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI) ) ) );
      double heightLow = log(tan(MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI)));
      int heightMax = iHeight - 1;
      
      cv::MatIterator_<cv::Vec4f> it, end; 
      
      for( it = scan.begin<cv::Vec4f>(), end = scan.end<cv::Vec4f>(); it != end; ++it){
	double kart[3], polar[3], phi, theta, range;
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
	int x = (int) ( xFactor * phi);
	if (x < 0) x = 0;
	if (x > widthMax) x = widthMax;
	int y = (int) ( yFactor * (log(tan(theta) + (1/cos(theta))) - heightLow) );
	y = heightMax - y;
	if (y < 0) y = 0;
	if (y > heightMax) y = heightMax;
	
	//create the iReflectance iRange and map
	map(x, y, it, range);
      }
    }
    
    //RECTILINEAR projection
    if(pMethod == RECTILINEAR){
      //default value for nImages
      if(nImages == 0) nImages = 3;
      cout<<"Number of images per scan is: "<<nImages<<endl;
      double l0, p1, iMinx, iMaxx, iMiny, iMaxy, interval;
      interval = 2 * M_PI / nImages;
      iMiny = -M_PI/9;
      iMaxy = 2*M_PI/9;
      //latitude of projection center
      p1 = 0;
      
      //go through all points 
      cv::MatIterator_<cv::Vec4f> it, end; 
      
      for( it = scan.begin<cv::Vec4f>(), end = scan.end<cv::Vec4f>(); it != end; ++it){
	double kart[3], polar[3], phi, theta, range;
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
	for(unsigned int j = 0 ; j < nImages ; j++){
	  iMinx = j * interval;
	  iMaxx = (j + 1) * interval;
	  //check for point in interval
	  if(phi < iMaxx && phi > iMinx){
	    double max, min, coscRectilinear;
	    //the longitude of projection center
	    l0 = iMinx + interval / 2;
	    //finding the min and max of the x direction
	    coscRectilinear = sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0);
	    max = (cos(iMaxy) * sin(iMaxx - l0) / coscRectilinear);
	    coscRectilinear = sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0);
	    min = (cos(iMiny) * sin(iMinx - l0) / coscRectilinear);
	    double xFactor = (double) (iWidth / nImages) / (max - min);
	    double xlow = min;
	    int widthMax = (iWidth / nImages) - 1;
	    //finding the min and max of y direction
	    coscRectilinear = sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0);
	    max = ( (cos(p1) * sin(iMaxy) - sin(p1) * cos(iMaxy) * cos(iMaxx - l0) )/ coscRectilinear);
	    coscRectilinear = sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0);
	    min = ( (cos(p1) * sin(iMiny) - sin(p1) * cos(iMiny) * cos(iMinx - l0) )/ coscRectilinear);
	    double yFactor = (double) iHeight / (max - min);
	    double heightLow = min;
	    int heightMax = iHeight - 1;
	    //project the points and add them to image
	    coscRectilinear = sin(p1) * sin(theta) + cos(p1) * cos(theta) * cos(phi - l0);
	    int x = (int)(xFactor) * ((cos(theta) * sin(phi - l0) / coscRectilinear) - xlow);
	    if (x < 0) x = 0;
	    if (x > widthMax) x = widthMax;
	    x = x + (j * iWidth / nImages);
	    int y = (int) (yFactor) * (( (cos(p1) * sin(theta) - sin(p1) * cos(theta) * cos(phi - l0)) / coscRectilinear) - heightLow);
	    y = heightMax - y;
	    if (y < 0) y = 0;
	    if (y > heightMax) y = heightMax;
	    
	    //create the iReflectance iRange and map
	    map(x, y, it, range);
	  }
	}
      }
    }
    
    //PANNINI projection
    if(pMethod == PANNINI){
      //default values for nImages and dPannini==pParam
      if(pParam == 0) pParam = 1;
      if(nImages == 0) nImages = 3;
      cout << "Parameter d is:" << pParam <<", Number of images per scan is:" << nImages << endl;
      double l0, p1, iMinx, iMaxx, iMiny, iMaxy, interval;
      interval = 2 * M_PI / nImages;
      iMiny = -M_PI/9;
      iMaxy = 2*M_PI/9;
      //latitude of projection center
      p1 = 0;
      
      cv::MatIterator_<cv::Vec4f> it, end; 
         
      for( it = scan.begin<cv::Vec4f>(), end = scan.end<cv::Vec4f>(); it != end; ++it){
	double kart[3], polar[3], phi, theta, range;
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
	for(unsigned int j = 0 ; j < nImages ; j++){
	  iMinx = j * interval;
	  iMaxx = (j + 1) * interval;
	  //check for point in interval
	  if(phi < (iMaxx) && phi > (iMinx)){
	    double max, min, sPannini;
	    //the longitude of projection center
	    l0 = iMinx + interval / 2;
	    //use the S variable of pannini projection mentioned in the thesis
	    //finding the min and max of the x direction
	    sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMaxy) + cos(p1) * cos(iMaxx - l0));
	    max = sPannini * (sin(iMaxx - l0));
	    sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMiny) + cos(p1) * cos(iMinx - l0));
	    min = sPannini * (sin(iMinx - l0));
	    double xFactor = (double) (iWidth / nImages) / (max - min);
	    double xlow = min;
	    int widthMax = (iWidth / nImages) - 1;
	    //finding the min and max of y direction
	    sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMaxy) + cos(p1) * cos(iMaxx - l0));
	    max = sPannini * (tan(iMaxy) * (cos(p1) - sin(p1) * 1/tan(iMaxy) * cos(iMaxx - l0)));
	    sPannini = (pParam + 1) / (pParam + sin(p1) * tan(iMiny) + cos(p1) * cos(iMinx - l0));
	    min = sPannini * (tan(iMiny) * (cos(p1) - sin(p1) * 1/tan(iMiny) * cos(iMinx - l0)));
	    double yFactor = (double) iHeight / (max - min);
	    double heightLow = min;
	    int heightMax = iHeight - 1;
	    //project the points and add them to image
	    sPannini = (pParam + 1) / (pParam + sin(p1) * tan(theta) + cos(p1) * cos(phi - l0));
	    int x = (int)(xFactor) * (sPannini * sin(phi - l0) - xlow);
	    if (x < 0) x = 0;
	    if (x > widthMax) x = widthMax;
	    x = x + (j * iWidth / nImages);
	    int y = (int) (yFactor) * ( (sPannini * tan(theta) * (cos(p1) - sin(p1) * (1/tan(theta)) * cos(phi - l0) ) ) - heightLow );
	    y = heightMax - y;
	    if (y < 0) y = 0;
	    if (y > heightMax) y = heightMax;
		    
	    //create the iReflectance iRange and map
	    map(x, y, it, range);
	  }
	}
      }
    }
    
    //STEREOGRAPHIC projection
    if(pMethod == STEREOGRAPHIC){
      //default values for nImages and rStereographic==pParam
      if(pParam == 0) pParam = 2;
      if(nImages == 0) nImages = 3;
      cout << "Paremeter R is:" << pParam << ", Number of images per scan is:" << nImages << endl;
      // l0 and p1 are the center of projection iminx, imaxx, iminy, imaxy are the bounderis of intervals
      double l0, p1, iMinx, iMaxx, iMiny, iMaxy, interval;
      interval = 2 * M_PI / nImages;
      iMiny = -M_PI/9;
      iMaxy = 2*M_PI/9;
      //latitude of projection center
      p1 = 0;
      
      //go through all points
      cv::MatIterator_<cv::Vec4f> it, end; 
         
      for( it = scan.begin<cv::Vec4f>(), end = scan.end<cv::Vec4f>(); it != end; ++it){
	double kart[3], polar[3], phi, theta, range;
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
	for (unsigned int j = 0 ; j < nImages ; j++){
	  iMinx = j * interval;
	  iMaxx = (j + 1) * interval;
	  //check for point in intervals
	  if(phi < (iMaxx) && phi > (iMinx)){
	    double max, min, k;
	    //longitude of projection center
	    l0 = iMinx + interval / 2;
	    //use the R variable of stereographic projection mentioned in the thesis
	    //finding the min and max of x direction
	    k = (2 * pParam) / (1 + sin(p1) * sin(p1) + cos(p1) * cos(p1) * cos(iMaxx - l0));
	    max = k * cos(p1) * sin (iMaxx - l0);
	    k = (2 * pParam) / (1 + sin (p1) * sin(p1) + cos(p1) * cos(p1) * cos(iMinx -l0));
	    min = k * cos(p1) * sin (iMinx -l0);
	    double xFactor = (double) (iWidth / nImages) / (max - min);
	    double xlow = min;
	    int widthMax = (iWidth / nImages) - 1;
	    //finding the min and max of y direction
	    k = (2 * pParam) / (1 + sin(p1) * sin(iMaxy) + cos(p1) * cos(iMaxy) * cos(iMaxx - l0));
	    max = k * (cos(p1) * sin(iMaxy) - sin(p1) * cos(iMaxy) * cos(iMaxx - l0));
	    k = (2 * pParam) / (1 + sin(p1) * sin(iMiny) + cos(p1) * cos(iMiny) * cos(iMinx - l0));
	    min = k * (cos(p1) * sin(iMiny) - sin(p1) * cos(iMiny) * cos(iMinx - l0));
	    double yFactor = (double) iHeight / (max - min);
	    double heightLow = min;
	    int heightMax = iHeight - 1;
	    //project the points and add them to image
	    k = (2 * pParam) / (1 + sin(p1) * sin(theta) + cos(p1) * cos(theta) * cos(phi - l0));
	    int x = (int) (xFactor) * (k * cos(theta) * sin(phi - l0) - xlow);
	    if (x < 0) x = 0;
	    if (x > widthMax) x = widthMax;
	    x = x + (j * iWidth / nImages);
	    int y = (int) (yFactor) * (k * ( cos(p1) * sin(theta) - sin(p1) * cos(theta) * cos(phi - l0) ) - heightLow);
	    y = heightMax - y;
	    if (y < 0) y = 0;
	    if (y > heightMax) y = heightMax;
	    
	    //create the iReflectance iRange and map
	    map(x, y, it, range);
	  }
	}
      }
    }
    
    //ZAXIS projection
    if(pMethod == ZAXIS){
      double zmin = -200;
      double zmax = 4000;
      //adding the longitude to x axis and latitude to y axis
      double xFactor = (double) iWidth / 2 / M_PI;
      int widthMax = iWidth - 1;
      cout << "ZMAX= " << zmax << " ZMIN= "<< zmin << endl;
      double yFactor = (double) iHeight / (zmax - zmin);
      //shift all the valuse to positive points on image 
      double heightLow = zmin;
      int heightMax = iHeight - 1;
      
      cv::MatIterator_<cv::Vec4f> it, end; 
         
      for( it = scan.begin<cv::Vec4f>(), end = scan.end<cv::Vec4f>(); it != end; ++it){
	double kart[3], polar[3], phi, theta, range;
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
	int x = (int) ( xFactor * phi);
	if (x < 0) x = 0;
	if (x > widthMax) x = widthMax;
	///////////////////check this
	int y = (int) ( yFactor * ((*it)[1] - heightLow) );
	y = heightMax - y;
	if (y < 0) y = 0;
	if (y > heightMax) y = heightMax;
	
	//create the iReflectance iRange and map
	map(x, y, it, range);
      }
    }
  }

  void panorama::recoverPointCloud(const cv::Mat& range_image,
          cv::Mat& reflectance_image, vector<cv::Vec4f> &reduced_points) {
      if (range_image.cols != reflectance_image.cols
              || range_image.rows != reflectance_image.rows) {
          cerr << "range image and reflectance image have different geometries - using empty range image" << endl;
          reflectance_image.create(range_image.size(), CV_8U);
          reflectance_image = cv::Scalar::all(0);
      }

    //recover from EQUIRECTANGULAR projection
    if(pMethod == EQUIRECTANGULAR) {
      double xFactor = (double) range_image.size().width / 2 / M_PI;
      //int widthMax = range_image.size().width - 1;
      double yFactor = (double) range_image.size().height / ((MAX_ANGLE - MIN_ANGLE) / 360 * 2 * M_PI);
      double heightLow = (0 - MIN_ANGLE) / 360 * 2 * M_PI;
      int heightMax = range_image.size().height - 1;

      bool first_seen = true;
      for (int row = 0; row < range_image.size().height; ++row) {
        for (int col = 0; col < range_image.size().width; ++col) {
          float range = range_image.at<float>(row, col);
          float reflectance = reflectance_image.at<uchar>(row,col)/255.0;
          float theta = (heightMax - row) / yFactor - heightLow; 
          float phi = col / xFactor; 
          phi *= 180.0 / M_PI;
          phi = 360.0 - phi;
          phi *= M_PI / 180.0;
          theta *= 180.0 / M_PI;
          theta *= -1;
          theta += 90.0;
          theta *= M_PI / 180.0;
          double polar[3] = { theta, phi, range }, cartesian[3] = {0., 0., 0.}; 
        	toKartesian(polar, cartesian);
          if( fabs(cartesian[0]) < 1e-5 && fabs(cartesian[1]) < 1e-5 && fabs(cartesian[2]) < 1e-5) {
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
    if(pMethod == CYLINDRICAL) {
      double xFactor = (double) range_image.size().width / 2 / M_PI;
      //int widthMax = range_image.size().width - 1;
      double yFactor = (double) range_image.size().height / (tan(MAX_ANGLE / 360 * 2 * M_PI) - tan(MIN_ANGLE / 360 * 2 * M_PI));
      double heightLow = (MIN_ANGLE) / 360 * 2 * M_PI;
      //int heightMax = range_image.size().height - 1;
      
      bool first_seen = true;
      for (int row = 0; row < range_image.size().height; ++row) {
        for (int col = 0; col < range_image.size().width; ++col) {
          float range = range_image.at<float>(row, col);
          float reflectance = reflectance_image.at<uchar>(row,col)/255.0;
          float theta = atan2(row + yFactor * tan(heightLow), yFactor);
          float phi = col / xFactor; 
          phi *= 180.0 / M_PI;
          phi = 360.0 - phi;
          phi *= M_PI / 180.0;
          theta *= 180.0 / M_PI;
          theta *= -1;
          theta += 90.0;
          theta *= M_PI / 180.0;
          double polar[3] = { theta, phi, range }, cartesian[3] = {0., 0., 0.}; 
        	toKartesian(polar, cartesian);
          if( fabs(cartesian[0]) < 1e-5 && fabs(cartesian[1]) < 1e-5 && fabs(cartesian[2]) < 1e-5) {
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
    if(pMethod == MERCATOR) {
      double xFactor = (double) range_image.size().width / 2 / M_PI;
      double yFactor = (double) range_image.size().height / ( log( tan( MAX_ANGLE / 360 * 2 * M_PI ) + ( 1 / cos( MAX_ANGLE / 360 * 2 * M_PI ) ) ) - log ( tan( MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI) ) ) );
      double heightLow = log(tan(MIN_ANGLE / 360 * 2 * M_PI) + (1/cos(MIN_ANGLE / 360 * 2 * M_PI)));
      int heightMax = range_image.size().height - 1;
      
      bool first_seen = true;
      for (int row = 0; row < range_image.size().height; ++row) {
        for (int col = 0; col < range_image.size().width; ++col) {
          float range = range_image.at<float>(row, col);
          float reflectance = reflectance_image.at<uchar>(row,col)/255.0;
          float theta = 2 * atan2(exp((heightMax - row) / yFactor + heightLow), 1.) - M_PI_2;
          float phi = col / xFactor; 
          phi *= 180.0 / M_PI;
          phi = 180.0 - phi;
          phi *= M_PI / 180.0;
          theta *= 180.0 / M_PI;
          theta *= -1;
          theta += 90.0;
          theta *= M_PI / 180.0;
          double polar[3] = { theta, phi, range }, cartesian[3] = {0., 0., 0.}; 
        	toKartesian(polar, cartesian);
          if( fabs(cartesian[0]) < 1e-5 && fabs(cartesian[1]) < 1e-5 && fabs(cartesian[2]) < 1e-5) {
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
    if(pMethod == CONIC) {
       // set up maximum latitude and longitude angles of the robot 
			double MIN_VERT_ANGLE = MIN_ANGLE * M_PI / 180.0, MAX_VERT_ANGLE = MAX_ANGLE * M_PI / 180.0,
             MIN_HORIZ_ANGLE = -M_PI, MAX_HORIZ_ANGLE = M_PI;
      // set up initial parameters according to MathWorld: http://mathworld.wolfram.com/AlbersEqual-AreaConicProjection.html
      double Lat0 = 0., Long0 = 0.;
      double Phi1 = -40. * M_PI / 180.0, Phi2 = 60 * M_PI / 180.0;
      double n = (sin(Phi1) + sin(Phi2)) / 2.;
      double C = sqr(cos(Phi1)) + 2 * n * sin(Phi1);
      double Rho0 = sqrt(C - 2 * n * sin(Lat0)) / n;
			// set up max values for x and y and add the longitude to x axis and latitude to y axis
			double xmax = (1./n * sqrt(C - 2*n*sin( MIN_VERT_ANGLE )) ) * sin(n * (MAX_HORIZ_ANGLE - Long0));
			double xmin = (1./n * sqrt(C - 2*n*sin( MIN_VERT_ANGLE )) ) * sin(n * (MIN_HORIZ_ANGLE - Long0));
      double xFactor = (double) range_image.size().width / ( xmax - xmin );
			double ymin = Rho0 - (1./n * sqrt(C - 2*n*sin(MIN_VERT_ANGLE)) ) * cos(n * ( 0. - Long0 ));
			double ymax = Rho0 - (1./n * sqrt(C - 2*n*sin(MAX_VERT_ANGLE)) ) * cos(n * (MAX_HORIZ_ANGLE - Long0 ));
      double yFactor = (double) range_image.size().height / ( ymax - ymin );
      int heightMax = range_image.size().height - 1;

      bool first_seen = true;
      for (int row = 0; row < range_image.size().height; ++row) {
        for (int col = 0; col < range_image.size().width; ++col) {
          float range = range_image.at<float>(row, col);
          float reflectance = reflectance_image.at<uchar>(row,col)/255.0;
          float x = col * 1. / xFactor - fabs(xmin);
          float y = (heightMax - row) * 1. / yFactor - fabs(ymin);
          float theta = asin((C - (x*x + (Rho0 - y) * (Rho0 - y)) * n * n) / (2 * n));
          float phi = Long0 + (1./n) * ::atan2(x, Rho0 - y);

          phi *= 180.0 / M_PI;
          phi = 360.0 - phi;
          phi *= M_PI / 180.0;
          theta *= 180.0 / M_PI;
          theta *= -1;
          theta += 90.0;
          theta *= M_PI / 180.0;

          double polar[3] = { theta, phi, range }, cartesian[3] = {0., 0., 0.}; 
        	toKartesian(polar, cartesian);
          //if ( std::isnan(cartesian[0]) || std::isnan(cartesian[1]) || std::isnan(cartesian[2]) ) continue;
          if( fabs(cartesian[0]) < 1e-5 && fabs(cartesian[1]) < 1e-5 && fabs(cartesian[2]) < 1e-5) {
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

  unsigned int panorama::getImageWidth(){
    return iWidth;
  }

  unsigned int panorama::getImageHeight(){
    return iHeight;
  }

  projection_method panorama::getProjectionMethod(){
    return pMethod;
  }

  unsigned int panorama::getNumberOfImages(){
    return nImages;
  }
  
  double panorama::getProjectionParam(){
    return pParam;
  }
  
  cv::Mat panorama::getReflectanceImage(){
    return iReflectance;
  }

  cv::Mat panorama::getMap(){
    return iMap;
  }

  cv::Mat panorama::getRangeImage(){
    return iRange;
  }
  
  vector<vector<vector<cv::Vec3f> > > panorama::getExtendedMap(){
    return extendedIMap;
  }

  panorama_map_method panorama::getMapMethod(){
    return mapMethod;
  }

  void panorama::getDescription(){
    cout << "panorama created with width: " << iWidth << ", and height: "
	    << iHeight << ", and projection method: " << projectionMethodToString(pMethod)
	    << ", number of images: " << nImages << ", projection param: " << pParam << "."
	    << endl;
    cout << endl;
  }
}

