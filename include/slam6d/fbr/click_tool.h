//
//  ClickTool
//
//  Created by Jochen Barf on 02/08/16.
//  Copyright © 2016 Jochen Barf. All rights reserved.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include <fstream>
using namespace cv;

//these constants were found to be good, but my be altered
const float zoomIncrement = 1.25;   //!< defines how much zoom there is for pressing of the +/- buttons
const int maxWindowHeight = 2048;   //!< the maximum height a window is allowed to reach
const int minWindowWidth = 100;     //!< the minimum height a window is allowed to reach

/**
 Thanks to this enum several method can be used for processing both photo and scan data
 */
enum Picture{

    SCAN = 0,
    PHOTO = 1

};

/**

 */
enum Image{

	RANGE = 0,
	REFLECTANCE = 1,

};

/**
 Represents a point pair of a 2D point and a 3D point
 */
struct PointPair{

    Point2i points[2];      //!< the pixel coordinates of both scan and photo point
    Scalar color;           //!< the color that is used for bth representations in the images
    Point3f coordinates;    //!< the 3D coordinates of the laser scan point

};

/**
 Represents the position of the cursor
 */
struct MousePosition{

    Point2i position;
    Picture picture;

};
