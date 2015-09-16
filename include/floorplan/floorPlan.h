/*
 * floorPlan.h
 *
 *  Created on: Apr 29, 2013
 *      Author: rdumitru
 */

#ifndef FLOORPLAN_H_
#define FLOORPLAN_H_

//==============================================================================
//  Includes.
//==============================================================================
// OpenCV includes.
#include <opencv2/opencv.hpp>

// C++ includes.
#include <vector>

//==============================================================================
//  Class declaration.
//==============================================================================
namespace floorplan {

class FloorPlan {
private:
    // Private fields.
    cv::Mat grayImg;                // the original gray image
    cv::Mat thresh;                 // the thresholded binary image
    std::vector<cv::Vec4i> walls;   // the walls as lines
    double scale;                   // what does one pixel mean?

    // Private static fields.
    static const double STD_DEV_MUL;        // the coefficient for the standard deviation when thresholding
    static const double ANGLE_RES;          // the dominant orientation histogram resolution in degrees

    static const double HOUGH_RHO;          // rho used in the Hough transform
    static const double HOUGH_THETA;        // theta used in the Hough transform
    static const int    HOUGH_THRESH;       // Hough transform threshold
    static const double HOUGH_MIN_LINE_LEN; // minimum line length for Hough transform in cm; we take care of scaling

    static const double MAX_ANGLE;          // the maximum angle of the orientation histogram
    static const double SNAP_ANGLE_THRESH;  // angle to consider a line close to a dominant orientation

    // Private methods.
    std::vector<cv::Vec4i> extractWallLines();
    std::vector<int> computeOrientationHist();
    void correctWallLines();


protected:
    explicit inline FloorPlan() {}

public:
    // Constructors.
    FloorPlan(const cv::Mat &grayPlan, const double &scale);
    FloorPlan(const FloorPlan &other);
    ~FloorPlan();

};

} /* namespace floorplan */

#endif /* FLOORPLAN_H_ */
