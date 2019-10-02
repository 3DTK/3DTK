//
//  PictureHandler.h
//  CameraCalibration
//
//  Created by Joschka van der Lucht on 04.09.15.
//  Copyright (c) 2015 Joschka van der Lucht. All rights reserved.
//

#ifndef __CameraCalibration__PictureHandler__
#define __CameraCalibration__PictureHandler__

#include <stdio.h>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <apriltag.h>
#include "calibration/AprilTag.h"

using namespace AprilTag;


class PictureHandler {
private:
    image_u8_t *image;
    cv::Mat matImage;
    apriltag_detector_t *aprilDetector;
    apriltag_family_t *tagFam;
    std::vector<cv::Point2f> point2fVec;
    std::vector<AprilTag2f> pointMap;
    std::vector<AprilTag3f> matMap;
    cv::Size size;
    /**
     initialize the aprilTag detector
     */
    void initDetector(float decimate, float blur, int threads, bool debug, bool refine_edges);

    /**
     *  create a map<int, matd_t> with detected id's and 3x3 homography matrix describing the projection from an "ideal" tag (with corners at (-1,-1), (1,-1), (1,1), and (-1,1)) to pixels in the image
     */
    void getmatMapFromZarray(zarray_t *detections);



public:
    /**
     *  Constructor
     */
    PictureHandler();

    /**
     *  Constructor with parameters for detection
     *
     *  @param double decimate
     *  @param double blur
     *  @param int threads
     *  @param bool debug
     *  @param bool refine_edges
     *  @param bool refine_decodes
     */
    PictureHandler(float decimate, float blur, int threads, bool debug, bool refine_edges, std::string tagFamily);
    /**
     *  Destructor
     */
    ~PictureHandler();

    /**
     *  set the used aprilTag family
     *
     *  @param string family expected 'tag36h11', 'tag36h10', 'tag25h9', 'tag25h7'
     */
    void setTagFamily(std::string family);

    /**
     *  load the image from an given filepath
     *
     *  @param path filepath to the image
     */
    void loadImage(std::string path);

    /**
     *  start the detection of aprilTags
     *
     *  @return zarray_t with contains the results of the detection
     */
    int detectTags(Settings::Pattern pattern, Settings &s);

    /**
     *  getter for pointMap
     *
     *  @return map<int, Point3f> with <id, Point>
     */
    std::vector<AprilTag2f> getPointList();

    void setPointList(std::vector<AprilTag2f> list);

    bool clearPointlist();

    /**
     *   getter for matMap
     *
     *  @return map<int, matd_t> with id and 3x3 homography matrix describing the projection from an "ideal" tag (with corners at (-1,-1), (1,-1), (1,1), and (-1,1)) to pixels in the image
     */
    std::vector<AprilTag3f> getMatMap();

    /**
     * getter for size of image
     *
     * @return int imagesize
     */
    cv::Size getImageSize();

    std::vector<cv::Point2f> getPoint2fVec();

};


#endif /* defined(__CameraCalibration__PictureHandler__) */
