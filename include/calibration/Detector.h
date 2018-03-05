//
// Created by Joschka van der Lucht on 28.02.18.
//

#ifndef INC_3DTK_DETECTOR_H
#define INC_3DTK_DETECTOR_H

#include "AprilTag2f.h"
#include <common/image_types.h>
#include <apriltag.h>
#include <tag36h10.h>
#include <tag36h11.h>
#include <tag25h9.h>
#include <tag25h7.h>
#include <tag16h5.h>
#include <string>
#include <iostream>

namespace calibration {
    class Detector {
    private:

    public:
        void detectChessboard(Mat &image, std::vector<cv::Point2f> &points, Size boardSize);

        void detectAprilTag(image_u8_t *image, std::vector<AprilTag::AprilTag2f> &tags, float decimate = 1,
                            float blur = 0.8,
                            int threads = 4, bool debug = false, bool refine_edges = true, bool refine_decodes = true,
                            bool refine_pose = true, std::string tagFamily = "tag36h11");
    };
}
#endif //INC_3DTK_DETECTOR_H
