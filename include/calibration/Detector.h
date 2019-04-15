//
// Created by Joschka van der Lucht on 28.02.18.
//

#ifndef INC_3DTK_DETECTOR_H
#define INC_3DTK_DETECTOR_H

#include "AprilTag.h"
#include "AprilTag2f.h"
#include "Chessboard.h"
#include <common/image_types.h>
#include <apriltag.h>
#include <tag36h11.h>
#include <tag25h9.h>
#include <tag16h5.h>
#include <string>
#include <iostream>

namespace calibration {
    class Detector {
    private:

    public:
        void detectChessboard(cv::Mat image, std::vector<cv::Point2f> *points, cv::Size boardSize);

        void detectAprilTag(image_u8_t *image, std::vector<AprilTag::AprilTag2f> *tags, float decimate = 1,
                            float blur = 0.8,
                            int threads = 4, bool debug = false, bool refine_edges = true, std::string tagFamily = "tag36h11");

        void readApilTagDetectionsFromFile(std::string path, std::vector<AprilTag::AprilTag2f> *tags);

        void readChessboardDetectionsFromFile(std::string path, std::vector<cv::Point2f> *tags);

        void writeApilTagDetectionsToFile(std::string path, std::vector<AprilTag::AprilTag2f> tags);

        void writeChessboardDetectionsToFile(std::string path, std::vector<cv::Point2f> tags);
    };
}
#endif //INC_3DTK_DETECTOR_H
