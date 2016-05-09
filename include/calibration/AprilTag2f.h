//
// Created by Joschka van der Lucht on 23.03.16.
//

#ifndef CAMERACALIBRATIONTOOL_APRILTAG2F_H
#define CAMERACALIBRATIONTOOL_APRILTAG2F_H

#include <string>
#include <opencv2/core/core.hpp>
#include "calibration/AprilTag.h"
#include "calibration/AprilTag3f.h"

namespace AprilTag{

    class AprilTag3f;

    class AprilTag2f {
    public:
        AprilTag2f();

        AprilTag2f(int id);

        AprilTag2f(int id, cv::Point2f leftup, cv::Point2f leftdown, cv::Point2f rightdown, cv::Point2f rightup);

        ~AprilTag2f();

        int id;
        cv::Point2f point1;
        cv::Point2f point2;
        cv::Point2f point3;
        cv::Point2f point4;

        int compair(AprilTag::AprilTag2f aprilTag2f);

        int compair(AprilTag::AprilTag3f aprilTag3f);

        std::string toString();
    };

}



#endif //CAMERACALIBRATIONTOOL_APRILTAG2F_H
