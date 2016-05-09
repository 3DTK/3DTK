//
// Created by Joschka van der Lucht on 23.03.16.
//

#ifndef CAMERACALIBRATIONTOOL_APRILTAG3F_H
#define CAMERACALIBRATIONTOOL_APRILTAG3F_H

#include <string>
#include "calibration/AprilTag.h"
#include "calibration/AprilTag2f.h"

namespace AprilTag{

    class AprilTag3f {

    public:

        AprilTag3f();

        AprilTag3f(int id);

        AprilTag3f(int id, cv::Point3f leftup, cv::Point3f leftdown, cv::Point3f rightdown, cv::Point3f rightup);

        ~AprilTag3f();

        int id;
        cv::Point3f point1;
        cv::Point3f point2;
        cv::Point3f point3;
        cv::Point3f point4;

        int compair(AprilTag::AprilTag2f aprilTag2f);

        int compair(AprilTag::AprilTag3f aprilTag3f);

        std::string toString();
    };
}



#endif //CAMERACALIBRATIONTOOL_APRILTAG3F_H
