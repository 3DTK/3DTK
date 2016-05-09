//
// Created by Joschka van der Lucht on 22.03.16.
//

#ifndef CAMERACALIBRATIONTOOL_APRILTAG_H
#define CAMERACALIBRATIONTOOL_APRILTAG_H


#include <opencv2/core/core.hpp>
#include <stdio.h>
#include <sstream>
#include <fstream>
#include <iostream>
#include "calibration/AprilTag3f.h"
#include "calibration/AprilTag2f.h"
#include "calibration/Settings.h"


namespace AprilTag {

    std::vector<AprilTag::AprilTag3f> createAprilTag3fFromFile(std::string path, Settings &s);
    std::vector<AprilTag::AprilTag2f> createAprilTag2fFromFile(std::string path);

}
#endif //CAMERACALIBRATIONTOOL_APRILTAG_H
