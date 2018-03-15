//
// Created by Joschka van der Lucht on 02.03.18.
//

#ifndef INC_3DTK_CALIBRATOR_H
#define INC_3DTK_CALIBRATOR_H

#include "AprilTag.h"

namespace calibration {
    class Calibrator {
    private:

    public:
        void
        matchAprilTags(std::vector<AprilTag::AprilTag2f> imagePoints,
                       std::vector<AprilTag::AprilTag3f> objectPoints,
                       std::vector<std::vector<cv::Point2f>> *matchedImagePoints,
                       std::vector<std::vector<cv::Point3f>> *matchedObjectPoints,
                       std::vector<int> *estimationIDs = nullptr,
                       std::vector<std::vector<cv::Point2f>> *matchedEstimationImagePoints = nullptr,
                       std::vector<std::vector<cv::Point3f>> *matchedEstimationObjectPoints = nullptr);

        void calibrateIntrinsic();

        void calibrateExtrinsic();
    };
}


#endif //INC_3DTK_CALIBRATOR_H
