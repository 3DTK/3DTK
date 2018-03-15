//
// Created by Joschka van der Lucht on 02.03.18.
//

#include "calibration/Calibrator.h"


namespace calibration {

    void Calibrator::matchAprilTags(std::vector<AprilTag::AprilTag2f> imagePoints,
                                    std::vector<AprilTag::AprilTag3f> objectPoints,
                                    std::vector<cv::Point2f> *matchedImagePoints,
                                    std::vector<cv::Point3f> *matchedObjectPoints,
                                    std::vector<int> estimationIDs,
                                    std::vector<cv::Point2f> *matchedEstimationImagePoints,
                                    std::vector<cv::Point3f> *matchedEstimationObjectPoints) {
        
    }

    void Calibrator::calibrateIntrinsic() {

    }

    void Calibrator::calibrateExtrinsic() {

    }
}