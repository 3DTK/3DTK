//
// Created by Joschka van der Lucht on 02.03.18.
//

#include "calibration/Calibrator.h"

using namespace cv;

namespace calibration {

    void Calibrator::matchAprilTags(std::vector<AprilTag::AprilTag2f> imagePoints,
                                    std::vector<AprilTag::AprilTag3f> objectPoints,
                                    std::vector<std::vector<cv::Point2f>> *matchedImagePoints,
                                    std::vector<std::vector<cv::Point3f>> *matchedObjectPoints,
                                    std::vector<int> *estimationIDs,
                                    std::vector<std::vector<cv::Point2f>> *matchedEstimationImagePoints,
                                    std::vector<std::vector<cv::Point3f>> *matchedEstimationObjectPoints) {
        std::vector<Point2f> imgPoints;
        std::vector<Point3f> patPoints;

        std::vector<Point2f> imgEstPoints;
        std::vector<Point3f> patEstPoints;

        for (AprilTag::AprilTag2f aprilTag2f : imagePoints) {
            for (AprilTag::AprilTag3f aprilTag3f : objectPoints) {
                if (aprilTag3f.compair(aprilTag2f) == 0) {
                    imgPoints.push_back(aprilTag2f.point1);
                    imgPoints.push_back(aprilTag2f.point2);
                    imgPoints.push_back(aprilTag2f.point3);
                    imgPoints.push_back(aprilTag2f.point4);

                    patPoints.push_back(aprilTag3f.point1);
                    patPoints.push_back(aprilTag3f.point2);
                    patPoints.push_back(aprilTag3f.point3);
                    patPoints.push_back(aprilTag3f.point4);

                    if (estimationIDs != nullptr &&
                        std::find(estimationIDs->begin(), estimationIDs->end(), aprilTag2f.id) !=
                        estimationIDs->end()) {
                        imgEstPoints.push_back(aprilTag2f.point1);
                        imgEstPoints.push_back(aprilTag2f.point2);
                        imgEstPoints.push_back(aprilTag2f.point3);
                        imgEstPoints.push_back(aprilTag2f.point4);

                        patEstPoints.push_back(Point3f(aprilTag3f.point1.x, aprilTag3f.point1.y, 0));
                        patEstPoints.push_back(Point3f(aprilTag3f.point2.x, aprilTag3f.point2.y, 0));
                        patEstPoints.push_back(Point3f(aprilTag3f.point3.x, aprilTag3f.point3.y, 0));
                        patEstPoints.push_back(Point3f(aprilTag3f.point4.x, aprilTag3f.point4.y, 0));
                    }
                    break;
                }
            }
        }
        if (imgPoints.size() > 4 && patPoints.size() > 4) {
            matchedImagePoints->push_back(imgPoints);
            matchedObjectPoints->push_back(patPoints);

            if (imgEstPoints.size() > 4 && matchedEstimationImagePoints != nullptr && matchedEstimationObjectPoints !=
                                                                                      nullptr) {
                matchedEstimationImagePoints->push_back(imgEstPoints);
                matchedEstimationObjectPoints->push_back(patEstPoints);
            }
        }
    }

    void Calibrator::calibrateIntrinsic() {

    }

    void Calibrator::calibrateExtrinsic() {

    }
}
