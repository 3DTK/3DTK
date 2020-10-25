#ifndef CALIBRATION_ARUCODETECTOR_H
#define CALIBRATION_ARUCODETECTOR_H

#include "calibration/Detector.h"
#include "calibration/AprilTag.h"
#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

namespace calibration
{

class ArucoDetector : public Detector
{
public:
    ArucoDetector(std::vector<AprilTag::AprilTag3f> patternPoints = std::vector<AprilTag::AprilTag3f>(), std::string dictionaryName = "DICT_6X6_250");
    ~ArucoDetector();

public:
    bool detect(const cv::Mat& image);
    void writeDetectionsToFile(const std::string& path);
    void readDetectionsFromFile(const std::string& path);

private:
    std::vector<AprilTag::AprilTag3f> _patternPoints;
    std::string _dictionaryName;
    cv::Ptr<cv::aruco::Dictionary> _dictionary;
    cv::Ptr<cv::aruco::DetectorParameters> _detectorParams;
    std::vector<AprilTag::AprilTag2f> _tags;
};

} // namespace calibration

#endif // CALIBRATION_ARUCODETECTOR_H
