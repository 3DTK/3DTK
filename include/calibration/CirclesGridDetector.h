#ifndef CALIBRATION_CIRCLESGRIDDETECTOR_H
#define CALIBRATION_CIRCLESGRIDDETECTOR_H

#include "calibration/Detector.h"
#include <opencv2/opencv.hpp>

namespace calibration
{

class CirclesGridDetector : public Detector
{
public:
    CirclesGridDetector(cv::Size patternSize, float gridSize = 1, bool symmetric = true);

public:
    bool detect(const cv::Mat& image);
    void writeDetectionsToFile(const std::string& path);
    void readDetectionsFromFile(const std::string& path);

private:
    std::vector<cv::Point3f> _patternPoints;
    cv::Size _patternSize;
    float _gridSize;
    int _flags;
};

} // namespace calibration

#endif // CALIBRATION_CIRCLESGRIDDETECTOR_H
