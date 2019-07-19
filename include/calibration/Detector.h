#ifndef CALIBRATION_DETECTOR_H
#define CALIBRATION_DETECTOR_H

#include <opencv2/opencv.hpp>

namespace calibration
{

class Detector
{
public:
    virtual ~Detector() {};

public:
    virtual bool detect(const cv::Mat& image) = 0;
    virtual void writeDetectionsToFile(const std::string& path) = 0;
    virtual void readDetectionsFromFile(const std::string& path) = 0;

public:
    std::vector<cv::Point2f> getImagePoints() { return _imagePoints; };
    std::vector<cv::Point3f> getObjectPoints() { return _objectPoints; };
    long getDetectionTimeMilliSec() { return _detectionTime; };

protected:
    std::vector<cv::Point2f> _imagePoints;
    std::vector<cv::Point3f> _objectPoints;
    long _detectionTime;
};

} // namespace calibration

#endif // CALIBRATION_DETECTOR_H
