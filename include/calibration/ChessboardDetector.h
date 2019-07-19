#ifndef CALIBRATION_CHESSBOARDDETECTOR_H
#define CALIBRATION_CHESSBOARDDETECTOR_H

#include "calibration/Detector.h"
#include <opencv2/opencv.hpp>

namespace calibration
{

class ChessboardDetector : public Detector
{
public:
    ChessboardDetector(cv::Size patternSize, float squareSize = -1, bool adaptiveThreshold = true, bool normalizeImage = true, bool filterQuads = true, bool fastCheck = false);

public:
    bool detect(const cv::Mat& image);
    void writeDetectionsToFile(const std::string& path);
    void readDetectionsFromFile(const std::string& path);

private:
    std::vector<cv::Point3f> _patternPoints;
    cv::Size _patternSize;
    float _squareSize;
    int _flags;
};

} // namespace calibration

#endif // CALIBRATION_CHESSBOARDDETECTOR_H
