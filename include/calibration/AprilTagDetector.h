#ifndef CALIBRATION_APRILTAGDETECTOR_H
#define CALIBRATION_APRILTAGDETECTOR_H

#include "calibration/Detector.h"
#include "calibration/AprilTag.h"
#include <opencv2/opencv.hpp>

typedef struct apriltag_detector apriltag_detector_t;
typedef struct apriltag_family apriltag_family_t;

namespace calibration
{

class AprilTagDetector : public Detector
{
public:
    AprilTagDetector(std::vector<AprilTag::AprilTag3f> patternPoints = std::vector<AprilTag::AprilTag3f>(), std::string tagFamily = "tag36h11", float decimate = 1.0, float blur = 0.0, int hamming = 0, bool refineEdges = true, int threads = 4, bool debug = false);
    ~AprilTagDetector();

public:
    bool detect(const cv::Mat& image);
    void writeDetectionsToFile(const std::string& path);
    void readDetectionsFromFile(const std::string& path);

private:
    std::vector<AprilTag::AprilTag3f> _patternPoints;
    std::string _tagFamilyName;
    apriltag_detector_t* _apriltagDetector;
    apriltag_family_t* _tagFamily;
    std::vector<AprilTag::AprilTag2f> _tags;
};

} // namespace calibration

#endif // CALIBRATION_APRILTAGDETECTOR_H
