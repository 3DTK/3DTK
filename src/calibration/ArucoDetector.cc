#include "calibration/ArucoDetector.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>

#if CV_MAJOR_VERSION > 3

namespace calibration {

ArucoDetector::ArucoDetector(std::vector<AprilTag::AprilTag3f> patternPoints, std::string dictionaryName) :
    _patternPoints(patternPoints),
    _dictionaryName(dictionaryName)
{
    if (_dictionaryName.compare("DICT_6X6_250") == 0) {
        _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
    } else if (_dictionaryName.compare("DICT_APRILTAG_36h11") == 0) {
        _dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_APRILTAG_36h11);
    } else {
        throw(std::invalid_argument("unrecognized dictionary name, only DICT_6X6_250 and DICT_APRILTAG_36h11 are supported!"));
    }

    _detectorParams = cv::aruco::DetectorParameters::create();
    _detectorParams->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
}

ArucoDetector::~ArucoDetector()
{
}

bool ArucoDetector::detect(const cv::Mat& image)
{
    _tags.clear();

    cv::Mat gray;

    if (image.channels() != 1) {
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    std::vector<int> markerIds;
    std::vector<std::vector<cv::Point2f> > markerCorners;
    cv::aruco::detectMarkers(gray, _dictionary, markerCorners, markerIds, _detectorParams);

    for (uint i = 0; i < markerIds.size(); i++) {
        AprilTag::AprilTag2f aprilTag2f = AprilTag::AprilTag2f(markerIds.at(i));

        aprilTag2f.point1 = markerCorners.at(i).at(0);
        aprilTag2f.point2 = markerCorners.at(i).at(1);
        aprilTag2f.point3 = markerCorners.at(i).at(2);
        aprilTag2f.point4 = markerCorners.at(i).at(3);

        _tags.push_back(aprilTag2f);
    }

     _imagePoints.clear();
    _objectPoints.clear();

    if (_patternPoints.size() == 0) {
        // if no pattern points provided copy all tags to image points
        for (AprilTag::AprilTag2f aprilTag2f : _tags) {
            _imagePoints.push_back(aprilTag2f.point1);
            _imagePoints.push_back(aprilTag2f.point2);
            _imagePoints.push_back(aprilTag2f.point3);
            _imagePoints.push_back(aprilTag2f.point4);
        }
    } else {
        // if pattern points provided copy only matching image points
        for (AprilTag::AprilTag2f aprilTag2f : _tags) {
            for (AprilTag::AprilTag3f aprilTag3f : _patternPoints) {
                if (aprilTag3f.compair(aprilTag2f) == 0) {
                    _imagePoints.push_back(aprilTag2f.point1);
                    _imagePoints.push_back(aprilTag2f.point2);
                    _imagePoints.push_back(aprilTag2f.point3);
                    _imagePoints.push_back(aprilTag2f.point4);

                    _objectPoints.push_back(aprilTag3f.point1);
                    _objectPoints.push_back(aprilTag3f.point2);
                    _objectPoints.push_back(aprilTag3f.point3);
                    _objectPoints.push_back(aprilTag3f.point4);
                }
            }
        }
    }

    return (_tags.size() > 0);
}

void ArucoDetector::writeDetectionsToFile(const std::string& path)
{
    std::fstream f;
    f.open(path, std::ios::out);
    f << "#APRIL_2D" << std::endl;
    for (AprilTag::AprilTag2f tag : _tags) {
        f << tag.toString();
    }
    f.close();
}

void ArucoDetector::readDetectionsFromFile(const std::string& path)
{
    _tags.clear();

    if (boost::filesystem::exists(boost::filesystem::path(path))) {
        _tags = AprilTag::createAprilTag2fFromFile(path);
    }
}

} // namespace calibration

#endif