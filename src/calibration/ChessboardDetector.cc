#include "calibration/ChessboardDetector.h"
#include <chrono>
#include <fstream>
#include <boost/filesystem.hpp>
#include <calibration/DetectionFileHandler.h>
#include "calibration/Chessboard.h"

namespace calibration {

ChessboardDetector::ChessboardDetector(cv::Size patternSize, float squareSize, bool adaptiveThreshold, bool normalizeImage, bool filterQuads, bool fastCheck, bool sectorBasedApproach) :
    _patternSize(patternSize),
    _squareSize(squareSize),
    _sectorBasedApproach(sectorBasedApproach)
{
    _flags = 0;
    if (adaptiveThreshold && !sectorBasedApproach) { _flags |= cv::CALIB_CB_ADAPTIVE_THRESH; }
    if (normalizeImage) { _flags |= cv::CALIB_CB_NORMALIZE_IMAGE; }
    if (filterQuads && !sectorBasedApproach) { _flags |= cv::CALIB_CB_FILTER_QUADS; }
    if (fastCheck && !sectorBasedApproach) { _flags |= cv::CALIB_CB_FAST_CHECK; }

    for (int i = 0; i < patternSize.height; i++) {
        for (int j = 0; j < patternSize.width; j++) {
            _patternPoints.push_back(cv::Point3f(j * _squareSize, i * _squareSize, 0));
        }
    }
}

bool ChessboardDetector::detect(const cv::Mat& image)
{
    cv::Mat gray;

    if (image.channels() != 1) {
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    auto start = std::chrono::high_resolution_clock::now();

    bool found = false;

    if (_sectorBasedApproach) {
#if CV_MAJOR_VERSION > 3
        found = findChessboardCornersSB(gray, _patternSize, _imagePoints, _flags | cv::CALIB_CB_ACCURACY);
#endif
    } else {
        found = findChessboardCorners(gray, _patternSize, _imagePoints, _flags);

        if (found) {
            cornerSubPix(gray, _imagePoints, cv::Size(11, 11), cv::Size(-1, -1),
                         cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
        }
    }

    auto end = std::chrono::high_resolution_clock::now();
    _detectionTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    if (_imagePoints.size() == _patternPoints.size()) {
        _objectPoints = _patternPoints;
    } else {
        _objectPoints = std::vector<cv::Point3f>();
    }

    return found;
}

void ChessboardDetector::writeDetectionsToFile(const std::string& path)
{
    std::map<std::string, cv::Point3f> detections;
    int i = 0;
    for(cv::Point2f point : _imagePoints){
        //corner 1
        std::stringstream id;
        id << "CHE" << _patternSize.width << "X" << _patternSize.height << "ID" << i;
        detections.insert({id.str(), cv::Point3f(point.x, point.y, 0.0)});
        i++;
    }
    DetectionFileHandler::writeDetectionsToFile(path, detections);
}

void ChessboardDetector::readDetectionsFromFile(const std::string& path)
{
    //TODO not compatible with the new detection file format
    std::cerr << "depricated function, don't use it! it's not compatible with the new detection file format!" << std::endl;
    _imagePoints.clear();

    if (boost::filesystem::exists(boost::filesystem::path(path))) {
        _imagePoints = Chessboard::readPoint2fChessboardFromFile(path);
    }
}

} // namespace calibration
