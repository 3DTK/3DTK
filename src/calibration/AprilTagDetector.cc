#include "calibration/AprilTagDetector.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <apriltag.h>
#include <tag36h11.h>
#include <tag25h9.h>
#include <tag16h5.h>

namespace calibration {

AprilTagDetector::AprilTagDetector(std::vector<AprilTag::AprilTag3f> patternPoints, std::string tagFamily, float decimate, float blur, int hamming, bool refineEdges, int threads, bool debug) :
    _patternPoints(patternPoints),
    _tagFamilyName(tagFamily)
{
    //set tag family
    if (_tagFamilyName.compare("tag36h11") == 0) {
        _tagFamily = tag36h11_create();
    } else if (_tagFamilyName.compare("tag25h9") == 0) {
        _tagFamily = tag25h9_create();
    } else if (_tagFamilyName.compare("tag16h5") == 0) {
        _tagFamily = tag16h5_create();
    } else {
        throw(std::invalid_argument("unrecognized tag family name, only tag36h11, tag25h9, tag16h5 are supported!"));
    }

    // init AprilTag detector
    _apriltagDetector = apriltag_detector_create();
    if (hamming > 0) { apriltag_detector_add_family_bits(_apriltagDetector, _tagFamily, hamming); }
    apriltag_detector_add_family(_apriltagDetector, _tagFamily);
    _apriltagDetector->quad_decimate = decimate;
    _apriltagDetector->quad_sigma = blur;
    _apriltagDetector->nthreads = threads;
    _apriltagDetector->debug = debug;
    _apriltagDetector->refine_edges = refineEdges;
}

AprilTagDetector::~AprilTagDetector()
{
    apriltag_detector_destroy(_apriltagDetector);

    if (_tagFamilyName.compare("tag36h11") == 0) {
        tag36h11_destroy(_tagFamily);
    } else if (_tagFamilyName.compare("tag25h9") == 0) {
        tag25h9_destroy(_tagFamily);
    } else if (_tagFamilyName.compare("tag16h5") == 0) {
        tag16h5_destroy(_tagFamily);
    }
}

bool AprilTagDetector::detect(const cv::Mat& image)
{
    _tags.clear();

    cv::Mat gray;

    if (image.channels() != 1) {
        cvtColor(image, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = image;
    }

    // Make an image_u8_t header for the Mat data
    image_u8_t im = {
        gray.cols, // width
        gray.rows, // height
        gray.cols, // stride
        gray.data // buf
    };

    auto start = std::chrono::high_resolution_clock::now();
    zarray_t *detections = apriltag_detector_detect(_apriltagDetector, &im);

    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);
        AprilTag::AprilTag2f aprilTag2f = AprilTag::AprilTag2f(det->id);
        aprilTag2f.point4 = cv::Point2f((float) ((det->p)[0][0]), (float) ((det->p)[0][1]));
        aprilTag2f.point3 = cv::Point2f((float) ((det->p)[1][0]), (float) ((det->p)[1][1]));
        aprilTag2f.point2 = cv::Point2f((float) ((det->p)[2][0]), (float) ((det->p)[2][1]));
        aprilTag2f.point1 = cv::Point2f((float) ((det->p)[3][0]), (float) ((det->p)[3][1]));
        _tags.push_back(aprilTag2f);
        apriltag_detection_destroy(det);
    }
    zarray_destroy(detections);
    auto end = std::chrono::high_resolution_clock::now();
    _detectionTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

    if (_patternPoints.size() == 0) {
        // if no pattern points provided copy all tags to image points
        for (AprilTag::AprilTag2f aprilTag2f : _tags) {
            _imagePoints.push_back(aprilTag2f.point1);
            _imagePoints.push_back(aprilTag2f.point2);
            _imagePoints.push_back(aprilTag2f.point3);
            _imagePoints.push_back(aprilTag2f.point4);
        }

        _objectPoints.clear();
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

void AprilTagDetector::writeDetectionsToFile(const std::string& path)
{
    std::fstream f;
    f.open(path, std::ios::out);
    f << "#APRIL_2D" << std::endl;
    for (AprilTag::AprilTag2f tag : _tags) {
        f << tag.toString();
    }
    f.close();
}

void AprilTagDetector::readDetectionsFromFile(const std::string& path)
{
    _tags.clear();

    if (boost::filesystem::exists(boost::filesystem::path(path))) {
        _tags = AprilTag::createAprilTag2fFromFile(path);
    }
}

} // namespace calibration
