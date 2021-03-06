#include "calibration/AprilTagDetector.h"
#include "calibration/DetectionFileHandler.h"
#include <chrono>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <apriltag.h>
#include <tag36h11.h>
#include <tag25h9.h>
#include <tag16h5.h>

namespace calibration {

AprilTagDetector::AprilTagDetector(std::vector<AprilTag::AprilTag3f> patternPoints, std::string tagFamily, float decimate, float blur, int hamming, bool refineEdges, bool cornerSubpixel, int threads, bool debug) :
    _patternPoints(patternPoints),
    _tagFamilyName(tagFamily),
    _cornerSubpixel(cornerSubpixel)
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
        if (_cornerSubpixel) {
            std::vector<cv::Point2f> tagPoints;
            tagPoints.push_back(cv::Point2f((float) ((det->p)[0][0]), (float) ((det->p)[0][1])));
            tagPoints.push_back(cv::Point2f((float) ((det->p)[1][0]), (float) ((det->p)[1][1])));
            tagPoints.push_back(cv::Point2f((float) ((det->p)[2][0]), (float) ((det->p)[2][1])));
            tagPoints.push_back(cv::Point2f((float) ((det->p)[3][0]), (float) ((det->p)[3][1])));

            cornerSubPix(gray, tagPoints, cv::Size(8, 8), cv::Size(-1, -1),
                     cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));

            aprilTag2f.point4 = tagPoints.at(0);
            aprilTag2f.point3 = tagPoints.at(1);
            aprilTag2f.point2 = tagPoints.at(2);
            aprilTag2f.point1 = tagPoints.at(3);
        } else {
            aprilTag2f.point4 = cv::Point2f((float) ((det->p)[0][0]), (float) ((det->p)[0][1]));
            aprilTag2f.point3 = cv::Point2f((float) ((det->p)[1][0]), (float) ((det->p)[1][1]));
            aprilTag2f.point2 = cv::Point2f((float) ((det->p)[2][0]), (float) ((det->p)[2][1]));
            aprilTag2f.point1 = cv::Point2f((float) ((det->p)[3][0]), (float) ((det->p)[3][1]));
        }

        _tags.push_back(aprilTag2f);
        apriltag_detection_destroy(det);
    }
    zarray_destroy(detections);
    auto end = std::chrono::high_resolution_clock::now();
    _detectionTime = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

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

void AprilTagDetector::writeDetectionsToFile(const std::string& path)
{
    std::map<std::string, cv::Point3f> detections;
    for(AprilTag::AprilTag2f tag : _tags){
        //corner 1
        std::stringstream id;
        id << "APR" << tag.id << "TL";
        detections.insert({id.str(), cv::Point3f(tag.point1.x, tag.point1.y, 0.0)});
        //corner 2
        id.str("");
        id << "APR" << tag.id << "BL";
        detections.insert({id.str(), cv::Point3f(tag.point2.x, tag.point2.y, 0.0)});
        //corner 3
        id.str("");
        id << "APR" << tag.id << "BR";
        detections.insert({id.str(), cv::Point3f(tag.point3.x, tag.point3.y, 0.0)});
        //corner 4
        id.str("");
        id << "APR" << tag.id << "TR";
        detections.insert({id.str(), cv::Point3f(tag.point4.x, tag.point4.y, 0.0)});
    }
    DetectionFileHandler::writeDetectionsToFile(path, detections);
}

void AprilTagDetector::readDetectionsFromFile(const std::string& path)
{
    _tags.clear();
    //TODO not compatible with the new detection file format
    std::cerr << "depricated function, don't use it! it's not compatible with the new detection file format!" << std::endl;
    if (boost::filesystem::exists(boost::filesystem::path(path))) {
        _tags = AprilTag::createAprilTag2fFromFile(path);
    }
}

} // namespace calibration
