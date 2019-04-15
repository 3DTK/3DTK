//
// Created by Joschka van der Lucht on 28.02.18.
//


#include"calibration/Detector.h"
using namespace cv;

namespace calibration {
    void Detector::detectAprilTag(image_u8_t *image, std::vector<AprilTag::AprilTag2f> *tags, float decimate,
                                  float blur,
                                  int threads, bool debug, bool refine_edges, std::string tagFamily) {
        apriltag_family_t *tagFam;
        //set tag family
        if (tagFamily.compare("tag36h11") == 0) {
            tagFam = tag36h11_create();
        } else if (tagFamily.compare("tag25h9") == 0) {
            tagFam = tag25h9_create();
        } else if (tagFamily.compare("tag16h5") == 0) {
            tagFam = tag16h5_create();
        } else {
            std::cout << "error: false tagFamily, only tag36h11, tag36h10, tag25h9, tag25h7 and tag16h5 supported"
                      << std::endl;
            return;
        }
        //init AprilTag detector
        apriltag_detector_t *apriltagDetector;
        apriltagDetector = apriltag_detector_create();
        apriltag_detector_add_family(apriltagDetector, tagFam);
        apriltagDetector->quad_decimate = decimate;
        apriltagDetector->quad_sigma = blur;
        apriltagDetector->nthreads = threads;
        apriltagDetector->debug = debug;
        apriltagDetector->refine_edges = refine_edges;
        //stop time for detect tags
        timeval start, end;
        gettimeofday(&start, 0);
        zarray_t *detections = apriltag_detector_detect(apriltagDetector, image);
        gettimeofday(&end, 0);
        std::cout << "Time to detect aprilTags: " << end.tv_sec - start.tv_sec << " sec" << std::endl;

        if (detections == nullptr) {
            std::cout << "detection nullptr, no tag found";
        } else {
            for (int i = 0; i < zarray_size(detections); i++) {
                apriltag_detection_t *det;
                zarray_get(detections, i, &det);
                AprilTag::AprilTag2f aprilTag2f = AprilTag::AprilTag2f(det->id);
                aprilTag2f.point4 = Point2f((float) ((det->p)[0][0]), (float) ((det->p)[0][1]));
                aprilTag2f.point3 = Point2f((float) ((det->p)[1][0]), (float) ((det->p)[1][1]));
                aprilTag2f.point2 = Point2f((float) ((det->p)[2][0]), (float) ((det->p)[2][1]));
                aprilTag2f.point1 = Point2f((float) ((det->p)[3][0]), (float) ((det->p)[3][1]));
                tags->push_back(aprilTag2f);
                apriltag_detection_destroy(det);
            }
            zarray_destroy(detections);
        }
        apriltag_detector_destroy(apriltagDetector);
        std::cout << tags->size() << " Tags detected." << std::endl;
    }

    void Detector::detectChessboard(Mat image, std::vector<cv::Point2f> *points, Size boardSize) {
        bool found = findChessboardCorners(image, boardSize, points[0],
                                           CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE |
                                           CV_CALIB_CB_FILTER_QUADS);
        timeval start, end;
        if (found) {
            std::cout << "Performing subpixel refinement of chessboard corners." << std::endl;
            cornerSubPix(image, points[0], cvSize(11, 11), cvSize(-1, -1),
                         cvTermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));
        }

        gettimeofday(&end, 0);
        std::cout << "Time to detect chessboard: " << end.tv_sec - start.tv_sec << " sec" << std::endl;

    }

    void Detector::readApilTagDetectionsFromFile(std::string path, std::vector<AprilTag::AprilTag2f> *tags){
        if(FILE * detectFile = fopen(path.c_str(), "r")){
            fclose(detectFile);
            *tags = AprilTag::createAprilTag2fFromFile(path);
            std::cout << "load points from file" << std::endl;
        }
    }

    void Detector::writeApilTagDetectionsToFile(std::string path, std::vector<AprilTag::AprilTag2f> tags) {
        std::fstream f;
        f.open(path, std::ios::out);
        f << "#APRIL_2D" << std::endl;
        for(AprilTag::AprilTag2f tag : tags){
            f << tag.toString();
        }
        f.close();
    }

    void Detector::readChessboardDetectionsFromFile(std::string path, std::vector<cv::Point2f> *tags) {
        if(FILE * detectFile = fopen(path.c_str(), "r")){
            fclose(detectFile);
            *tags = Chessboard::readPoint2fChessboardFromFile(path);
            std::cout << "load points from file" << std::endl;
        }
    }

    void Detector::writeChessboardDetectionsToFile(std::string path, std::vector<cv::Point2f> tags) {
        std::fstream f;
        f.open(path, std::ios::out);
        f << "#CHESSBOARD" << std::endl;
        f << Chessboard::toString(tags);
        f.close();
    }
}
