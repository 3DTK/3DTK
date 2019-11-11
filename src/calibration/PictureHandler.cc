//
//  PictureHandler.cpp
//  CameraCalibration
//
//  Created by Joschka van der Lucht on 04.09.15.
//  Copyright (c) 2015 Joschka van der Lucht. All rights reserved.
//

#include "calibration/PictureHandler.h"
#include <tag36h11.h>
#include <tag25h9.h>
#include <iostream>


using namespace std;
using namespace cv;

    PictureHandler::PictureHandler() :
        image(NULL),
        aprilDetector(NULL),
        tagFam(NULL)
    {

    }

    PictureHandler::PictureHandler(float decimate, float blur, int threads, bool debug, bool refine_edges,
                                   string tagFamily) :
        image(NULL),
        aprilDetector(NULL),
        tagFam(NULL)
    {
        setTagFamily(tagFamily);
        initDetector(decimate, blur, threads, debug, refine_edges);
    }


    PictureHandler::~PictureHandler() {
        apriltag_detector_destroy(aprilDetector);
    }


    int PictureHandler::detectTags(Settings::Pattern pattern, Settings &s) {
        // detect Tags und Zeit stoppen
        bool found = false;
        timeval start, end;
        gettimeofday(&start, 0);
        if(pattern == Settings::APRILTAG) {
            zarray_t *detections = apriltag_detector_detect(aprilDetector, image);
            gettimeofday(&end, 0);
            cout << "Time to detect aprilTags: " << end.tv_sec - start.tv_sec << " sec" << endl;

            if (detections == nullptr) {
                cout << "detection nullptr";
            } else {
                this->getmatMapFromZarray(detections);
            }
            image_u8_destroy(image);

        }else if (pattern == Settings::CHESSBOARD){
            found = findChessboardCorners(matImage, s.boardSize, this->point2fVec,
                cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_NORMALIZE_IMAGE | cv::CALIB_CB_FILTER_QUADS);

            if (found) {
                cout << "Performing subpixel refinement of chessboard corners." << endl;
                cornerSubPix(matImage, this->point2fVec, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria( cv::TermCriteria::EPS + cv::TermCriteria::MAX_ITER, 30, 0.1));
            }

            gettimeofday(&end, 0);
            cout << "Time to detect chessboard: " << end.tv_sec - start.tv_sec << " sec" << endl;

        }else if (pattern == Settings::CIRCLES_GRID){
            found = findCirclesGrid(matImage, s.boardSize, this->point2fVec);
            gettimeofday(&end, 0);
            cout << "Time to detect circles grid: " << end.tv_sec - start.tv_sec << " sec" << endl;

        }else if (pattern == Settings::ASYMMETRIC_CIRCLES_GRID){
            //TODO implement ASYMMETRIC_CIRCLES_GRID
            cout << "ASYMMETRIC_CIRCLES_GRID not implemented" << endl;
        }
        return found;
    }


    void PictureHandler::initDetector(float decimate, float blur, int threads, bool debug, bool refine_edges) {
        aprilDetector = apriltag_detector_create();
        if (tagFam == nullptr) {
            //TODO: Tag Familie nicht initialisiert
            cout << "Fehler: initDetector, tagFam ist NULL" << endl;
        }
        apriltag_detector_add_family(aprilDetector, tagFam);
        aprilDetector->quad_decimate = decimate;
        aprilDetector->quad_sigma = blur;
        aprilDetector->nthreads = threads;
        aprilDetector->debug = debug;
        aprilDetector->refine_edges = refine_edges;
    }


    void PictureHandler::setTagFamily(string family) {
        if (family.compare("tag36h11") == 0) {
            tagFam = tag36h11_create();
        } else if (family.compare("tag25h9") == 0) {
            tagFam = tag25h9_create();
        } else {
            //TODO: falsche Tag Familie angegben
            cout << "Fehler beim setzen der TagFamily" << endl;
        }
    }


    void PictureHandler::loadImage(string path) {
        Mat imageCV = imread(path,
#if CV_MAJOR_VERSION > 2
				cv::ImreadModes::IMREAD_GRAYSCALE
#else
				CV_LOAD_IMAGE_GRAYSCALE
#endif
				);
        this->matImage = imageCV;
        if (imageCV.rows > 0 && imageCV.cols > 0) {
            image = image_u8_create(imageCV.cols, imageCV.rows);

            for (int y = 0; y < imageCV.rows; y++) {
                uchar* row = imageCV.ptr<uchar>(y);
                for (int x = 0; x < imageCV.cols; x++) {
                    image->buf[y * image->stride + x] = row[x];
                }
            }
        }

        if (image == NULL) {
            perror("Error");
            cout << "Bild konnte nicht geladen werden!\nPfad: " << path << "\n"<< endl;
            //TODO: Bild konnte nicht geladen werden
            return;
        }
        this->size.height = image->height;
        this->size.width = image->width;
    }

    vector<AprilTag2f>PictureHandler::getPointList() {
        return this->pointMap;
    }

    void PictureHandler::setPointList(vector<AprilTag2f> list){
        this->pointMap = vector<AprilTag2f>(list);
    }

    vector<AprilTag3f> PictureHandler::getMatMap() {
        return this->matMap;
    }

    void PictureHandler::getmatMapFromZarray(zarray_t *detections) {
         for (int i = 0; i < zarray_size(detections); i++) {
             apriltag_detection_t *det;
             zarray_get(detections, i, &det);
             AprilTag2f aprilTag2f = AprilTag2f(det->id);
             aprilTag2f.point4 = Point2f((float)((det->p)[0][0]), (float)((det->p)[0][1]));
             aprilTag2f.point3 = Point2f((float)((det->p)[1][0]), (float)((det->p)[1][1]));
             aprilTag2f.point2 = Point2f((float)((det->p)[2][0]), (float)((det->p)[2][1]));
             aprilTag2f.point1 = Point2f((float)((det->p)[3][0]), (float)((det->p)[3][1]));
             this->pointMap.push_back(aprilTag2f);
             apriltag_detection_destroy(det);
         }
         zarray_destroy(detections);
    }

    Size PictureHandler::getImageSize() {
        return this->size;

    }

    bool PictureHandler::clearPointlist() {
        this->pointMap.clear();
        return true;
    }

vector<Point2f> PictureHandler::getPoint2fVec(){
    return this->point2fVec;
}

