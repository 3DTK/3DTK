/*
 * extrinsic implementation
 *
 * Copyright (C) Stanislav Serebryakov
 *
 * Released under the GPL version 3.
 *
 */

#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif


int main(int argc, char **argv) {

    if(argc < 3) {
        printf("extrinic: x y img1 img2\n");
        exit(1);
    }
    // extrinic params: rotation and translation
    CvMat *rotCam   = cvCreateMat(1, 3, CV_32FC1);
    CvMat *transCam = cvCreateMat(1, 3, CV_32FC1);
    CvMat *rotPMD   = cvCreateMat(1, 3, CV_32FC1);
    CvMat *transPMD = cvCreateMat(1, 3, CV_32FC1);

    CvSize boardSz = cvSize(atoi(argv[1]), atoi(argv[2]));
    int totalPoints = boardSz.width*boardSz.height;
    double boardSide = 0.04;
    CvMat *objPts = cvCreateMat(totalPoints, 3, CV_32FC1);
    CvPoint2D32f *cornersCam = (CvPoint2D32f*)cvAlloc(totalPoints * sizeof(CvPoint2D32f));
    CvMat imgPtsCam = cvMat(totalPoints, 1, CV_32FC2, cornersCam);
    CvPoint2D32f *cornersPMD = (CvPoint2D32f*)cvAlloc(totalPoints * sizeof(CvPoint2D32f));
    CvMat imgPtsPMD = cvMat(totalPoints, 1, CV_32FC2, cornersPMD);

    for(int i = 0; i < totalPoints; i++) {
        CV_MAT_ELEM(*objPts, float, i, 0)   = boardSide * (i / boardSz.width);
        CV_MAT_ELEM(*objPts, float, i, 1)   = boardSide * (i % boardSz.width);
        CV_MAT_ELEM(*objPts, float, i, 2)   = 0.0f;
    }

    IplImage *imgPMD = cvLoadImage(argv[3], CV_LOAD_IMAGE_GRAYSCALE);
    IplImage *imgPMDU = cvCreateImage(cvGetSize(imgPMD), 8, 1);
    IplImage *imgCam = cvLoadImage(argv[4], CV_LOAD_IMAGE_GRAYSCALE);
    IplImage *imgCamU = cvCreateImage(cvGetSize(imgCam), 8, 1);

    CvMat *intrinsicsCam = (CvMat*)cvLoad("../intrinsic-cam-6x4.xml");
    CvMat *intrinsicsPMD = (CvMat*)cvLoad("../intrinsic-pmd-6x4.xml");
    CvMat *distortionCam = (CvMat*)cvLoad("../distortion-cam-6x4.xml");
    CvMat *distortionPMD = (CvMat*)cvLoad("../distortion-pmd-6x4.xml");

    cvUndistort2(imgPMD, imgPMDU, intrinsicsPMD, distortionPMD);
    cvUndistort2(imgCam, imgCamU, intrinsicsCam, distortionCam);

    int cornersCountCam, cornersCountPMD;
    int foundPMD = cvFindChessboardCorners(imgPMDU, boardSz, cornersPMD, 
                &cornersCountPMD, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    if(foundPMD) {
        cvFindCornerSubPix(imgPMDU, cornersPMD, cornersCountPMD, cvSize(2, 2), cvSize(-1,-1), 
                           cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        printf("foundPMD\n");
    }
    int foundCam = cvFindChessboardCorners(imgCamU, boardSz, cornersCam, 
                &cornersCountCam, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
    if(foundPMD) {
        cvFindCornerSubPix(imgCamU, cornersCam, cornersCountCam, cvSize(11, 11), cvSize(-1,-1), 
                           cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        printf("foundCam\n");
    }

    cvNamedWindow("c", 0);
    cvNamedWindow("p", 0);

    cvShowImage("c", imgCamU);
    cvShowImage("p", imgPMDU);
    cvWaitKey(0);


    if(foundCam) cvFindExtrinsicCameraParams2(objPts, &imgPtsCam, intrinsicsCam, distortionCam, rotCam, transCam);
    if(foundPMD) cvFindExtrinsicCameraParams2(objPts, &imgPtsPMD, intrinsicsPMD, distortionPMD, rotPMD, transPMD);

    cvSave("./rotcam.xml", rotCam);
    cvSave("./rotpmd.xml", rotPMD);
    cvSave("./transcam.xml", transCam);
    cvSave("./transpmd.xml", transPMD);
}
