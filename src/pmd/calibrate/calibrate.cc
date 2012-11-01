/*
 * calibrate implementation
 *
 * Copyright (C) Stanislav Serebryakov
 *
 * Released under the GPL version 3.
 *
 */

#include <stdio.h>

#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#include <highgui.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif


void usage(char *progName) {
    printf("%s <board-size-x> <board-size-y> <square-size> <images-list>\n", progName);
    printf("\twhere board-size-x and y is count of *inner* corners of the booard");
    printf("i.e.: %s 6 4 0.04 image*\n", progName);
    printf("Use more then ten images.\nPress space bar to proceed.\n");
}

void calibrate( CvMat *intrinsic, CvMat *distortion, CvSize imgSz, CvSize boardSz
              , double boardSide, CvPoint2D32f **corners, int boardsCnt) {

    int totalPoints = boardSz.width * boardSz.height;
    // object points (model)
    CvMat *objPts = cvCreateMat(totalPoints * boardsCnt, 3, CV_32FC1);
    // found points
    CvMat *imgPts = cvCreateMat(totalPoints * boardsCnt, 2, CV_32FC1);
    // points count
    CvMat *ptsCnt = cvCreateMat(boardsCnt, 1, CV_32SC1);

    // copy corners to matrix and fill model matrix
    for(int i = 0; i < boardsCnt; i++) {
        for(int j = 0; j < totalPoints; j++) {
             int s = i * totalPoints;
             CV_MAT_ELEM(*imgPts, float, s+j, 0)   = corners[i][j].x;
             CV_MAT_ELEM(*imgPts, float, s+j, 1)   = corners[i][j].y;
             CV_MAT_ELEM(*objPts, float, s+j, 0)   = boardSide * (j / boardSz.width);
             CV_MAT_ELEM(*objPts, float, s+j, 1)   = boardSide * (j % boardSz.width);
             CV_MAT_ELEM(*objPts, float, s+j, 2)   = 0.0f;
        }
        CV_MAT_ELEM(*ptsCnt, int, i, 0) = totalPoints;
    }

    // initial guess
    CV_MAT_ELEM(*intrinsic, float, 0, 0) = 1.0f;
    CV_MAT_ELEM(*intrinsic, float, 1, 1) = 1.0f;

    cvCalibrateCamera2( objPts, imgPts, ptsCnt, imgSz
                      , intrinsic, distortion
                      , NULL, NULL, 0 );
    return;
}


int main(int argc, char **argv)
{
    if(argc < 5) {
        usage(argv[0]);
        exit(1);
    }
 
    int patx = atoi(argv[1]);
    int paty = atoi(argv[2]);
    CvSize boardSz = cvSize(patx, paty);
    int loadImageCnt = argc - 4;
    int cornersTotal = boardSz.width * boardSz.height;
    float boardSide = atof(argv[3]);;
    char **images = &argv[4];

    // intrinsic matrices and ditortion params
    CvMat *intrinsic = cvCreateMat(3, 3, CV_32F);
    CvMat *distortion = cvCreateMat(1, 4, CV_32F);

    
    IplImage *img = cvLoadImage(images[0], CV_LOAD_IMAGE_GRAYSCALE);
    IplImage *imgColor = cvCreateImage(cvGetSize(img), 8, 3);
    
    int subpixel;
    if(cvGetSize(img).width < 300) {
        subpixel = 4;
    }
    else {
        subpixel = 11;
    }

    int imagesWithBoard = 0;

    CvPoint2D32f **corners = (CvPoint2D32f**)calloc(100, sizeof(CvPoint2D32f*));
    corners[0] = (CvPoint2D32f*) malloc(cornersTotal * sizeof(CvPoint2D32f));


    cvNamedWindow("Cam", 0);

    for(int imagesLoaded = 0; imagesLoaded < loadImageCnt; imagesLoaded++) {
        img = cvLoadImage(images[imagesLoaded], CV_LOAD_IMAGE_GRAYSCALE);

        cvCvtColor(img, imgColor, CV_GRAY2BGR);

        int cornersCount;         
        
        int found = cvFindChessboardCorners(img, boardSz, corners[imagesWithBoard], 
                &cornersCount, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
        if(found) cvFindCornerSubPix(img, corners[imagesWithBoard], cornersCount, cvSize(subpixel, subpixel), cvSize(-1,-1), 
                           cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));

        if(found && (cornersCount == cornersTotal)) {
            cvDrawChessboardCorners(imgColor, boardSz, corners[imagesWithBoard], cornersCount, found);
            imagesWithBoard++;
            corners[imagesWithBoard] = (CvPoint2D32f*) malloc(cornersTotal * sizeof(CvPoint2D32f));
        }
        cvShowImage("Cam", imgColor);
        cvWaitKey(0);

    }

    cvDestroyAllWindows();

    printf("calibrating...\n");
    fflush(stdout);
    
    //TODO: can be started in parallel to watch calibration on image undistortion :)
    calibrate(intrinsic, distortion, cvGetSize(img), boardSz, boardSide, corners, imagesWithBoard); 

    // save to xml files
    cvSave("./intrinsic.xml", intrinsic);
    cvSave("./distortion.xml", distortion);
    printf("matrices saved to xml files.\n");

    // let OS clean all images and matrices
    return 0;
}


/*void usage(char *progName) {
    printf( "usage:\n \
             %s <x> <y> <camera-id> [-s]\n \
                          \tcamera-id is v4l id or -1 for pmd cam \
             \tpress space or 'a' to grab image\n \
             \tpress 'n' to skip grabbed frame\n \
             \tpress 'c' to finish frame grabbing start calibration\n \
             or:\n \
             %s <x> <y> <image1cam.jpg, image1pmd.jpg, image2cam.png...>\n", progName, progName);
    fflush(stdout);
}*/

