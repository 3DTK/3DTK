/*
 * grabFramesPMD implementation
 *
 * Copyright (C) Stanislav Serebryakov
 *
 * Released under the GPL version 3.
 *
 */

#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#include <highgui.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif

#include <stdio.h>
#include <stdlib.h>

#include "pmdsdk2.h"
#include "cvpmd.h"


/* TODO:
* flags:
*  subpixel
*  camera id
*  flip x and y
*  pmd mode?
*/

void usage(char *progName) {
    printf("%s <board-size-x> <board-size-y> <camera-id>\n", progName);
    printf("i.e.: %s 6 4 0\n", progName);
    printf("press space to detect chessboard and (again) to proceed.\n");
}

int main(int argc, char **argv) {

    if(argc < 4) {
        usage(argv[0]);
        exit(1);
    }

    
    PMD *pmd = initPMD("../o3d.L32.pcp", "192.168.0.69");
    IplImage *img = cvCreateImage(pmdGetSize(pmd), 8, 1);
    pmdUpdate(pmd->hnd);
    pmdQueryImage(pmd, img);

    IplImage *imgColor = cvCreateImage(cvGetSize(img), 8, 3);

    int patx = atoi(argv[1]);
    int paty = atoi(argv[2]);
    CvSize patternSize = cvSize(patx, paty);

    int cornersTotal  = patternSize.width * patternSize.height;

    CvPoint2D32f *corners = (CvPoint2D32f*) malloc(cornersTotal * sizeof(CvPoint2D32f));

    cvNamedWindow("Camera", 0);

    int imageCnt = 0;
    bool grabFrame = false;
    char *filename = (char*) malloc(11 * sizeof(char));


    while(1) {
       pmdUpdate(pmd->hnd);
       pmdQueryImage(pmd, img);

       cvFlip(img, 0, 1); // flips image around the x-axes
       



       if(grabFrame) {
            /* ----- Chessboard detection ----- 
              -------------------------------- */
            int cornersCount; // should be the same for wcam and pmd and equal totalCorners
        
            int found = cvFindChessboardCorners(img, patternSize, corners, 
                          &cornersCount, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
            cvFindCornerSubPix(img, corners, cornersCount, cvSize(4,4), cvSize(-1,-1), 
                           cvTermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
 
            if(found && (cornersCount == cornersTotal)) {
                cvCvtColor(img, imgColor, CV_GRAY2BGR);
                cvDrawChessboardCorners(imgColor, patternSize, corners, cornersCount, found);
                cvShowImage("Camera", imgColor);
                printf("grab?\n");
                int k = cvWaitKey(0);
                if(k == (int)' ') {
                    imageCnt++;
                    sprintf(filename, "image%02i.jpg", imageCnt); //TODO:try png
                    printf("saved %s.\n", filename);
                    cvSaveImage(filename, img);
                    grabFrame = false;
                    continue;
                }
            }
       }

       cvShowImage("Camera", img);
       if((int)' ' == cvWaitKey(5)) grabFrame = true;
    }

    return 0;
}

