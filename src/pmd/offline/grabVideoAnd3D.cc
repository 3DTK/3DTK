/*
 * grabVideoAnd3D implementation
 *
 * Copyright (C) Stanislav Serebryakov
 *
 * Released under the GPL version 3.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#include <highgui.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif

#include "pmdsdk2.h"
#include "cvpmd.h"

int main(int argc, char **argv)
{
    int totalFrames = 0;
    int ui = 1;
    printf("%i\n", argc);
    if(argc > 1) {
        totalFrames = atoi(argv[1]);
        ui = 0;
    }
    if(argc > 2) ui = 1;

    //FIXME: here is lots of old code, should be removed


    PMD *pmd = initPMD("../o3d.L32.pcp", "192.168.0.69");
    
    CvCapture *capture = cvCaptureFromCAM(1); //FIXME: should be passed via argc

    CvSize pmdSz = pmdGetSize(pmd);
    printf("pmd sz: %i x %i\n", pmdSz.width, pmdSz.height);
    IplImage *imgCamColor = cvQueryFrame(capture);
    IplImage *imgPMD = cvCreateImage(pmdGetSize(pmd), 8, 1);
    IplImage *imgPMDA = cvCreateImage(cvGetSize(imgPMD), 8, 1);
    IplImage *imgCam = cvCreateImage(cvGetSize(imgCamColor), 8, 1);
    CvPoint3D32f **pmdPts = (CvPoint3D32f**) cvAlloc(pmdSz.height * sizeof(CvPoint3D32f*));
    for(int i = 0; i < pmdSz.height; i++) pmdPts[i] = (CvPoint3D32f*) cvAlloc(pmdSz.width * sizeof(CvPoint3D32f));

    CvMat *intrinsicCam  = (CvMat*)cvLoad("../intrinsic-cam-6x4.xml");
    CvMat *distortionCam = (CvMat*)cvLoad("../distortion-cam-6x4.xml");
    CvMat *intrinsicPMD  = (CvMat*)cvLoad("../intrinsic-pmd-6x4.xml");
    CvMat *distortionPMD = (CvMat*)cvLoad("../distortion-pmd-6x4.xml");
    if(!intrinsicCam || !distortionCam|| !intrinsicPMD || !distortionPMD) {
        fprintf(stderr, "ERROR: can't load intrinsic and/or distortion xml files!\n");
        return 1;
    }


//    FILE *vPMDA = fopen("./s001.arv", "wb"); // ala dot Amplitudes Raw Video NOTE: i'm not sure it is amplitude :P
//    FILE *vPMDAf = fopen("./s001.farv", "wb"); // same but Floating 
    FILE *vPMDI = fopen("./s001.irv", "wb"); // same but Intensities
    FILE *vPMDIf = fopen("./s001.firv", "wb");
    FILE *headers = fopen("./s001.head", "wb");
    CvVideoWriter *vCam = cvCreateVideoWriter( "./s001.avi"
                                             , CV_FOURCC('D', 'I', 'V', 'X') 
                                             , 25, cvGetSize(imgCam), 1);

    FILE *pmdPtsFile = fopen("./s001.3dp", "w");

    if(ui) {
        cvNamedWindow("Cam", 0);
        cvNamedWindow("PMD", 1);
    }

    printf("DEBUG: init done, going to grab %i frames.\n", totalFrames);

    int frames = 0;
    while(1) {
        frames++;
        if(0 == frames % 100) printf("%i frames grabbed...\n", frames);
        // Image retriving 
        pmdUpdate(pmd->hnd);
        imgCamColor = cvQueryFrame(capture);
       
        pmdQueryImageAsync(pmd, imgPMD);
        fwrite(pmdDataPtr(pmd), sizeof(float), pmdSz.width*pmdSz.height, vPMDIf);

        //pmdQueryAmplitudesAsync(pmd, imgPMDA);
        //fwrite(pmdDataPtr(pmd), sizeof(float), pmdSz.width*pmdSz.height, vPMDAf);


        pmdRetriveDistancesAsync(pmd);
        pmdProjectArrayToCartesian(pmd, intrinsicPMD, pmdPts);

        ImageHeaderInformation *header = retriveHeader();

        
        if(ui) {
            cvShowImage("Cam", imgCamColor);
            cvShowImage("PMD", imgPMD);
        }

        //FIXME: order col/str
        for(int i = 0; i < pmdSz.height; i++) 
            fwrite(pmdPts[i], sizeof(CvPoint3D32f), pmdSz.width, pmdPtsFile);

        fwrite(imgPMD->imageData, sizeof(char), pmdSz.width*pmdSz.height, vPMDI);
        //fwrite(imgPMDA->imageData, sizeof(char), pmdSz.width*pmdSz.height, vPMDA);
        fwrite(header, sizeof(ImageHeaderInformation), 1, headers);

        cvWriteFrame(vCam, imgCamColor);

        if(totalFrames && (frames >= totalFrames)) break;
        if(ui) if(27 == cvWaitKey(5)) break;
    }
//    fclose(vPMDA);
//    fclose(vPMDAf);
    fclose(vPMDI);
    fclose(vPMDIf);
    fclose(pmdPtsFile);
    fclose(headers);
    printf("grabbed %i frames.\n", frames);
    printf("See s001* files (you want to rename them).\n");
}

