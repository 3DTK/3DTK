#pragma once

#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cxcore.h>
#include <highgui.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif

#include <libconfig.h>

#include "cvpmd.h"

#include "slam6d/icp6Dminimizer.h"

struct PMDFiles {
    FILE *i; // intens
    FILE *a; // ampl
    FILE *p; // 3d pts
    FILE *h; // pmd image headers
};

struct TrackingSettings {
    int winSz;
    double quality;
    int minFeatures;
    int maxFeatures;
    int minDist;
    IplImage *pyr;
    IplImage *pyrPrev;
    IplImage *iCamPrev;
    CvPoint2D32f *pts[2];
    char *ptsStatus;
    int trackingFlags;
};

struct PMDCam {
    PMD *_pmd;
    CvCapture *_capture;
    PMDFiles _f;
    CvMat *intrinsicPMD;
    CvMat *intrinsicCam;
    CvMat *distortionPMD;
    CvMat *distortionCam;
    CvMat *rotation;
    CvMat *translation;
    // Undistortion maps
    IplImage *_mapXCam;
    IplImage *_mapYCam;
    IplImage *_mapXPMD;
    IplImage *_mapYPMD;
    // Data
    IplImage *_iCamColorUBuffer;
    IplImage *iCamColor;
    IplImage *iCam;
    IplImage *_iCamColorU;
    IplImage *iPMDI;
    IplImage *_iPMDIU;
    IplImage *iPMDA;
    IplImage *_iPMDAU;
    CvPoint3D32f **pts;
    unsigned int timestamp;
    unsigned int timestampUsec;
    // Pose Estimation
    icp6Dminimizer *icp;
    int minPts4Pose;
    double maxError;
    int minConsensusPts;
    int savePoses;   
    // Settings
    ImageHeaderInformation *header;
    int synchronous;
    int hybrid;
    int _offlineMode;
    int historyLen;
    TrackingSettings _track;
    double sigmaDepth;
    double sigmaColor;
    double dpThreshold;

};

PMDCam *initPMDCam(const char *confPath);
int grabData(PMDCam *pmdc);


