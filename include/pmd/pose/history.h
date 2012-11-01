#pragma once

#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cxcore.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif


struct Frame {
    CvMat *trn;
    CvMat *rot;
    IplImage *img;
    CvPoint **status;
    CvPoint3D32f **pts;
    CvSize sz;
    double alignError;
};

Frame *allocFrame3DData(CvSize pmdSz);
void fillFrame(Frame *f, IplImage *img, CvSize pmdSz, CvPoint3D32f **pts, CvPoint **status
              , CvMat *rot, CvMat *trn, double alignError);
void releaseFrame(Frame **f);

struct History {
    Frame *frame;
    History *prev;
};

History *createHistory(); 
History *addFrame(History *h, Frame *f);
void releaseTail(History *h);
void checkHistoryLen(History *h, int maxLen);
