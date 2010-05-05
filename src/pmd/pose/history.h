#pragma once

#include <cxtypes.h>

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
