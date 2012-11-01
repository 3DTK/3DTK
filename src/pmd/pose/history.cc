/*
 * history implementation
 *
 * Copyright (C) Stanislav Serebryakov
 *
 * Released under the GPL version 3.
 *
 */

#include "history.h"
#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif

Frame *allocFrame3DData(CvSize pmdSz) {
    Frame *f = (Frame*)cvAlloc(sizeof(Frame));

    f->sz = pmdSz;
    f->img = 0;

    f->trn = cvCreateMat(3, 1, CV_32FC1);
    f->rot = cvCreateMat(3, 1, CV_32FC1);

    f->status = (CvPoint**) cvAlloc(pmdSz.height * sizeof(CvPoint*));
    f->pts = (CvPoint3D32f**) cvAlloc(pmdSz.height * sizeof(CvPoint3D32f*));
    for(int i = 0; i < pmdSz.height; i++) {
        f->status[i] = (CvPoint*) cvAlloc(pmdSz.width * sizeof(CvPoint));
        f->pts[i] = (CvPoint3D32f*) cvAlloc(pmdSz.width * sizeof(CvPoint3D32f));
    }

    return f;
}

void fillFrame(Frame *f, IplImage *img, CvSize pmdSz, CvPoint3D32f **pts, CvPoint **status
              , CvMat *rot, CvMat *trn, double alignError) {

    assert(pmdSz.width == f->sz.width || pmdSz.height == f->sz.height);

    if(f->img) cvCopy(img, f->img, NULL);
    else f->img = cvCloneImage(img);
    f->alignError = alignError;        

    for(int j = 0; j < pmdSz.width; j++)
      for(int i = 0; i < pmdSz.height; i++) {
        f->status[i][j] =  status[i][j];
        f->pts[i][j] = pts[i][j];
        }

    cvCopy(rot, f->rot, NULL);
    cvCopy(trn, f->trn, NULL);
}

void releaseFrame(Frame **f) {
    Frame *t = *f;
    cvReleaseImage(&t->img);
    cvReleaseMat(&t->rot);
    cvReleaseMat(&t->trn);
    for(int i = 0; i < t->sz.height; i++) {
        cvFree(&t->pts[i]); 
        cvFree(&t->status[i]);
    }
    cvFree(&t->pts);
    cvFree(&t->status);
    *f = NULL;
}


History *createHistory() {
    History *h = (History*)cvAlloc(sizeof(History));
    h->frame = 0;
    h->prev = 0; // essential!!!
    return h;
}

History *addFrame(History *h, Frame *f) {
    if(!h->frame) { // first frame, FIXME: should be out here
        h->prev = 0; // ensure
        h->frame = f;
        return h;
    }
    // else
    History *n = (History*)cvAlloc(sizeof(History));
    n->prev = h;
    n->frame = f;
    return n;
}

void releaseTail(History *h) {
    History *tmp = h->prev;    
    History *prev = 0;
    h->prev = 0;
    while(tmp) {
       prev = tmp->prev;       
       releaseFrame(&tmp->frame);
       cvFree(&tmp);
       tmp = prev;
    }
}

void checkHistoryLen(History *h, int maxLen) {
    History *histI = h;
    for(int i = 0; histI->prev; i++) 
        if(i > maxLen-2) // -2 because I release *prev* elemnents
            releaseTail(histI);
        else histI = histI->prev;
}

