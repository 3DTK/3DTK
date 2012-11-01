/*
 * PMD implementation
 *
 * Copyright (C) Stanislav Serebryakov
 *
 * Released under the GPL version 3.
 *
 */

#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif

#include "pmdsdk2.h"

#include <stdlib.h>
#include <stdio.h>

#include "cvpmd.h"
#include <inttypes.h>

/** TODO
 * inten max amplitude
 * error handling
 */

PMD *initPMD(const char* plugin, const char *ip) {
    PMD *io = (PMD*)malloc(sizeof(PMD));

    if(0 != pmdOpen (&io->hnd, plugin, ip, plugin, "")) {
        fprintf(stderr, "ERROR: could not connect!\n");
        exit(1);
    }

    //pmdUpdate(io->hnd); 
    //pmdGetSourceDataDescription(io->hnd, &io->dd);
    io->dd.std.numColumns = 64;
    io->dd.std.numRows = 50; //FIXME!

    io->data = (float*)malloc(io->dd.std.numColumns * io->dd.std.numRows * sizeof(float));
 
    return io;
}

float *pmdDataPtr(PMD *p) {
    return p->data;
}

void releasePMD(PMD **pmd) {
    pmdClose((*pmd)->hnd);
    free((*pmd)->data);
    free(*pmd);
    *pmd = 0;
}


static float max(const PMD *p) {    
    float max = 0.0f;
    for(unsigned int k = 0; k < p->dd.std.numRows*p->dd.std.numColumns; k++) {
        if(p->data[k] > max) max = p->data[k];
    }
    //printf("max = %f\n", max);
    return max;
}

//static inline void setPix(const IplImage *m, const int c, const int r, const char v) { m->imageData[r*m->width + c] = v; }


CvSize pmdGetSize(const PMD *p) {
    return cvSize(p->dd.std.numColumns, p->dd.std.numRows);
}


IplImage *toIplImage(const PMD *p, IplImage *img = 0) {
    int cols = p->dd.std.numColumns;
    int rows = p->dd.std.numRows;
    
    IplImage *imgp;
    if(!img) imgp = cvCreateImage(pmdGetSize(p), 8, 1);
    else imgp = img;

    //FIXME: scaled!
    float m = max(p);
    for(int r = 0; r < rows; r++) {
        for(int c = 0; c < cols; c++) {
            //FIXME: mess with the rows and colums
            CV_IMAGE_ELEM(imgp, uint8_t, r, c) =  (uint8_t) 255.0f * p->data[r*cols + c] / m;
        }
    }

    return imgp;
}

/*CvPoint3D32f unionVec(const CvPoint uv, const CvMat *intrinsic) {
    //TODO: not defined yet
    // with this function pmdProjectToCartesian would look like:
    // pmdProjectToCartesian pt depth mat = depth * unionVec pt mat
     
    return cvPoint3D32f(0,0,0);
}*/


static inline CvPoint3D32f pmdProjectToCartesian(const CvMat *intrinsicMatrix, const CvPoint2D32f uv, const float dist) {
  /* Lukas Dierks and Jan Wuelfing's projectToCartesian */
  float fx = cvmGet(intrinsicMatrix, 0, 0);
  float cx = cvmGet(intrinsicMatrix, 0, 2);
  float fy = cvmGet(intrinsicMatrix, 1, 1);
  float cy = cvmGet(intrinsicMatrix, 1, 2);
  
  float u = uv.x;
  float v = uv.y;
  float r = dist;
  float u2 = u*u;
  float v2 = v*v;

  float cx2 = cx*cx;
  float cy2 = cy*cy;
  float fx2 = fx*fx;
  float fy2 = fy*fy;
  
  // Reverse Projection
  float squareroot = sqrt(
					 cy2 * fx2 +
					 cx2 * fy2 +
					 fx2 * fy2 -
					 2 * cx * fy2 * u +
					 fy2 * u2 -
					 2 * cy * fx2 * v +
					 fx2 * v2
					 );
   
  return cvPoint3D32f((fy * r * (cx - u)) / squareroot,
	                  (fx * r * (cy - v)) / squareroot,
	                  (fx * fy * r) / squareroot);
}

CvPoint3D32f *cvProjectArrayToCartesian( const CvMat *intrinsicMatrix
                                       , const CvPoint2D32f *pts, const int ptsCnt
                                       , CvPoint3D32f *unitVs) {

    for(int i = 0; i < ptsCnt; i++)
        unitVs[i] = pmdProjectToCartesian(intrinsicMatrix, pts[i], 1.0);
    return unitVs;
}


CvPoint3D32f **pmdProjectArrayToCartesian(const PMD *p, const CvMat *intrinsicMatrix, CvPoint3D32f **pts) {
  for(unsigned int i = 0; i < p->dd.std.numRows; i++)
   for(unsigned int j = 0; j < p->dd.std.numColumns; j++)
       pts[i][j] = pmdProjectToCartesian(intrinsicMatrix, cvPoint2D32f(i,j), p->data[j*p->dd.std.numColumns + i]);
  return pts;
}



IplImage *pmdQueryImage(PMD *p, IplImage *img = 0) {
    pmdGetIntensities(p->hnd, p->data, p->dd.std.numColumns * p->dd.std.numRows * sizeof(float));
    return toIplImage(p, img);
}

IplImage *pmdQueryImageAsync(PMD *p, IplImage *img = 0) {
    pmdGetIntensitiesAsync(p->hnd, p->data, p->dd.std.numColumns * p->dd.std.numRows * sizeof(float));
    return toIplImage(p, img);
}


void pmdRetriveDistances(PMD *p) {
    pmdGetDistances(p->hnd, p->data, p->dd.std.numColumns * p->dd.std.numRows * sizeof(float));
    return;
}
void pmdRetriveDistancesAsync(PMD *p) {
    pmdGetDistancesAsync(p->hnd, p->data, p->dd.std.numColumns * p->dd.std.numRows * sizeof(float));
    return;
}

IplImage *pmdQueryDistances(PMD *p,  IplImage *img = 0) {
    pmdGetDistances(p->hnd, p->data, p->dd.std.numColumns * p->dd.std.numRows * sizeof(float));
    return toIplImage(p, img);

}

IplImage *pmdQueryDistancesAsync(PMD *p,  IplImage *img = 0) {
    pmdGetDistancesAsync(p->hnd, p->data, p->dd.std.numColumns * p->dd.std.numRows * sizeof(float));
    return toIplImage(p, img);

}

IplImage *pmdQueryAmplitudes(PMD *p, IplImage *img = 0) {
    pmdGetAmplitudesAsync(p->hnd, p->data, p->dd.std.numColumns * p->dd.std.numRows * sizeof(float));
    return toIplImage(p, img);
}

IplImage *pmdQueryAmplitudesAsync(PMD *p, IplImage *img = 0) {
    pmdGetAmplitudes(p->hnd, p->data, p->dd.std.numColumns * p->dd.std.numRows * sizeof(float));
    return toIplImage(p, img);
}
