/**
 * @file cvpmd.h
 * @brief pmdaccess wrappers, PMD -> OpenCV marshalling and a bit of math.
 * @author Stanislav `Cfr` Serebryakov
 */
#pragma once

#include "pmdsdk2.h"
#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cxcore.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif

/**
 * @brief Structure for PMD IO, see initPMD.
 */
typedef struct PMD {
    PMDHandle hnd;          /**< handle for pmd IO */
    float *data;            /**< image quering buffer */
    PMDDataDescription dd;  /**< contains image size, integration time, etc */
} PMD;

/**
 * PMD Constructor
 * @param *plugin IO plugin name i.e. o3d.L32.pcp
 * @param *ip PMD camera IP address
 * @return initialized PMD struct
 */
PMD *initPMD(const char* plugin, const char *ip);

/**
 * Destructor
 */
void releasePMD(PMD **pmd);


/**
 * Returns pointer to the data (used to save float data for offline mode)
 */
float *pmdDataPtr(PMD *p);


/**
 * Converts PMD data (stored in PMD struct) to provided IplImage (new will be created if NULL passed)
 * @param *p initialized PMD struct with grabbed data (i.e. pmdGetIntensities(p->hnd, p->data, p->dd.std.numColumns * p->dd.std.numRows * sizeof(float)))
 * @param *img destination IplImage  (default: 0, new one will be created)
 * @return generated IplImage
 */
IplImage *toIplImage(const PMD *p, IplImage *img);

/**
 * Converts PMD distance point to cartesian coords with intrinsic matrix
 * @param uv PMD depth point coords (row, col) //FIXME: order!
 * @param dist PMD distance for given point
 * @param intrinsicMatrix PMD camera' intrinsic matrix (from calibration)
 * @return 3D Point in OpenCV format (CvPoint3D32f)
 */

CvPoint3D32f *cvProjectArrayToCartesian( const CvMat *intrinsicMatrix
                                       , const CvPoint2D32f *pts, const int ptsCnt
                                       , CvPoint3D32f *unitVs);


CvPoint3D32f **pmdProjectArrayToCartesian(const PMD *p, const CvMat *intrinsicMatrix, CvPoint3D32f **pts);

/**
 * Get pmd camera' image size
 * @param *p initialized PMD struct
 * @return image size in OpenCV format (CvSize)
 */
CvSize pmdGetSize(const PMD *p);

/**
 * Query PMD inrensities image. Dont forget to call pmdUpdate before quering!
 * @param *p initialized PMD struct
 * @param *img destination IplImage (defaul: 0, new one will be created if null passed)
 * @return image in OpenCV format (IplImage)
 */
IplImage *pmdQueryImage(PMD *p, IplImage *img); 
IplImage *pmdQueryImageAsync(PMD *p, IplImage *img); 

/**
 * Store PMD distances (inside p->data)
 */
void pmdRetriveDistances(PMD *p);
void pmdRetriveDistancesAsync(PMD *p);


/**
 * Query PMD distance image in OpenCV format
 */
IplImage *pmdQueryDistances(PMD *p,  IplImage *img);
IplImage *pmdQueryDistancesAsync(PMD *p,  IplImage *img);

/**
 * Query PMD aplitudes image in OpenCV format (might be used as belief measure)
 */
IplImage *pmdQueryAmplitudes(PMD *p, IplImage *img);
IplImage *pmdQueryAmplitudesAsync(PMD *p, IplImage *img);

