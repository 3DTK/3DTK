/*
 * pose implementation
 *
 * Copyright (C) Stanislav Serebryakov
 *
 * Released under the GPL version 3.
 *
 */

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <math.h>

// OpenCV
#if (defined(_WIN32) || defined(__WIN32__) || defined(__TOS_WIN__) || defined(__WINDOWS__) || (defined(__APPLE__) & defined(__MACH__)))
#include <cv.h>
#include <highgui.h>
#elif (CV_MAJOR_VERSION == 2) && (CV_MINOR_VERSION < 2)
#include <opencv/cv.h>
#else
#include <opencv2/opencv.hpp>
#endif

// GL: GLFW (window etc, ala glut) and FTGL (text rendering)
#include <GL/glfw.h>
#include <ftgl.h>

// PMD-related stuff
#include "pmdsdk2.h"
#include "cvpmd.h"
#include "pmdWrap.h"
#include "history.h"

// SLAM6D
#include "slam6d/point.h"
#include "slam6d/ptpair.h"
#include <vector>


FTGLPixmapFont *font; // opengl text rendring

void usage(char* progName) {
    printf("usage: %s [options]\n", progName);
    printf("options:\n \
\t--ui 0|1\t disable|enable ui (default: 1)\n\
\t--gl 0|1\t disable|enable 3d rendering (default: 1)\n\
\t--cfg file\t parse configuration from file (default: pmdc.conf)\n\
\t--help\t\t display this help.\n");
}



void render( History *history, CvPoint3D32f *camPts,  int trPtsCnt
           , CvMat *rot,  CvMat *trn
           , CvPoint3D32f **camPts3D, char **pts3DStatus
           , int reprojected, bool poseEstimated);

/*
   reprojects PMD-aquired 3D points on the web cam image
*/

//FIXME this function should be dispached >_>
int projectImageToPMD( CvMat *rot,  CvMat *trn,  CvMat *intrinsic
                     , CvPoint3D32f **pmdPts,  CvSize pmdSz
                     , CvSize camSz, CvPoint **pmd2imgIdxs
                     , CvPoint2D32f *trackedPts 
                     , int *trPtsCnt, CvPoint3D32f *trackedPts3D, char *pts3DStatus
                     , CvPoint *tr2pmdIdxs) {
    /* TODO:
       assert null image, not clean image, bad image depth
       */
    // pmd point's matrix
    CvMat *pt = cvCreateMat(3, 1, CV_32FC1);
    // point in cam coorinate system
    CvMat *reprojPt = cvCreateMat(3, 1, CV_32FC1);
    // points on cam screen
    CvMat *camPt3 = cvCreateMat(3, 1, CV_32FC1); 
    CvMat *camPt  = cvCreateMat(2, 1, CV_32FC1); 
    // rotation matrix
    CvMat *rotMat = cvCreateMat(3, 3, CV_32FC1);
    cvRodrigues2(rot, rotMat, NULL);

    // clear previous found tracked pts
    for(int k = 0; k < *trPtsCnt; k++) pts3DStatus[k] = 0;
    
    //int l = 0; // count corresponding points
    int reprojected = 0;

    for(int j = 0; j < pmdSz.width; j++)
      for(int i = 0; i < pmdSz.height; i++) {
          if( fabs(pmdPts[i][j].x) > 20.0
           || fabs(pmdPts[i][j].y) > 20.0 
           || fabs(pmdPts[i][j].z) > 20.0) continue; //FIXME: more realistic limit, status = false
          // convert to mat 
          CV_MAT_ELEM(*pt, float, 0, 0) = (float)pmdPts[i][j].x;
          CV_MAT_ELEM(*pt, float, 1, 0) = (float)pmdPts[i][j].y;
          CV_MAT_ELEM(*pt, float, 2, 0) = (float)pmdPts[i][j].z;
          
          // reproject to the camera coordinate system
          cvMatMulAdd(rotMat, pt, trn, reprojPt);
          // project to the image
          cvMatMul(intrinsic, reprojPt, camPt3);
//          cvConvertPointsHomogenious(camPt3, camPt);
/*          cvProjectPoints2(pt, rot, trn, intrinsic, NULL, camPt
                          , NULL, NULL, NULL, NULL, NULL);*/
          float scale = (float)CV_MAT_ELEM(*camPt3, float, 2, 0);
          float xf = CV_MAT_ELEM(*camPt3, float, 0, 0) / scale;
          float yf = CV_MAT_ELEM(*camPt3, float, 1, 0) / scale;
          int x = (int)xf;
          int y = camSz.height - (int)yf; //FIXME revise coordinates

          if((x < 0) || (x > camSz.width) ||
             (y < 0) || (y > camSz.height)) {
              pmd2imgIdxs[i][j].x = -1;
              pmd2imgIdxs[i][j].y = -1;
              continue;
          } else { // point is projected to cam
            // contains which PMD point is seen from this pixel
            pmd2imgIdxs[i][j].x = x;
            pmd2imgIdxs[i][j].y = y;
          }

          //find tracked 3d points (fused here, can be dispached)
          for(int k = 0; k < *trPtsCnt; k++) 
              if(pts3DStatus[k]) continue; // kill to accumulate (*)
              else 
                if( (abs((trackedPts[k].x - (float)x)) < 3)   //TODO: distance SHOULD be chousen depending on depth!!!
                 && (abs((trackedPts[k].y - (float)y)) < 3)) { //FIXME: hardcoded distance
                    //TODO: we can accumulate points here to get more presise results (*)
                  trackedPts3D[k].x = pmdPts[i][j].x;
                  trackedPts3D[k].y = pmdPts[i][j].y;
                  trackedPts3D[k].z = pmdPts[i][j].z;
                  pts3DStatus[k] = 1;
                  tr2pmdIdxs[k].x = i; //FIXME: cvPoint(i,j)?
                  tr2pmdIdxs[k].y = j;
                  reprojected++;
                  break; // kill to accumulate (*)
              }
      }
 
    cvReleaseMat(&pt);
    cvReleaseMat(&reprojPt);
    cvReleaseMat(&camPt);
    cvReleaseMat(&rotMat);

    return reprojected;
}

static inline float cvPoint3D32fNorm(CvPoint3D32f pt) {
    return sqrt( (pt.x - pt.x)*(pt.x - pt.x)
                +(pt.y - pt.y)*(pt.y - pt.y)
                +(pt.z - pt.z)*(pt.z - pt.z));
}

float dpthS( IplImage *img
           , CvPoint ptI, CvPoint dp
           , CvPoint3D32f **pts 
           , CvPoint ptD, CvSize pmdSz
           , float sD, float sC) {
    CvSize imgSz = cvGetSize(img);
    if( ptD.x + dp.x > pmdSz.width - 1 || ptD.x + dp.x < 0
     || ptD.y + dp.y > pmdSz.height - 1 || ptD.y + dp.y < 0
     || ptI.x + dp.x > imgSz.width - 1 || ptI.x + dp.x < 0
     || ptI.y + dp.y > imgSz.height - 1 || ptI.y + dp.y < 0) return 0.0;

    uchar *pI = &((uchar*) (img->imageData + img->widthStep * ptI.y))[ptI.x*3];
    uchar *pJ = &((uchar*) (img->imageData + img->widthStep * (ptI.y+dp.y)))[(ptI.x+dp.x)*3];
    float dr = (float)pI[2] - (float)pJ[2];
    float dg = (float)pI[1] - (float)pJ[1];
    float db = (float)pI[0] - (float)pJ[0];
    float wij = exp(-(sqrt(dr*dr + dg*dg + db*db))/sC);
    float dpI = cvPoint3D32fNorm(pts[ptD.x][ptD.y]);
    float dpJ = cvPoint3D32fNorm(pts[ptD.x + dp.x][ptD.y + dp.y]); 
    float s = exp(-wij*(dpI - dpJ)*(dpI - dpJ)/sD);
    return s;
}



#define DPTHS(dx, dy) (dpthS( img, idx, cvPoint((dx),(dy)) \
                            , pmdPts \
                            , pmdIdx, pmdSz \
                            , sigmaDepth, sigmaColor))


void outliersDepthAndColor( CvPoint3D32f **pmdPts, IplImage *img, CvSize pmdSz
                          , CvPoint2D32f *trackedPts, int trPtsCnt, CvPoint* tr2pmdIdxs
                          , char *pts3DStatus // this actually return parameter
                          , float sigmaDepth, float threshold, float sigmaColor
                          ) {
   if(threshold < 0.0) return;   
   // depth score outliers removal, see pmdc.conf comments
   for(int k = 0; k < trPtsCnt; k++) {
       //FIXME: check array bounds
       CvPoint pmdIdx = tr2pmdIdxs[k];
       CvPoint idx = cvPointFrom32f(trackedPts[k]);
       float s00, s01, s02;
       s00 = DPTHS(-1, -1); 
       s01 = DPTHS( 0, -1);
       s02 = DPTHS( 1, -1);
       float s10 = DPTHS(-1,  0); 
       float s12 = DPTHS( 1,  0);
       float s20 = DPTHS(-1,  1); 
       float s21 = DPTHS( 0,  1);
       float s22 = DPTHS( 1,  1);
       float score = s00 + s01 + s02 
                   + s10 + 0.0 + s12
                   + s20 + s21 + s22;
       printf("score = %f\n", score);
       if(score < threshold) pts3DStatus[k] = 0;
   }                  

}

int motionMeanAndVariance(CvPoint3D32f **camPts3D, char **pts3DStatus, int trPtsCnt, float *mean, float *var) {    

//    float *mean = meanAndVariance;
//    float *var = &(meanAndVariance[3]);
//    float motion[3];
    float dx, dy, dz;
    float magSq;
    float motionSum = 0; //[3] = {0.0, 0.0, 0.0};
    float motionSqrSum = 0; //[3] = {0.0, 0.0, 0.0};

    int sz = 0; // pairs count    
    for(int i = 0; i < trPtsCnt; i++) 
        if(pts3DStatus[1][i] && pts3DStatus[0][i]) { 
            sz++;

            dx = camPts3D[1][i].x - camPts3D[0][i].x;
            dy = camPts3D[1][i].y - camPts3D[0][i].y;
            dz = camPts3D[1][i].z - camPts3D[0][i].z;

            magSq = dx*dx + dy*dy + dz*dz; 

            motionSum += sqrt(magSq); //FIXME: optimisation, we can use it without sqrt

            motionSqrSum += magSq; //thus, it would be sqr here

/*          motionSum[0] += motion[0];
            motionSum[1] += motion[1];
            motionSum[2] += motion[2];

            motionSqrSum[0] += motion[0]*motion[0];
            motionSqrSum[1] += motion[1]*motion[1];
            motionSqrSum[2] += motion[2]*motion[2];*/
    }

    if(0 == sz) return 0;
    // mean
    *mean = motionSum / sz;
/*  mean[0] = motionSum[0] / (float)sz;
    mean[1] = motionSum[1] / (float)sz;
    mean[2] = motionSum[2] / (float)sz;*/
        
    // variance

    for(int i = 0; i < trPtsCnt; i++) 
        if(pts3DStatus[1][i] && pts3DStatus[0][i]) {                   
            dx = camPts3D[1][i].x - camPts3D[0][i].x;
            dy = camPts3D[1][i].y - camPts3D[0][i].y;
            dz = camPts3D[1][i].z - camPts3D[0][i].z;

            magSq = dx*dx + dy*dy + dz*dz; 

            *var += (magSq - *mean)*(magSq - *mean);
        }

    *var /= sz;
        // = motionSqrSum / sz - (*mean)*(*mean);

/*  var[0] = motionSqrSum[0] / (float)sz - mean[0]*mean[0];
    var[1] = motionSqrSum[1] / (float)sz - mean[1]*mean[1];
    var[2] = motionSqrSum[2] / (float)sz - mean[2]*mean[2];*/
    return 1;
}

void outliersSpeedSigma(CvPoint3D32f **camPts3D, char **pts3DStatus, int trPtsCnt,  float mean, float var) {

        float dx, dy, dz;
        float mag;
        float sigma = sqrt(var);
        for(int i = 0; i < trPtsCnt; i++) 
           if(pts3DStatus[1][i] && pts3DStatus[0][i]) { 
               dx = camPts3D[1][i].x - camPts3D[0][i].x;
               dy = camPts3D[1][i].y - camPts3D[0][i].y;
               dz = camPts3D[1][i].z - camPts3D[0][i].z;

               mag = sqrt(dx*dx + dy*dy + dz*dz); 
                                                                                              
               if(fabs(mag - mean) > sigma) {
                 pts3DStatus[0][i] = 0;
               }
           }
}


int main(int argc, char **argv) {
    // settings
    bool ui = true;
    bool gl = true;
    bool fps = false;
    const char *config = "./pmdc.conf";

    // args parsing
    for(int i = 1; i < argc; i++) {
        if      (!strcmp(argv[i], "--ui"))           ui = (bool) atoi(argv[++i]);
        else if (!strcmp(argv[i], "--gl"))           gl = (bool) atoi(argv[++i]);
        else if (!strcmp(argv[i], "--fps"))          fps = true;
        else if (!strcmp(argv[i], "--cfg"))          config = argv[++i];
        //TODO: config
        else if (!strcmp(argv[i], "--help") 
             || !strcmp(argv[i], "-h")) {
                usage(argv[0]);
                return 1;
        } else {
            fprintf(stderr, "ERROR: unknown flag: %s\n", argv[i]);
            return 1;
        }
    }
    // pose guess
    CvMat *rotMatGuess = cvCreateMat(3,3, CV_32FC1);
    CvMat *rotGuess = cvCreateMat(3, 1, CV_32FC1); 
    CvMat *trnGuess = cvCreateMat(3, 1, CV_32FC1); 


    /***** init device and allocate images *****/
    PMDCam *pmdc = initPMDCam(config);
    CvSize pmdSz = cvGetSize(pmdc->iPMDI);
    CvSize camSz = cvGetSize(pmdc->iCam);
    printf("pose: pmd init done\n");

    /***** essential matrix *****/    

    CvMat *rot = (CvMat*)cvLoad("../essential-rot.xml"); //FIXME: load path from cfg
    CvMat *trn = (CvMat*)cvLoad("../essential-trn.xml");

   /***** LK-tracking *****/
    IplImage *swapTemp;

    int featuresMax = pmdc->_track.maxFeatures;
    int trPtsCnt = 0; // counts found points

    // eigenvalues for GFTT
    IplImage* eig = cvCreateImage(camSz, 32, 1); 
    IplImage* tmp = cvCreateImage(camSz, 32, 1);
    IplImage* mask = cvCreateImage(camSz, IPL_DEPTH_8U, 1);

    // previous image and pyramides
    IplImage *imgCamPrv     = cvCreateImage(camSz, 8, 1);
    IplImage *imgCamPyr     = cvCreateImage(camSz, 8, 1);
    IplImage *imgCamPyrPrv  = cvCreateImage(camSz, 8, 1);

    // prev and curr tracked points
    CvPoint2D32f *camPts[2];
    camPts[0] = (CvPoint2D32f*) cvAlloc(featuresMax * sizeof(CvPoint2D32f));
    camPts[1] = (CvPoint2D32f*) cvAlloc(featuresMax * sizeof(CvPoint2D32f));
    CvPoint3D32f *camPts3D[2];
    camPts3D[0] = (CvPoint3D32f*) cvAlloc(featuresMax * sizeof(CvPoint3D32f));
    camPts3D[1] = (CvPoint3D32f*) cvAlloc(featuresMax * sizeof(CvPoint3D32f));

    CvPoint2D32f *swapPts;
    CvPoint3D32f *swapPts3; // i guess i can use void* d:D
    char *swapStatus;
    char *camPtsStatus = (char*)cvAlloc(featuresMax);
    char *pts3DStatus[2];
    pts3DStatus[0]  = (char*) cvAlloc(featuresMax *  sizeof(char));
    pts3DStatus[1]  = (char*) cvAlloc(featuresMax * sizeof(char));
    CvPoint *tr2pmdIdxs = (CvPoint*) cvAlloc(featuresMax * sizeof(CvPoint));

    // 3d rays where points points :P
    CvPoint3D32f *trackedPts = (CvPoint3D32f*) cvAlloc(featuresMax * sizeof(CvPoint3D32f));

    // contains (row,col) of pmd 3D pts 
    CvPoint **pmd2imgIdxs = (CvPoint**) cvAlloc(pmdSz.height * sizeof(CvPoint*));
    for(int i = 0; i < pmdSz.height; i++) 
        pmd2imgIdxs[i] = (CvPoint*) cvAlloc(pmdSz.width * sizeof(CvPoint));

    // pmd history
    History *history = createHistory();

    /***** ui and gl stuff *****/
    if(ui) {
        cvNamedWindow("PMD", 0);
        cvNamedWindow("Cam", 0);
    }
   
    if(gl) {
        glfwInit();
        if(!glfwOpenWindow(640, 480, 8, 8, 8, 8, 24, 0, GLFW_WINDOW)) {
            glfwTerminate();
            fprintf(stderr, "ERROR: can't init glfw window!\n");
            return 1;
        } 
    }
    //FIXME: put this in if(gl)
    font = new FTGLPixmapFont("./djvm.ttf");

    if(font->Error()) {
            fprintf(stderr, "ERROR: can't load font ./djvm.ttf");
            return 1;
    }
    font->FaceSize(20);

    // fps counting
    time_t tic=time(0);
    time_t tac;

    int fpsCnt = 0;    

    //icp pairs
    vector<PtPair> pairs;
    vector<PtPair> motion;
    float mean, var;

    /***** main loop *****/
    /*********************/
    int frames = 0;
    int goodFrames = 0;
    while(1) {
        /***** fps counting *****/
        if(fps) {
            tac = time(0);
            if(tac - tic >= 1) {
                printf("%i FPS\n", fpsCnt);
                fflush(stdout);
                fpsCnt = 0;
                tic=tac;
            }
            fpsCnt++;
        }
        if(grabData(pmdc)) break; // end of seq?
    

        /***** tracking *****/
        //if(trPtsCnt)
        cvCalcOpticalFlowPyrLK( imgCamPrv, pmdc->iCam, imgCamPyrPrv, imgCamPyr
                              , camPts[0], camPts[1], trPtsCnt
                              , cvSize(pmdc->_track.winSz, pmdc->_track.winSz), 3, camPtsStatus, 0
                              , cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03)
                              , pmdc->_track.trackingFlags);
        pmdc->_track.trackingFlags |= CV_LKFLOW_PYR_A_READY;

        // filter out not tracked points        
        int k = 0; 
        for(int i = 0; i < trPtsCnt; i++) 
            if(camPtsStatus[i]) {
                camPts3D[0][k] = camPts3D[0][i];
                pts3DStatus[0][k] = pts3DStatus[0][i];
                camPts[1][k++] = camPts[1][i];
            }

        trPtsCnt = k;            
        
        /***** 3d (re)projection *****/
        cvProjectArrayToCartesian(pmdc->intrinsicCam, camPts[1], trPtsCnt, trackedPts);

        int reprojected = projectImageToPMD( rot, trn, pmdc->intrinsicCam, pmdc->pts, pmdSz, camSz, pmd2imgIdxs
                                           , camPts[1], &trPtsCnt, camPts3D[1], pts3DStatus[1], tr2pmdIdxs);
        double alignError = 0.0;
        bool poseEstimated = false;
        if(reprojected >= pmdc->minPts4Pose) { // we need at least minPts4Pose points

            if(motionMeanAndVariance(camPts3D, pts3DStatus, trPtsCnt, &mean, &var))
                    // TODO: can be fused with centroid computation
                    outliersSpeedSigma(camPts3D, pts3DStatus, trPtsCnt, mean, var);           

            outliersDepthAndColor( pmdc->pts, pmdc->iCamColor, pmdSz, camPts[1], trPtsCnt
                                 , tr2pmdIdxs, pts3DStatus[1], pmdc->sigmaDepth
                                 , pmdc->dpThreshold, pmdc->sigmaColor);


            pairs.clear();
            double centroidM[3] = {0.0, 0.0, 0.0};
            double centroidD[3] = {0.0, 0.0, 0.0};
            double pt1[3];
            double pt2[3];

            for(int i = 0; i < trPtsCnt; i++) 
                if(pts3DStatus[1][i] && pts3DStatus[0][i]) {                   
                    pt1[0] = camPts3D[0][i].x;
                    pt1[1] = camPts3D[0][i].y;
                    pt1[2] = camPts3D[0][i].z;
                    pt2[0] = camPts3D[1][i].x;
                    pt2[1] = camPts3D[1][i].y;
                    pt2[2] = camPts3D[1][i].z;

                    //TODO can be fused -- (+=) :: a -> a -> a
                    centroidM[0] += camPts3D[0][i].x;
                    centroidM[1] += camPts3D[0][i].y;
                    centroidM[2] += camPts3D[0][i].z;
                    centroidD[0] += camPts3D[1][i].x;
                    centroidD[1] += camPts3D[1][i].y;
                    centroidD[2] += camPts3D[1][i].z;

                    PtPair currentPair(pt1, pt2);
                    pairs.push_back(currentPair);
                }

            reprojected = pairs.size();

            if(reprojected >= pmdc->minPts4Pose) { // enough corresponding points
            centroidM[0] /= reprojected; 
            centroidM[1] /= reprojected; 
            centroidM[2] /= reprojected; 
            centroidD[0] /= reprojected; 
            centroidD[1] /= reprojected; 
            centroidD[2] /= reprojected; 

            double transformMat[16];
            try { alignError = pmdc->icp->Align(pairs, transformMat, centroidM, centroidD); }
            catch(...) { fprintf(stderr, "ERROR: matrix is singular!\n"); }

            if(!gl) printf( "%i: align error: %f, 3d pts count: %i, 2d pts count: %i\n"
                          , frames, alignError, reprojected, trPtsCnt);          

            for(int i = 1; i < 16; i++) 
                if (i%4 > 2) continue; // bottom row
                else if(i/4 > 2) CV_MAT_ELEM(*trnGuess, float, i%4, 0) = transformMat[i];
                     else CV_MAT_ELEM(*rotMatGuess, float, i/4, i%4) = transformMat[i]; // right col

            cvRodrigues2(rotMatGuess, rotGuess, NULL);

            if(alignError < pmdc->maxError) 
                    poseEstimated = true;
            }
        } 


        /**** Print pose to file ****/
        //TODO: config option
        if(pmdc->savePoses) { 
            char filename[] = "./dat/scan0000.pose";       
            sprintf(filename, "./dat/scan%04d.pose", frames);
            FILE *pose = fopen(filename, "wb");
	        if(!pose) fprintf(stderr, "cant create file %s!\n", filename);

            if(poseEstimated) {
                fprintf(pose, "%f %f %f\n%f %f %f\n"
                       , 100.0*CV_MAT_ELEM(*rotGuess, float, 0, 0)
                       , 100.0*CV_MAT_ELEM(*rotGuess, float, 0, 0)
                       , 100.0*CV_MAT_ELEM(*rotGuess, float, 0, 0)
                       , 100.0*CV_MAT_ELEM(*trnGuess, float, 0, 0)
                       , 100.0*CV_MAT_ELEM(*trnGuess, float, 1, 0)
                       , 100.0*CV_MAT_ELEM(*trnGuess, float, 2, 0));
                goodFrames++;            
            } else {
                fprintf( stderr, "ERROR: %i points found, align error: %f\n"
                       , reprojected, alignError);
                fprintf(pose, "%f %f %f\n%f %f %f\n", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
            }       

            fflush(pose);
            fclose(pose);
        }
        frames++;


        /***** find features *****/
        if(trPtsCnt < pmdc->_track.minFeatures) {
                int trPtsCntFound = featuresMax - trPtsCnt;
                cvSet(mask, cvScalarAll(255));
                for(int i = 0; i < trPtsCnt; i++) 
                    cvCircle(mask, cvPointFrom32f(camPts[1][i]), 20, CV_RGB(0,0,0), -1, 8, 0);
                cvGoodFeaturesToTrack( pmdc->iCam, eig, tmp, camPts[1] + trPtsCnt, &trPtsCntFound
                                     , pmdc->_track.quality, pmdc->_track.minDist, mask, 3, 0, 0.04);
                cvFindCornerSubPix( pmdc->iCam, camPts[1] + trPtsCnt, trPtsCntFound
                                  , cvSize(pmdc->_track.winSz,pmdc->_track.winSz), cvSize(-1,-1)
                                  , cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
                trPtsCnt += trPtsCntFound;
        }

        Frame *f = allocFrame3DData(pmdSz);
        fillFrame(f, pmdc->iCamColor, pmdSz, pmdc->pts, pmd2imgIdxs, rotGuess, trnGuess, alignError);
        history = addFrame(history, f);
        checkHistoryLen(history, pmdc->historyLen);

        bool pause = false;
        do {
             /***** ui and rendring *****/
            if(gl) render( history, trackedPts, trPtsCnt
                         , rot, trn, camPts3D, pts3DStatus
                         , reprojected, poseEstimated);
            if(ui) {
                for(int i = 0; i < trPtsCnt; i++)
                    cvCircle( pmdc->iCamColor, cvPointFrom32f(camPts[1][i])
                            , 3, CV_RGB(255,0,255), -1, 8, 0);

                cvShowImage("PMD", pmdc->iPMDI);
                cvShowImage("Cam", pmdc->iCamColor);
            }

            int key = cvWaitKey(5);
            if(27 == key) return 0; // ESC pressed FIXME: release stuff
            if((int)' ' == key) pause = !pause;
            if(gl) if(glfwGetKey(GLFW_KEY_ESC) == GLFW_PRESS) return 0; //in OpenGL window ;)
        } while(pause);

        CV_SWAP(imgCamPrv, pmdc->iCam, swapTemp);
        CV_SWAP(imgCamPyrPrv, imgCamPyr, swapTemp);
        CV_SWAP(camPts[0], camPts[1], swapPts);
        CV_SWAP(camPts3D[0], camPts3D[1], swapPts3);
        CV_SWAP(pts3DStatus[0], pts3DStatus[1], swapStatus);


    } // while(1)

    // let OS clean all images and matrices
    // TODO: releasePMD(&pmd);
    if(gl) {
        glfwTerminate();
    }
    printf("%i good frames, %i frames total.\n", goodFrames, frames);
    return 0;
}


// Global rendering settings
float rotx = 0;
float roty = 0;
float rotz = 0;
float scale = 0.7;
int renderCoords = 1;
int renderCams = 1;
int renderColorPts = 1;
int renderLines = 0;
int renderTracked = 1;
int renderHistory = 1;
int centerCloud = 0;



void renderFrame(Frame *f) {
    assert(f->img->imageData);
    glBegin(GL_POINTS);
    if(!renderColorPts) {
    //FIXME: mess with the indices (i,j)
        for(int j = 0; j < f->sz.width; j++) 
         for(int i = 0; i < f->sz.height; i++)
          glVertex3f(f->pts[i][j].x, f->pts[i][j].y, f->pts[i][j].z);
    glEnd();
    } else {
    uchar *imgCamPix = 0;
    for(int j = 0; j < f->sz.width; j++)
        for(int i = 0; i < f->sz.height; i++) {
            int x = f->status[i][j].x;
            int y = f->status[i][j].y;
            if(x > 0) {
                imgCamPix = &((uchar*)
                    (f->img->imageData + f->img->widthStep * y))[x*3];
                glColor3f( ((float)imgCamPix[2])/255.0
                         , ((float)imgCamPix[1])/255.0
                         , ((float)imgCamPix[0])/255.0); //BGR
            } else glColor3f(1.0, 0.0, 0.0);
            glVertex3f(f->pts[i][j].x, f->pts[i][j].y, f->pts[i][j].z);
        }
    } // if renderColorPts else
}

void render( History *history, CvPoint3D32f *camPts,  int trPtsCnt
           , CvMat *rot,  CvMat *trn
           , CvPoint3D32f **camPts3D, char **pts3DStatus
           , int reprojected, bool poseEstimated) {

    if(glfwGetKey((int)'W') == GLFW_PRESS) roty += 10.0;
    if(glfwGetKey((int)'S') == GLFW_PRESS) roty -= 10.0;
    if(glfwGetKey((int)'A') == GLFW_PRESS) rotx -= 10.0;
    if(glfwGetKey((int)'D') == GLFW_PRESS) rotx += 10.0;
    if(glfwGetKey((int)'Q') == GLFW_PRESS) rotz -= 10.0;
    if(glfwGetKey((int)'E') == GLFW_PRESS) rotz += 10.0;
    if(glfwGetKey((int)'R') == GLFW_PRESS) scale -= 0.1;
    if(glfwGetKey((int)'F') == GLFW_PRESS) scale += 0.1;
    if(glfwGetKey((int)'1') == GLFW_PRESS) renderCoords = !renderCoords;
    if(glfwGetKey((int)'2') == GLFW_PRESS) renderCams = !renderCams;
    if(glfwGetKey((int)'3') == GLFW_PRESS) renderColorPts = !renderColorPts;
    if(glfwGetKey((int)'4') == GLFW_PRESS) renderLines = !renderLines;
    if(glfwGetKey((int)'5') == GLFW_PRESS) renderTracked = !renderTracked;
    if(glfwGetKey((int)'6') == GLFW_PRESS) renderHistory = !renderHistory;
    if(glfwGetKey((int)'C') == GLFW_PRESS) centerCloud = !centerCloud;

    int width, height;
    GLUquadric *quadric;

    glfwGetWindowSize(&width, &height);
    height = height < 1 ? 1 : height;

    glViewport(0, 0, width, height);

    glEnable(GL_BLEND); 
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glClearColor(0.0f, 0.0f, 0.0f, 0.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(70, (double)width/(double)height, 0.01, 100.0);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    int cj = history->frame->sz.width / 2;
    int ci = history->frame->sz.height / 2;
    CvPoint3D32f **pts = history->frame->pts;
    //   gluLookAt(scale, scale, scale, 
       //     pts[ci][cj].x, pts[ci][cj].y, -pts[ci][cj].z, 0.0, 1.0, 0.0);
    gluLookAt(scale, scale, scale, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

    glScalef(1.0, 1.0, -1.0); // convert opengl coord system to left handed

    glEnable(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);

    /* text rendering */
    char info[30];
    FTPoint pt(5.0, 5.0);
    sprintf(info, "2D pts: %i", trPtsCnt);
    font->Render(info, -1, pt);

    pt.Y(35.0);
    sprintf(info, "3D pts: %i", reprojected);
    font->Render(info, -1, pt);

    pt.Y(65.0);
    sprintf(info, "Align error: %f", history->frame->alignError);
    font->Render(info, -1, pt);

    glRotatef(rotz, 0.0, 0.0, 1.0);
    glRotatef(roty, 0.0, 1.0, 0.0);
    glRotatef(rotx, 1.0, 0.0, 0.0);

    if(centerCloud) glTranslatef(-pts[ci][cj].x, -pts[ci][cj].y, -pts[ci][cj].z);


    /***** xyz axes *****/
    if(renderCoords) {
    glBegin(GL_LINES);
        glColor3f(1.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(1.0, 0.0, 0.0);
        glColor3f(0.0, 1.0, 0.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 1.0, 0.0);
        glColor3f(0.0, 0.0, 1.0);
        glVertex3f(0.0, 0.0, 0.0);
        glVertex3f(0.0, 0.0, 1.0);
    glEnd();
    }

    /***** render PMD-points PMD-cam in the O *****/
    fflush(stdout);
    renderFrame(history->frame);
    fflush(stdout);

    if(renderHistory) {
        fflush(stdout);
        History *histI = history;
        glPushMatrix();

        //for(int i = 0; histI->prev; i++) {
        while(histI->prev) {
                glColor3f(1.0, 0.0, 0.0);//, (100.0-i)/100);
                assert(histI);

                //if(histI->frame->alignError < 0.03) { //FIXME_ hardcoded thrs 
                    glRotatef(-CV_MAT_ELEM(*histI->frame->rot, float, 0, 0), 1.0f, 0.0f, 0.0f);
                    glRotatef(-CV_MAT_ELEM(*histI->frame->rot, float, 1, 0), 0.0f, 1.0f, 0.0f);
                    glRotatef(-CV_MAT_ELEM(*histI->frame->rot, float, 2, 0), 0.0f, 0.0f, 1.0f);
                    glTranslatef( -CV_MAT_ELEM(*histI->frame->trn, float, 0, 0)
                                , -CV_MAT_ELEM(*histI->frame->trn, float, 1, 0)
                                , -CV_MAT_ELEM(*histI->frame->trn, float, 2, 0));
                //}
                histI = histI->prev;
                renderFrame(histI->frame);
            }
        glPopMatrix();
    }
   
    quadric = gluNewQuadric();


    if(renderCams) {
        glColor3f(0.5, 0.5, 0.0);
        glPushMatrix();
            glScalef(1.0, 1.0, -1.0); // rotated cylinder
            gluCylinder(quadric, 0.05, 0.0, 0.05, 10, 10);
        glPopMatrix();
    }

    /***** render Cam and features according to essential matrix [R|t] *****/
    if(renderTracked) {
     glBegin(GL_LINES);
     for(int i = 0; i < trPtsCnt; i++) {
       if(pts3DStatus[1][i] &&
          pts3DStatus[0][i]) {
               glColor3f(1.0, 0.0, 0.0);
               glVertex3f(camPts3D[0][i].x, camPts3D[0][i].y, camPts3D[0][i].z);
               glVertex3f(camPts3D[1][i].x, camPts3D[1][i].y, camPts3D[1][i].z);
               glColor3f(0.0, 0.0, 0.5);
               glPushMatrix();
                glTranslatef(camPts3D[0][i].x, camPts3D[0][i].y, camPts3D[0][i].z);
                gluSphere(quadric, 0.005f, 10, 10);
               glPopMatrix();
               glColor3f(0.5, 0.0, 1.0);
               glPushMatrix();
                glTranslatef(camPts3D[1][i].x, camPts3D[1][i].y, camPts3D[1][i].z);
                gluSphere(quadric, 0.01f, 10, 10);
               glPopMatrix();
       }
     }
     glEnd();
    }
    glBegin(GL_LINES);
     for(int i = 0; i < trPtsCnt; i++) 
       if(pts3DStatus[1][i] &&
          pts3DStatus[0][i]) {
               glColor3f(1.0, 0.0, 0.0);
               glVertex3f(camPts3D[0][i].x, camPts3D[0][i].y, camPts3D[0][i].z);
               glVertex3f(camPts3D[1][i].x, camPts3D[1][i].y, camPts3D[1][i].z);
       }
    glEnd();


    glPushMatrix();
        glTranslatef( -CV_MAT_ELEM(*trn, float, 0, 0)
                    , -CV_MAT_ELEM(*trn, float, 1, 0)
                    , -CV_MAT_ELEM(*trn, float, 2, 0));
        glRotatef(-CV_MAT_ELEM(*rot, float, 0, 0), 1.0f, 0.0f, 0.0f);
        glRotatef(-CV_MAT_ELEM(*rot, float, 1, 0), 0.0f, 1.0f, 0.0f);
        glRotatef(-CV_MAT_ELEM(*rot, float, 2, 0), 0.0f, 0.0f, 1.0f);
        if(renderCams) {
            glColor3f(0.0, 0.5, 0.5);
            glPushMatrix();
            glScalef(1.0, 1.0, -1.0);
            gluCylinder(quadric, 0.05, 0.0, 0.05, 10, 10);
            glPopMatrix();
        }

        if(renderLines) {
            glColor3f(1.0, 0.0, 1.0);
            glBegin(GL_LINES);
             for(int i = 0; i < trPtsCnt; i++) {
              glVertex3f(0.0f, 0.0f, 0.0f);
              glVertex3f(-camPts[i].x, camPts[i].y, camPts[i].z); //FIXME: revise coordinates! why -camPts?! 
             }
            glEnd();
        }
    glPopMatrix();

    glPushMatrix();
      glRotatef(CV_MAT_ELEM(*history->frame->rot, float, 2, 0), 0.0f, 0.0f, 1.0f);
      glRotatef(CV_MAT_ELEM(*history->frame->rot, float, 1, 0), 0.0f, 1.0f, 0.0f);
      glRotatef(CV_MAT_ELEM(*history->frame->rot, float, 0, 0), 1.0f, 0.0f, 0.0f);
      glTranslatef( CV_MAT_ELEM(*history->frame->trn, float, 0, 0)
                  , CV_MAT_ELEM(*history->frame->trn, float, 1, 0)
                  , CV_MAT_ELEM(*history->frame->trn, float, 2, 0));
      if(poseEstimated) glColor3f(0.0, 0.0, 1.0);
      else glColor3f(0.0, 0.0, 0.3);
      //render pose and pmd cam
      glScalef(1.0, 1.0, -1.0);
      gluCylinder(quadric, 0.05, 0.0, 0.05, 10, 10);
      glColor4f(1.0, 0.0, 0.0, 0.5);
      gluSphere(quadric, history->frame->alignError, 20, 20);
    glPopMatrix();

    gluDeleteQuadric(quadric);
    glfwSwapBuffers();
}

