/*
 * PMDCam implementation
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

#include <libconfig.h>

#include "cvpmd.h"
#include "pmdWrap.h"

#include "slam6d/icp6Dhelix.h"
#include "slam6d/icp6Dortho.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6Dsvd.h"
#include "slam6d/icp6Dapx.h"


//TODO: releasePMDCam()

//TODO: !!! check config parse error !!! TODO

PMDCam *initPMDCam(const char *confPath) {
    PMDCam *pmdc = (PMDCam*)malloc(sizeof(PMDCam));
    config_t *conf = (config_t*)malloc(sizeof(config_t));
    config_init(conf);
    config_read_file(conf, confPath);
    
    config_lookup_bool(conf, "offline", &pmdc->_offlineMode);
    if(!pmdc->_offlineMode) {
        // init PMDCam
        const char *ip;
        const char *plugin;
        config_lookup_string(conf, "ip", &ip);
        config_lookup_string(conf, "plugin", &plugin);
        printf("Starting PMD...\n");
        pmdc->_pmd = initPMD(plugin, ip);
        //FIXME: pmdc initialization assert

        int camID;
#if (((LIBCONFIG_VER_MAJOR == 1) && (LIBCONFIG_VER_MINOR >= 4)) \
    || (LIBCONFIG_VER_MAJOR > 1))
        config_lookup_int(conf, "cameraID", &camID);
#else //libconfig API changed in version 1.4b
        config_lookup_int(conf, "cameraID", (long *)&camID);
#endif
        pmdc->_capture = cvCaptureFromCAM(camID);
        if(!pmdc->_capture) fprintf(stderr, "ERROR: Can't initialize capture, see camera id in pmdc.conf.\n"); 
    } else {      
        printf("Offline mode.\n");
        const char *camVid;
        const char *pmdI;
        const char *pmdA;
        const char *pmd3D;
        const char *headers;
        config_lookup_string(conf, "camVideoFile", &camVid);
        config_lookup_string(conf, "pmdIFile", &pmdI);
     //   config_lookup_string(conf, "pmdAFile", &pmdA);
        config_lookup_string(conf, "pmd3DFile", &pmd3D);        
        config_lookup_string(conf, "headersFile", &headers);
        //FIXME: check empty strings
        printf( "DEBUG: Reading from files: %s, %s, %s.\n"
              , camVid, pmdI, pmd3D);
        pmdc->_capture = cvCaptureFromFile(camVid);
        pmdc->_f.i = fopen(pmdI, "r");
      //  pmdc->_f.a = fopen(pmdA, "r");
        pmdc->_f.p = fopen(pmd3D, "r");
        pmdc->_f.h = fopen(headers, "r");
        if( !pmdc->_capture 
         || !pmdc->_f.i 
        // || !pmdc->_f.a
         || !pmdc->_f.p 
         || !pmdc->_f.h) fprintf(stderr, "ERROR: Can't open data file, see files settings in pmdc.conf.\n");
	 pmdc->header = (ImageHeaderInformation*)cvAlloc(sizeof(ImageHeaderInformation));

        //TODO: read pmd header somewhere
    }
    CvSize pmdSz;
#if (((LIBCONFIG_VER_MAJOR == 1) && (LIBCONFIG_VER_MINOR >= 4)) \
    || (LIBCONFIG_VER_MAJOR > 1))
    config_lookup_int(conf, "pmdSize.width", &pmdSz.width);
    config_lookup_int(conf, "pmdSize.height", &pmdSz.height);
#else //libconfig API changed in version 1.4b
    config_lookup_int(conf, "pmdSize.width", (long *)&pmdSz.width);
    config_lookup_int(conf, "pmdSize.height", (long *)&pmdSz.height);
#endif
    printf("DEBUG: pmdSz: %i %i\n", pmdSz.width, pmdSz.height);
    pmdc->_iPMDIU = cvCreateImage(pmdSz, IPL_DEPTH_8U, 1);
    pmdc->iPMDI   = cvCreateImage(pmdSz, IPL_DEPTH_8U, 1);
    pmdc->_iPMDAU = cvCreateImage(pmdSz, IPL_DEPTH_8U, 1);
    pmdc->iPMDA   = cvCreateImage(pmdSz, IPL_DEPTH_8U, 1);

    pmdc->_iCamColorU = cvQueryFrame(pmdc->_capture);
    CvSize camSz = cvGetSize(pmdc->_iCamColorU);
    printf("DEBUG: camSz: %i %i\n", camSz.width, camSz.height);


    config_lookup_bool(conf, "hybrid", &pmdc->hybrid);
    if(pmdc->hybrid) pmdc->_iCamColorUBuffer   = cvCreateImage(camSz, IPL_DEPTH_8U, 3);

    pmdc->iCamColor   = cvCreateImage(camSz, IPL_DEPTH_8U, 3);
    pmdc->iCam        = cvCreateImage(camSz, IPL_DEPTH_8U, 1);    

    pmdc->_mapXPMD = cvCreateImage(pmdSz, IPL_DEPTH_32F, 1);
    pmdc->_mapYPMD = cvCreateImage(pmdSz, IPL_DEPTH_32F, 1);
    pmdc->_mapXCam = cvCreateImage(camSz, IPL_DEPTH_32F, 1);
    pmdc->_mapYCam = cvCreateImage(camSz, IPL_DEPTH_32F, 1);
    printf("DEBUG: Created images...\n");

    const char *inc;
    const char *dsc;
    const char *inp;
    const char *dsp;
    config_lookup_string(conf, "intrinsicCam", &inc);
    config_lookup_string(conf, "distortionCam", &dsc);
    config_lookup_string(conf, "intrinsicPMD", &inp);
    config_lookup_string(conf, "distortionPMD", &dsp);
    pmdc->intrinsicCam  = (CvMat*)cvLoad(inc);
    pmdc->distortionCam = (CvMat*)cvLoad(dsc);
    pmdc->intrinsicPMD  = (CvMat*)cvLoad(inp);    
    pmdc->distortionPMD = (CvMat*)cvLoad(dsp);
    if(!pmdc->intrinsicCam || !pmdc->distortionCam || !pmdc->intrinsicPMD || !pmdc->distortionPMD)
        fprintf(stderr, "ERROR: Cant load matrix file, see pmdc.conf matrix files settings.\n");

    //TODO: essential mat
 
    cvInitUndistortMap(pmdc->intrinsicCam, pmdc->distortionCam, pmdc->_mapXCam, pmdc->_mapYCam);
    cvInitUndistortMap(pmdc->intrinsicPMD, pmdc->distortionPMD, pmdc->_mapXPMD, pmdc->_mapYPMD);
    printf("DEBUG: initialized undistortion maps...\n");

    CvPoint3D32f **pmdPts = (CvPoint3D32f**) cvAlloc(pmdSz.height * sizeof(CvPoint3D32f*));
    for(int i = 0; i < pmdSz.height; i++) pmdPts[i] = (CvPoint3D32f*) cvAlloc(pmdSz.width * sizeof(CvPoint3D32f));
    pmdc->pts = pmdPts;

   
    config_lookup_float(conf, "tracking.quality", &pmdc->_track.quality);
#if (((LIBCONFIG_VER_MAJOR == 1) && (LIBCONFIG_VER_MINOR >= 4)) \
    || (LIBCONFIG_VER_MAJOR > 1))
    config_lookup_int(conf, "tracking.min", &pmdc->_track.minFeatures);
    config_lookup_int(conf, "tracking.max", &pmdc->_track.maxFeatures);
    config_lookup_int(conf, "tracking.minDist", &pmdc->_track.minDist);
    config_lookup_int(conf, "tracking.winSize", &pmdc->_track.winSz);
    config_lookup_int(conf, "historyLen", &pmdc->historyLen);
#else //libconfig API changed in version 1.4b
    config_lookup_int(conf, "tracking.min", (long *)&pmdc->_track.minFeatures);
    config_lookup_int(conf, "tracking.max", (long *)&pmdc->_track.maxFeatures);
    config_lookup_int(conf, "tracking.minDist", (long *)&pmdc->_track.minDist);
    config_lookup_int(conf, "tracking.winSize", (long *)&pmdc->_track.winSz);
    config_lookup_int(conf, "historyLen", (long *)&pmdc->historyLen);
#endif
    pmdc->_track.trackingFlags = 0; //FIXME, if you init pyrs here

#if (((LIBCONFIG_VER_MAJOR == 1) && (LIBCONFIG_VER_MINOR >= 4)) \
    || (LIBCONFIG_VER_MAJOR > 1))
    config_lookup_int(conf, "minPts", &pmdc->minPts4Pose);
#else //libconfig API changed in version 1.4b
    config_lookup_int(conf, "minPts", (long *)&pmdc->minPts4Pose);
#endif
    config_lookup_float(conf, "maxError", &pmdc->maxError);

    config_lookup_float(conf, "outliersRemoval.sigmaDepth", &pmdc->sigmaDepth);
    config_lookup_float(conf, "outliersRemoval.sigmaColor", &pmdc->sigmaColor);
    config_lookup_float(conf, "outliersRemoval.threshold", &pmdc->dpThreshold);

    config_lookup_bool(conf, "savePoses", &pmdc->savePoses);
    config_lookup_bool(conf, "synchronous", &pmdc->synchronous);

    int icpVerbose;
    config_lookup_bool(conf, "icp.verbose", &icpVerbose);
    bool icpQuiet = !icpVerbose;
    const char *icpMethod;
    config_lookup_string(conf, "icp.method", &icpMethod);
    printf("%s\n", icpMethod);
    if(!strcmp(icpMethod, "helix"))      pmdc->icp = new icp6D_HELIX(icpQuiet);
    else if(!strcmp(icpMethod, "svd"))   pmdc->icp = new icp6D_SVD(icpQuiet);
    else if(!strcmp(icpMethod, "apx"))   pmdc->icp = new icp6D_APX(icpQuiet);
    else if(!strcmp(icpMethod, "quat"))  pmdc->icp = new icp6D_QUAT(icpQuiet);
    else if(!strcmp(icpMethod, "ortho")) pmdc->icp = new icp6D_ORTHO(icpQuiet);
    else {
        fprintf(stderr, "ERROR: Uknown ICP method: %s!\n", icpMethod);
        exit(1);
    }

    //TODO: init tracking?
    //TODO: alloc pts

    config_destroy(conf);
    return pmdc;
}


int grabData(PMDCam *pmdc) {
    static unsigned int latestTimestamp = 0;
    static unsigned int latestTimestampUsec = 0;
    while(1) { // grab data until the new have come
    if(!pmdc->_offlineMode) {
        pmdUpdate(pmdc->_pmd->hnd);
        
        pmdc->_iCamColorUBuffer = cvQueryFrame(pmdc->_capture);

        if(pmdc->synchronous) {            
            pmdRetriveDistances(pmdc->_pmd);
            pmdProjectArrayToCartesian(pmdc->_pmd, pmdc->intrinsicPMD, pmdc->pts);
            // project array has to be called right after retrive distances
            pmdQueryImageAsync(pmdc->_pmd, pmdc->_iPMDIU);

        } else {
            pmdRetriveDistancesAsync(pmdc->_pmd);
            pmdProjectArrayToCartesian(pmdc->_pmd, pmdc->intrinsicPMD, pmdc->pts);
            // project array has to be called right after retrive distances
            pmdQueryImageAsync(pmdc->_pmd, pmdc->_iPMDIU);
/*            printf("time: %f, utime: %f, valid: %f\n"
                  , pmdc->header->Seconds
                  , pmdc->header->Useconds
                  , pmdc->header->ValidImage
                  );*/

        }

        pmdc->header = retriveHeader();
        pmdc->timestamp = (unsigned int)pmdc->header->Seconds;            
        pmdc->timestampUsec = (unsigned int)pmdc->header->Useconds;   

        if(pmdc->hybrid) {
          if( pmdc->timestamp > latestTimestamp 
           || pmdc->timestampUsec > latestTimestampUsec) {
              latestTimestamp = pmdc->timestamp;
              latestTimestampUsec = pmdc->timestampUsec;
              cvCopy(pmdc->_iCamColorUBuffer, pmdc->_iCamColorU);
              break;
          } else continue;
        } else {
            pmdc->_iCamColorU = pmdc->_iCamColorUBuffer;
            break;
        }
    } else { // offliine mode
        pmdc->_iCamColorUBuffer = cvQueryFrame(pmdc->_capture);
        if(!pmdc->_iCamColorUBuffer) return 1;

        CvSize pmdSz = cvGetSize(pmdc->_iPMDIU);
        if(fread(pmdc->_iPMDIU->imageData, sizeof(char), pmdSz.width*pmdSz.height, pmdc->_f.i) 
                < (unsigned)pmdSz.width*pmdSz.height) return 1;
        //if(fread(pmdc->_iPMDAU->imageData, sizeof(char), pmdSz.width*pmdSz.height, pmdc->_f.a) 
        //        < (unsigned)pmdSz.width*pmdSz.height) return 1;
        for(int i = 0; i < pmdSz.height; i++) 
            fread(pmdc->pts[i], sizeof(CvPoint3D32f), pmdSz.width, pmdc->_f.p);

        fread(pmdc->header, sizeof(ImageHeaderInformation), 1, pmdc->_f.h);
        // i hope CvPt32f contains no holes :)
        pmdc->timestamp = (unsigned int)pmdc->header->Seconds;   
        pmdc->timestampUsec = (unsigned int)pmdc->header->Useconds;   

        /*printf("%i %i %i\n", (unsigned int)pmdc->header->Seconds
                , (unsigned int)pmdc->header->Useconds
                , (unsigned int)pmdc->header->ValidImage);*/


        if(pmdc->hybrid) {
          if(  pmdc->timestamp > latestTimestamp 
            || pmdc->timestampUsec > latestTimestampUsec) {
              latestTimestamp = pmdc->timestamp;
              latestTimestampUsec = pmdc->timestampUsec;
              cvCopy(pmdc->_iCamColorUBuffer, pmdc->_iCamColorU);
              break;
	      } else continue;
        } else {
            pmdc->_iCamColorU = pmdc->_iCamColorUBuffer;            
            break;
        }
    
    }
    }
    cvFlip(pmdc->_iPMDIU, 0, 1); // flip around x-axes
    cvRemap(pmdc->_iPMDIU, pmdc->iPMDI, pmdc->_mapXPMD, pmdc->_mapYPMD); //undistortion

    cvFlip(pmdc->_iCamColorU, 0, -1); // flip around x and y axes
    cvRemap(pmdc->_iCamColorU, pmdc->iCamColor, pmdc->_mapXCam, pmdc->_mapYCam);
    cvCvtColor(pmdc->iCamColor, pmdc->iCam, CV_BGR2GRAY);    
    return 0;
}

