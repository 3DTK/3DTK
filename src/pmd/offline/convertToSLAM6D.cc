/*
 * converToSLAM6D implementation
 *
 * Copyright (C) Stanislav Serebryakov
 *
 * Released under the GPL version 3.
 *
 */

#include <pmdWrap.h>

int main() {
    PMDCam *pmdc = initPMDCam("./pmdc.conf");

    int frames = 0;
    unsigned int latestTime = 0;

    while(1) {     

        if(grabData(pmdc)) break;

        if(latestTime >= pmdc->timestamp) continue;
        latestTime = pmdc->timestamp;
        //printf("%i %i\n", latestTime, pmdc->timestamp);

        CvSize pmdSz = cvGetSize(pmdc->iPMDI);
        pmdSz.width = 50;

        char filename[] = "./dat/scan000.pose";
        sprintf(filename, "./dat/scan%03d.3d", frames);
        FILE *scan = fopen(filename, "wb");
        if(!scan) fprintf(stderr, "ERROR: cant create file %s.\n", filename);
        sprintf(filename, "./dat/scan%03d.pose", frames);
        FILE *pose = fopen(filename, "wb");
        if(!pose) fprintf(stderr, "ERROR: cant create file %s.\n", filename);



        fprintf(scan, "%i x %i\n", pmdSz.width, pmdSz.height);

        for(int j = 0; j < pmdSz.width; j++)
          for(int i = 0; i < pmdSz.height; i++) 
             fprintf(scan, "%f %f %f\n", pmdc->pts[i][j].x*100.0, pmdc->pts[i][j].y*100.0, pmdc->pts[i][j].z*100.0);

        fprintf(pose, "%f %f %f\n%f %f %f\n", 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);

        if(0 == frames%100) printf("+100\n");

        fflush(scan);
        fflush(pose);
        fclose(scan);
        fclose(pose);
        
        frames++;
    }
    return 0;
}


