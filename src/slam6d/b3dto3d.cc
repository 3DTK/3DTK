#include <unistd.h>
#include <string>
#include <iostream>
#include <sys/stat.h>
#include <errno.h>

#include "slam6d/metaScan.h"
#include "slam6d/io_utils.h"
#include "slam6d/scan.h"
#include "slam6d/Boctree.h"

#include "slam6d/globals.icc"

#include "b3dpsreader.h"

int main(int argc, char **argv)
{
    if (argc != 3 && argc != 4) {
        std::cerr << "usage: " << argv[0] << " pointcloud.b3d output_directory [poses.bps]" << std::endl;
        return 1;
    }

    char *b3d_fname = argv[1];

    if (access(b3d_fname, F_OK ) == -1) {
        std::cerr << "file does not exist: " << b3d_fname << std::endl;
        return 1;
    }

    if (strlen(b3d_fname) < 4 || strcmp(b3d_fname+strlen(b3d_fname)-4, ".b3d") != 0) {
        std::cerr << "file does not end in .b3d: " << b3d_fname << std::endl;
        return 1;
    }

    char *bps_fname;

    if (argc == 4) {
        bps_fname = argv[3];
    } else {
        bps_fname = strdup(b3d_fname);
        bps_fname[strlen(bps_fname) - 3] = 'b';
        bps_fname[strlen(bps_fname) - 2] = 'p';
        bps_fname[strlen(bps_fname) - 1] = 's';
    }

    if (access(bps_fname, F_OK ) == -1) {
        std::cerr << "file does not exist: " << b3d_fname << std::endl;
        return 1;
    }

    char *out_dirname = argv[2];

    if (mkdir(out_dirname, 0755) != 0 && errno != EEXIST) {
        std::cerr << "Cannot create directory: " << out_dirname << std::endl;
        return 1;
    }

    double calibration[16];
    double calinv[16];
    double rPos[3] = {0.3, 0.0, -0.1};
    double rPosTheta[3] = {rad(0.0), rad(0.0), -rad(30.0)};
    EulerToMatrix4(rPos, rPosTheta, calinv);
    M4inv(calinv, calibration);
    //M4identity(calibration);

    double scaling[16] = { 0,    0,   100, 0,
        -100, 0,   0,   0,
        0,    100, 0,   0,
        0,    0,   0,   1 };
    //        double scaling[16]; M4identity(scaling);

    B3DPSReader *reader = new B3DPSReader(b3d_fname, bps_fname);


    char outFileName[255];
    vector<double *> points;
    for (size_t i = 0;;++i) {
        reader->getNextGlobal(points, scaling, calibration);
        if (points.empty()) break;

        snprintf(outFileName,255,"%s/scan%03d.3d",out_dirname, i);
        FILE *file = fopen(outFileName, "w");
        if (file == NULL) {
            std::cerr << "cannot open: " << outFileName << std::endl;
            return 1;
        }
        for (unsigned int i=0; i < points.size(); i++) {
            fprintf(file, "%lf %lf %lf\n", points[i][0], points[i][1], points[i][2]);
        }
        fclose(file);
        //cout << "Wrote " << points.size() << " points to " << outFileName << endl;

        reader->deleteGlobalPoints(points);

        snprintf(outFileName,255,"%s/scan%03d.pose",out_dirname, i);
        ofstream posefile(outFileName);
        posefile << "0 0 0" << endl << "0 0 0" << endl; 
        posefile.close();
    }
}

/* vim: set ts=4 sw=4 et: */
