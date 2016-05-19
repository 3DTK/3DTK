#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#define WANT_STREAM ///< define the WANT stream :)
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
#include <errno.h>

#include "slam6d/metaScan.h"
#include "slam6d/io_utils.h"
#include "slam6d/scan.h"
#include "slam6d/Boctree.h"

#include "slam6d/globals.icc"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#include <direct.h>
#define mkdir(path,mode) _mkdir (path)
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#include <windows.h>
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <strings.h>
#include <dlfcn.h>
#endif

#include "b3dpsreader.h"

/**
 * Main program for reducing scans.
 * Usage: bin/scan_red -r <NR> 'dir',
 * Use -r for octree based reduction  (voxel size=<NR>)
 * and 'dir' the directory of a set of scans
 * Reduced scans will be written to 'dir/reduced'
 * 
 */
int main(int argc, char **argv)
{
  int index = atoi(argv[1]); // which b3d file to use
  int start = atoi(argv[2]); // offset for output

  std::string dir = argv[3];

  char b3d[255];
  char bps[255];
  char outFileName[255];
  snprintf(b3d,255,"%sscan%.3d.b3d", dir.c_str(), index);
  snprintf(bps,255,"%sscan%.3d.bps", dir.c_str(), index);
  cout << "Read from " << b3d << " and " << bps << endl;
  
  double calinv[16];
  double calibration[16];
  double rPos[3] = {0.3, 0.0, -0.1};
  double rPosTheta[3] = {rad(0.0), rad(0.0), -rad(30.0)};
  EulerToMatrix4(rPos, rPosTheta, calinv);
  M4inv(calinv, calibration);
        
  double scaling[16] =
  { 0, 0, 100, 0,
    -100, 0, 0, 0,
    0, 100, 0, 0,
    0, 0, 0, 1 };       
  //        double scaling[16]; M4identity(scaling);
  
  B3DPSReader *reader = new B3DPSReader(b3d, bps);


  int fileCounter = start;
  int length = 720;
  vector<double *> points;
  while(true) {
    reader->getNextGlobal(points, scaling, calibration, length);
    if (points.empty()) break;

    snprintf(outFileName,255,"%sscan%.3d.3d",dir.c_str(), fileCounter);
    FILE *file = fopen(outFileName, "w");
    for (unsigned int i=0; i < points.size(); i++) {
      fprintf(file, "%lf %lf %lf\n", points[i][0], points[i][1], points[i][2]);
    }
    fclose(file);
    cout << "Wrote " << points.size() << " points to " << outFileName << endl;

    reader->deleteGlobalPoints(points);

    snprintf(outFileName,255,"%sscan%.3d.pose",dir.c_str(), fileCounter);
    ofstream posefile(outFileName);
    posefile << "0 0 0" << endl << "0 0 0" << endl; 
    posefile.close();

    fileCounter++;
  }
}
