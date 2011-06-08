/**
 * @file
 * @brief Implementation of reading 3D scans in colored XYZ format (i.e., pure
 * 3D data points and RGB color information)
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_xyz_rgb.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cerr;
using std::endl;


#include <algorithm>
using std::swap;

#ifdef _MSC_VER
#include <windows.h>
#endif

/**
 * Reads specified scans from given directory in the colored XYZ file format 
 * (i.e., pure 3D data points and RGB color * information only)
 * It will be compiled as shared lib.
 *
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param dir The directory from which to read
 * @param maxDist Reads only Points up to this Distance
 * @param minDist Reads only Points from this Distance
 * @param euler Initital pose estimates (will not be applied to the points
 * @param ptss Vector containing the read points
 */
int ScanIO_xyz_rgb::readScans(int start, int end, string &dir, int maxDist, int minDist,
					 double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;

  ifstream scan_in, pose_in;
  double maxDist2 = sqr(maxDist);
  double minDist2 = sqr(minDist);

  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  scanFileName = dir + to_string(fileCounter,3) + ".xyz";
  scan_in.open(scanFileName.c_str());
  // read 3D scan
  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  cout << "Processing Scan " << scanFileName;

  char firstline[255];
  scan_in.getline(firstline, 255);

  euler[0] = 0.0;
  euler[1] = 0.0;
  euler[2] = 0.0;
  euler[3] = 0.0;
  euler[4] = 0.0;
  euler[5] = 0.0;

  while (scan_in.good()) {
    Point p;
    int r, g, b;
    scan_in >> p.x >> p.z >> p.y >> r >> g >> b;
    p.x *= 100;
    p.y *= 100;
    p.z *= 100;
    p.rgb[0] = (char)r;
    p.rgb[1] = (char)g;
    p.rgb[2] = (char)b;
	 
    if (maxDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) < maxDist2)
      if (minDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) > minDist2)
        ptss.push_back(p);
  }

  cout << " done" << endl;

  scan_in.close();
  scan_in.clear();
  pose_in.close();
  pose_in.clear();
  fileCounter++;
  
  return 1;
}


/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) ScanIO* create()
#else
extern "C" ScanIO* create()
#endif
{
  return new ScanIO_xyz_rgb;
}


/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) void destroy(ScanIO *sio)
#else
extern "C" void destroy(ScanIO *sio)
#endif
{
  delete sio;
}

#ifdef _MSC_VER
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
	return TRUE;
}
#endif
