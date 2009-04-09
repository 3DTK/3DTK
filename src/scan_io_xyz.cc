/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "scan_io_xyz.h"
#include "globals.icc"

#include <algorithm>
using std::swap;

#ifdef _MSC_VER
#include <windows.h>
#endif

/**
 * Reads specified scans from given directory in
 * the XYZ file format
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
int ScanIO_xyz::readScans(int start, int end, string &dir, int maxDist, int mindist,
					 double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;

  ifstream scan_in, pose_in;

  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  scanFileName = dir + "Challenger.xyz"; // " + to_string(fileCounter,2) + ".xyz";
  scan_in.open(scanFileName.c_str());
  // read 3D scan
  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  cout << "Processing Scan " << scanFileName;

  char firstline[255];
  scan_in.getline(firstline, 255);

  double rPos[3] = { 0.0, 0.0, 0.0 };
  double rPosTheta[16]  = { 0.0, 0.0, 0.0 };

  euler[0] = rPos[0];
  euler[1] = rPos[1];
  euler[2] = rPos[2];
  euler[3] = rPosTheta[0];
  euler[4] = rPosTheta[1];
  euler[5] = rPosTheta[2];

  while (scan_in.good()) {
    Point p;
    int dummy;
    scan_in >> p.x >> p.y >> p.z >> dummy >> dummy >> dummy;
    p.x *= 100;
    p.y *= 100;
    p.z *= 100;
	 
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
  return new ScanIO_xyz;
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
