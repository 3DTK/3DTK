/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH
 */

#include "scan_io_riegl_txt.h"
#include "globals.icc"
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
 * Reads specified scans from given directory in
 * the file format Riegl Laser Measurement GmbH 
 * uses. It will be compiled as shared lib.
 *
 * Scan poses will NOT be initialized after a call
 * to this function. Initial pose estimation works 
 * only with the -p switch, i.e., trusting the initial
 * estimations by Riegl. Actually, the Riegl poses
 * are high precise and this function puts noise
 * on it.
 * 
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param dir The directory from which to read
 * @param maxDist Reads only Points up to this Distance
 * @param minDist Reads only Points from this Distance
 * @param euler Initital pose estimates (will not be applied to the points
 * @param ptss Vector containing the read points
 */
int ScanIO_riegl_txt::readScans(int start, int end, string &dir, int maxDist, int mindist,
						  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;
  string poseFileName;

  ifstream scan_in, pose_in;

  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  
  poseFileName = dir + "scan" + to_string(fileCounter,3) + ".dat";
  scanFileName = dir + "scan" + to_string(fileCounter,3) + ".txt";
    
  scan_in.open(scanFileName.c_str());
  pose_in.open(poseFileName.c_str());
  // read 3D scan

  if (!pose_in.good() && !scan_in.good()) return -1; // no more files in the directory
  if (!pose_in.good()) { cerr << "ERROR: Missing file " << poseFileName << endl; exit(1); }
  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  cout << "Processing Scan " << scanFileName;
  cout.flush();
  
  double rPos[3], rPosTheta[16];
  double inMatrix[16], tMatrix[16];
  for (unsigned int i = 0; i < 16; pose_in >> inMatrix[i++]);

  M4transpose(inMatrix, tMatrix);
  Matrix4ToEuler(tMatrix, rPosTheta, rPos);
    
  swap(rPos[1], rPos[2]);
  swap(rPosTheta[1], rPosTheta[2]);
  euler[0] = rPos[0];
  euler[1] = rPos[1];
  euler[2] = rPos[2];
  euler[3] = rPosTheta[0];
  euler[4] = rPosTheta[1];
  euler[5] = rPosTheta[2];

  long num_pts = 0;
  if (scan_in.good()) {
    scan_in >> num_pts;
    cout << " with " << num_pts << " Points";
    cout.flush();
    ptss.resize(num_pts);
  }
  
  while (scan_in.good()) {
    Point p;
    double range, theta, phi, reflectance;
    scan_in >> p.x >> p.z >> p.y >> range >> theta >> phi >> reflectance;
    /*
    p.x *= 100;
    p.y *= 100;
    p.z *= 100;
    */

    ptss.push_back(p);
  }

  cout << " done" << endl;

  scan_in.close();
  scan_in.clear();
  pose_in.close();
  pose_in.clear();
  fileCounter++;
  
  return fileCounter-1;
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
  return new ScanIO_riegl_txt;
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
