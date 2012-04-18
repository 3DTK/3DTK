/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_asc.h"
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
 * Reads specified scans from given directory in
 * the file format ZuF from Zoller+Froehlich AG 
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
int ScanIO_asc::readScans(int start, int end, string &dir, int maxDist, int mindist,
					   double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;
  string poseFileName;

  ifstream scan_in, pose_in;
  int my_fileNr = fileCounter;

  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  
  poseFileName = dir + "ScanPos" + to_string(fileCounter,2) + ".DAT";
  scanFileName = dir + "coordinates" + to_string(fileCounter,2) + ".asc";
    
  scan_in.open(scanFileName.c_str());
  pose_in.open(poseFileName.c_str());
  // read 3D scan

  if (!pose_in.good() && !scan_in.good()) return -1; // no more files in the directory
  if (!pose_in.good()) { cerr << "ERROR: Missing file " << poseFileName << endl; exit(1); }
  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  cout << "Processing Scan " << scanFileName;
  cout.flush();

  /*
  char firstline[255];
  scan_in.getline(firstline, 255);
  */
  
  double rPos[3] = { 0.0, 0.0, 0.0 };
  double rPosTheta[16] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  double inMatrix[16];
  /*
  char dummy[255];
  pose_in.getline(dummy, 255);
  pose_in.getline(dummy, 255);
  scan_in.getline(dummy, 255);
  */
  
  for (unsigned int i = 0; i < 16; pose_in >> inMatrix[i++]);

  Matrix4ToEuler(inMatrix, rPosTheta, rPos);
      
  euler[0] = inMatrix[3];
  euler[1] = inMatrix[11];
  euler[2] = inMatrix[7];

  euler[3] = rPosTheta[0];
  euler[4] = rPosTheta[2];
  euler[5] = rPosTheta[1];
  
  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << euler[3] << "," << euler[4] << ","  << euler[5] << ")" << endl;

  while (scan_in.good()) {
    Point p;
    double intensity;
    scan_in >> p.y >> p.z >> p.x >> intensity;
    p.x *= 100;
    p.y *= 100;
    p.z *= 100;
    if (intensity > 0) {
	 cout << p << endl;
	 ptss.push_back(p);
    }
  }

  cout << " done" << endl;

  scan_in.close();
  scan_in.clear();
  pose_in.close();
  pose_in.clear();
  fileCounter++;
  
  return my_fileNr;
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
  return new ScanIO_asc;
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
