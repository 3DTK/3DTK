/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_pcl.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#ifdef _MSC_VER
#include <windows.h>
#endif

/**
 * Reads specified scans from given directory.
 *
 * Scan poses will NOT be initialized after a call
 * to this function.
 *
 * This function actually implements loading of 3D scans
 * in UOS file format and will be compiled as shared lib.
 * 
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param dir The directory from which to read
 * @param maxDist Reads only Points up to this Distance
 * @param mindist Reads only Points from this Distance
 * @param euler Initital pose estimates (will not be applied to the points
 * @param ptss Vector containing the read points
 */
int ScanIO_pcl::readScans(int start, int end, string &dir, int maxDist, int mindist,
			  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;
  string poseFileName;

  ifstream scan_in, pose_in;

  double maxDist2 = sqr(maxDist);
  double minDist2 = sqr(mindist);

  int my_fileNr = fileCounter;
  
  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  // read 3D scan
  poseFileName = dir + "scan" + to_string(fileCounter,4) + ".pose";
  scanFileName = dir + "scan" + to_string(fileCounter,4) + ".pcl";
  scan_in.open(scanFileName.c_str());
  if (!scan_in.good()) { 
    cerr << "ERROR: Missing file" << scanFileName << endl;
    scan_in.close();
    return -1;
  }

  pose_in.open(poseFileName.c_str());
  double inMatrix[16], tMatrix[16];
  double rPosTheta[3], rPos[3];
  if (!pose_in.good()) { 
    cerr << "using default pose 0,0,0 !!!" << endl;
    M4identity(inMatrix);
  } else {
    for (unsigned int i = 0; i < 16; pose_in >> inMatrix[i++]);
  }

  // transform input pose
  tMatrix[0] = inMatrix[5];
  tMatrix[1] = -inMatrix[9];
  tMatrix[2] = -inMatrix[1];
  tMatrix[3] = -inMatrix[13];
  tMatrix[4] = -inMatrix[6];
  tMatrix[5] = inMatrix[10];
  tMatrix[6] = inMatrix[2];
  tMatrix[7] = inMatrix[14];
  tMatrix[8] = -inMatrix[4];
  tMatrix[9] = inMatrix[8];
  tMatrix[10] = inMatrix[0];
  tMatrix[11] = inMatrix[12];
  tMatrix[12] = -inMatrix[7];
  tMatrix[13] = inMatrix[11];
  tMatrix[14] = inMatrix[3];
  tMatrix[15] = inMatrix[15];
  
  Matrix4ToEuler(tMatrix, rPosTheta, rPos);
  
  euler[0] = 100*rPos[0];
  euler[1] = 100*rPos[1];
  euler[2] = 100*rPos[2];
  euler[3] = rPosTheta[0];
  euler[4] = rPosTheta[1];
  euler[5] = rPosTheta[2];

  cout << "Processing Scan " << scanFileName;
  

  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << deg(euler[3]) << "," << deg(euler[4]) << ","  << deg(euler[5]) << ")" << endl;
  
  
  // overread the first line
  char dummy[255];
  scan_in.getline(dummy, 255);
  
  while (scan_in.good()) {
    Point q;
    Point p;
    try {
      scan_in >> q;
    } catch (...) {
      break;
    }
    p.x = q.y*-100.0;
    p.y = q.z*100.0;
    p.z = q.x*100.0;

    // load points up to a certain distance only
    // maxDist2 = -1 indicates no limitation
    if (maxDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) < maxDist2)
      if (mindist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) > minDist2)
        ptss.push_back(p);
  }
  scan_in.close();
  scan_in.clear();
  pose_in.close();
  pose_in.clear();
  fileCounter++;
  
  return  my_fileNr;
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
  return new ScanIO_pcl;
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
