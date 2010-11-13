#include "slam6d/scan_io_rts.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cerr;
using std::endl;

#ifdef _MSC_VER
#include <windows.h>
#endif

/** define invalid type */
#define TYPE_INVALID 0x10

/**
 * Reads specified scans from given directory.
 *
 * Scan poses will NOT be initialized after a call
 * to this function.
 *
 * 
 * @param s 3D Scan to be read. The vector of points must be filled
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param maxDist Reads only Points up to this Distance
 * @param dir The directory from which to read
 */
int ScanIO_rts::readScans(int start, int end, string &dir,
					 int maxDist, int minDist,
					 double *euler, vector<Point> &ptss)
{
  static int  fileCounter = start;
  static string poseFileName;
  string scanFileName;
  double timestamp;

  static ifstream pose_in;
  ifstream scan_in;

  if (end > -1 && fileCounter > end) return -1;
  
  if (fileCounter == start) {
    poseFileName = dir + "odometry_0_sync_interpol.dat";
    pose_in.open(poseFileName.c_str());
    if (!pose_in.good()) { cerr << "ERROR: Missing file " << poseFileName << endl; exit(1); }
    for (int i = 0; i < start; i++) {
	 pose_in >> timestamp
		    >> euler[2] >> euler[0] >> euler[1]                  // x, y, z
		    >> euler[3] >> euler[5] >> euler[4];                 // theta_x, theta_y, theta_z
    }
  }
  
  pose_in >> timestamp
		>> euler[2] >> euler[0] >> euler[1]                  // x, y, z
		>> euler[3] >> euler[5] >> euler[4];                 // theta_x, theta_y, theta_z
  
  scanFileName = dir + "scan3d_0_" + to_string(fileCounter) + ".3d";

  scan_in.open(scanFileName.c_str());
  // read 3D scan

  if (!scan_in.good()) {
    pose_in.close();
    pose_in.clear();
    return -1;
  }
  cerr << "Processing Scan " << scanFileName;

  euler[3] = euler[5] = euler[1] = 0;
  
  cerr << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << deg(euler[3]) << "," << deg(euler[4]) << ","  << deg(euler[5]) << ")" << endl;
    
  for (unsigned int i = 0; i < 3; i++) euler[i] *= 0.1;
   
  while (scan_in.good()) {
    Point p;
    int type, dummy;
    scan_in >> p.z >> p.x >> p.y >> type >> dummy >> dummy;
    if (type & TYPE_INVALID) {
	 continue;
    } else {
	 p.x *= 0.1;
	 p.y *= -0.1;
	 p.z *= 0.1;
	 p.type = type;
	 if (scan_in.good()) ptss.push_back(p);
    }
  }
  scan_in.close();
  scan_in.clear();
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
  return new ScanIO_rts;
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
