#include "slam6d/scan_io_iais.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
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
 * 
 * @param s 3D Scan to be read. The vector of points must be filled
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param maxDist Reads only Points up to this Distance
 * @param dir The directory from which to read
 */
int ScanIO_iais::readScans(int start, int end, string &dir,
					 int maxDist, int minDist,
					 double *euler, vector<Point> &ptss)
{
  static int  fileCounter = start;
  static string poseFileName;
  string scanFileName;

  double maxDist2 = sqr(maxDist);

  double matrix[16];
  
  static ifstream pose_in;
  ifstream scan_in;

  if (fileCounter > end) return -1;
  
  if (fileCounter == start) {
    poseFileName = dir + "poses_abs_with_start.txt";
    pose_in.open(poseFileName.c_str());
    if (!pose_in.good()) { cerr << "ERROR: Missing file " << poseFileName << endl; exit(1); }
    for (int i = 0; i < start; i++) {
	 pose_in >> matrix[0] >> matrix[1] >> matrix[2] >> matrix[12]
		    >> matrix[4] >> matrix[5] >> matrix[6] >> matrix[13]
		    >> matrix[8] >> matrix[9] >> matrix[10] >> matrix[14];
	 //		    >> matrix[12] >> matrix[13] >> matrix[14];
	 matrix[15] = 1.0;
	 matrix[3] = matrix[7] = matrix[11] = 0.0;
    }
  }

  pose_in >> matrix[0] >> matrix[1] >> matrix[2] >> matrix[12]
		>> matrix[4] >> matrix[5] >> matrix[6] >> matrix[13]
		>> matrix[8] >> matrix[9] >> matrix[10] >> matrix[14];
  //		>> matrix[12] >> matrix[13] >> matrix[14];
  matrix[15] = 1.0;
  matrix[3] = matrix[7] = matrix[11] = 0.0;

  Matrix4ToEuler(matrix, euler+3, euler);
  euler[3] *= -1.0;
  euler[5] *= -1.0;
  
  if (end > -1 && fileCounter > end) return -1; // 'nuf read
  scanFileName = dir + "scan" + to_string(fileCounter,3) + ".3d";
  
  scan_in.open(scanFileName.c_str());

  // read 3D scan
  // no more files in the directory
  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  cout << "Processing Scan " << scanFileName;
  
  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << deg(euler[3]) << "," << deg(euler[4]) << ","  << deg(euler[5]) << ")" << endl;
  
  // overread the first line
  char dummy[255];
  scan_in.getline(dummy, 255);
  
  while (scan_in.good()) {
    Point p;
    try {
	 scan_in >> p;
    } catch (...) {
	 break;
    }
    // load points up to a certain distance only
    // maxDist2 = -1 indicates no limitation
    if (maxDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) < maxDist2)
	 ptss.push_back(p);
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
  return new ScanIO_iais;
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
