/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_xyzr.h"
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
 * in UOS + reflectance file format and will be compiled as shared lib.
 * 
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param dir The directory from which to read
 * @param maxDist Reads only Points up to this Distance
 * @param minDist Reads only Points from this Distance
 * @param euler Initital pose estimates (will not be applied to the points
 * @param ptss Vector containing the read points
 */
int ScanIO_xyzr::readScans(int start, int end, string &dir, int maxDist, int minDist,
			  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;
  string poseFileName;

  ifstream scan_in, pose_in;

  double maxDist2 = sqr(maxDist);
  double minDist2 = sqr(minDist);

  int my_fileNr = fileCounter;
  
  if (end > -1 && fileCounter > end) return -1; // 'nuf read
  /*
  scanFileName = dir + "scan" + to_string(fileCounter,3) + ".3d";
  poseFileName = dir + "scan" + to_string(fileCounter,3) + ".pose";
  
  scan_in.open(scanFileName.c_str());
  pose_in.open(poseFileName.c_str());
  */

  scanFileName = dir + "konv_bremenc_" + to_string(fileCounter,3) + ".txt";
  scan_in.open(scanFileName.c_str());
  poseFileName = dir + "scan" + to_string(fileCounter,3) + ".pose";
  pose_in.open(poseFileName.c_str());
  
  // read 3D scan
  //  if (!pose_in.good() && !scan_in.good()) return -1; // no more files in the directory
 if (!scan_in.good()) return -1; // no more files in the directory
 cout << "Processing Scan " << scanFileName;
  
 if (pose_in.good()) {
     for (unsigned int i = 0; i < 6; pose_in >> euler[i++]);

	// convert angles from deg to rad
	for (unsigned int i = 3; i <= 5; i++) euler[i] = rad(euler[i]);
 } else {
   cout << endl << "No pose estimate given." << endl;
   for (unsigned int i = 0; i < 6; euler[i++] = 0.0);
 }
 
 cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	 << "," << deg(euler[3]) << "," << deg(euler[4]) << ","  << deg(euler[5]) << ")" << endl;
  
  // convert angles from deg to rad
  for (unsigned int i = 3; i <= 5; i++) euler[i] = rad(euler[i]);
  
  // overread the first line
  char dummy[255];
  scan_in.getline(dummy, 255);

  double tmp;
  while (scan_in.good()) {
    Point p;
    try {
	 scan_in >> p;
	 
	 p.x -= 485531.0;
	 p.y -= 5882078.400;
	 p.z -= 52;

	 tmp = p.z;
	 p.z = p.y;
	 p.y = tmp;
	 
	 scan_in >> p.reflectance;
    } catch (...) {
	 break;
    }
    // load points up to a certain distance only
    // maxDist2 = -1 indicates no limitation
    if ((maxDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) < maxDist2) && (minDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) > minDist2))
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
  return new ScanIO_xyzr;
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
