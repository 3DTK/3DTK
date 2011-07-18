/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_ks_rgb.h"
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
int ScanIO_ks_rgb::readScans(int start, int end, string &dir, int maxDist, int mindist,
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
  scanFileName = dir + "Color_ScanPos" + to_string(fileCounter,3) + " - Scan001.txt";
  poseFileName = dir + "scan" + to_string(fileCounter,3) + ".pose";

  scan_in.open(scanFileName.c_str());
  pose_in.open(poseFileName.c_str());

  // read 3D scan
  if (!pose_in.good() && !scan_in.good()) return -1; // no more files in the directory
  if (!scan_in.good()) return -1; // no more files in the directory
  if (!pose_in.good()) { cerr << "ERROR: Missing file " << poseFileName << endl; exit(1); }
  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  cout << "Processing Scan " << scanFileName;
  
  for (unsigned int i = 0; i < 6; pose_in >> euler[i++]);

  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << euler[3] << "," << euler[4] << ","  << euler[5] << ")" << endl;

  // convert angles from deg to rad
  for (unsigned int i = 3; i <= 5; i++) euler[i] = rad(euler[i]);
  
  // overread the first line
  char dummy[255];
  scan_in.getline(dummy, 255);

  // format: X[m] Y[m] Z[m] R[0..1] G[0..1] B[0..1] Amplitude[0..1] Reflectance[]
  float r, g, b;
  while (scan_in.good()) {
    Point p;
    try {
	 scan_in >> p.x;
	 scan_in >> p.z;
	 scan_in >> p.y;
	 
	 scan_in >> r;
	 scan_in >> g;
	 scan_in >> b;
	 scan_in >> p.amplitude;
	 scan_in >> p.reflectance;

	 // that's REALLY not the best way to deal with a fixed offset... well, quick hack! ;-)
	 // seems to hold for all of their scans
	 p.x -= 70000.0;
	 p.z -= 20000.0;
	 
	 p.x *= 100.0;
	 p.y *= 100.0;
	 p.z *= 100.0;

	 p.rgb[0] = (char)(r * 255.0);
	 p.rgb[1] = (char)(g * 255.0);
	 p.rgb[2] = (char)(b * 255.0);
    } catch (...) {
	 break;
    }
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
  return new ScanIO_ks_rgb;
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
