/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_wrl.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cerr;
using std::endl;

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <cstring>

/**
 * Reads specified scans from given directory in
 * the PLY file format. It will be compiled as 
 * shared lib.
 *
 * Scan poses will NOT be initialized after a call
 * to this function.
 *
 * 
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param dir The directory from which to read
 * @param maxDist Reads only Points up to this Distance
 * @param minDist Reads only Points from this Distance
 * @param euler Initital pose estimates (will not be applied to the points
 * @param ptss Vector containing the read points
 */
int ScanIO_wrl::readScans(int start, int end, string &dir, int maxDist, int mindist,
					 double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;
  string poseFileName;
  int maxDist2 = (maxDist == -1 ? -1 : sqr(maxDist));

  int my_fileNr = fileCounter;
  
  ifstream scan_in, pose_in;

  for (int i = 0; i < 6; i++) euler[i] = 0;

  if (end > -1 && fileCounter > end) return -1;
  poseFileName = dir + to_string(fileCounter,3) + "/position.dat";
    
  pose_in.open(poseFileName.c_str());
  // read 3D scan

  if (!pose_in.good()) return -1; // no more files in the directory
  cout << "Processing Scan " << dir << to_string(fileCounter, 3);

  for (unsigned int i = 0; i < 6; pose_in >> euler[i++]);
  
  // convert mm to cm 
  for (unsigned int i = 0; i < 3; i++) euler[i] = euler[i] * 0.1;
  
  // convert angles from deg to rad
  for (unsigned int i = 3; i <= 5; i++) {
    euler[i] *= 0.01;
    //	 if (euler[i] < 0.0) euler[i] += 360;
    euler[i] = rad(euler[i]);
  }
  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << deg(euler[3]) << "," << deg(euler[4]) << ","  << deg(euler[5]) << ")" << endl;


  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  scanFileName = dir + to_string(fileCounter,3) + ".wrl";   
  scan_in.open(scanFileName.c_str());
  
  // read 3D scan
  if (!scan_in.good()) return -1;  // no more scans to read

  // overread the first 7 tokes / 4 lines - don't care here
  char dummy[255];
  for (int i = 0; i < 4; i++) {
    scan_in.getline(dummy, 255);
  }

  do {
    char cx[25], cy[25], cz[25];
    double xyz[3];
    scan_in >> cx >> cy >> cz;
    if (!scan_in.good()) break;
    // remove the "," from the z coordinate
    cz[strlen(cz)-1] = 0;
    xyz[0] = atof(cx);
    xyz[1] = atof(cy);
    xyz[2] = atof(cz);
    Point p(xyz);
    if (maxDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) < maxDist2)
	 ptss.push_back(p);
  } while (scan_in.good());
  
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
  return new ScanIO_wrl;
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
