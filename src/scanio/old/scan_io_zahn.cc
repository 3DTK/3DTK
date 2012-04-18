/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_zahn.h"
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
 * Reads specified scans from given directory in
 * the tooth format. It will be compiled as 
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
int ScanIO_zahn::readScans(int start, int end, string &dir, int maxDist, int mindist,
			   double *euler, vector<Point> &ptss)
{
  static int  fileCounter = start;
  string scanFileName;
  string poseFileName;
  double maxDist2 = sqr(maxDist);

  ifstream scan_in, pose_in;

  int my_fileNr = fileCounter;
  
  euler[0] = 0;    
  euler[1] = 0;    
  euler[2] = 0;    
  euler[3] = 0;    
  euler[4] = 0;    
  euler[5] = 0;
  
  if (end > -1 && fileCounter > end) return -1; // 'nuf read
  scanFileName = dir + "Element_" + to_string(fileCounter) + ".asc";
  
  scan_in.open(scanFileName.c_str());
  // read 3D scan
  
  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  cout << "Processing Scan " << scanFileName;
  
  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << deg(euler[3]) << "," << deg(euler[4]) << ","  << deg(euler[5]) << ")" << endl;
  
  for (unsigned int i = 0; i < 3; i++) euler[i] *= 0.1;
  
  while (scan_in.good()) {
    Point p;
    scan_in >> p.x >> p.y >> p.z;
    p.x *= 10;
    p.y *= 10;
    p.z *= -10;
    // load points up to a certain distance only
    // maxDist2 = -1 indicates no limitation
    if (sqr(p.x) + sqr(p.y) + sqr(p.z) < maxDist2)
	 ptss.push_back(p);
  }
    
  scan_in.close();
  scan_in.clear();
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
  return new ScanIO_zahn;
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
