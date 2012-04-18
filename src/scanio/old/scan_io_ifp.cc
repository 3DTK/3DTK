/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_ifp.h"
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
int ScanIO_ifp::readScans(int start, int end, string &dir, int maxDist, int mindist,
					  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;
  string poseFileName;
  ifstream scan_in, pose_in;

  int my_fileNr = fileCounter;
  
  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  scanFileName = dir + "ScanWorld " + to_string(fileCounter) + ".pts";
  scan_in.open(scanFileName.c_str());
  // read 3D scan

  if (!scan_in.good()) {
    scanFileName = dir + "SW" + to_string(fileCounter) + ".pts";
    scan_in.open(scanFileName.c_str());
  }
  if (!scan_in.good()) {
    return -1;
  }
  cout << "Processing Scan " << scanFileName;

  euler[0] = euler[1] = euler[2] = euler[3] = euler[4] = euler[5] = 0.0;
  
  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << euler[3] << "," << euler[4] << ","  << euler[5] << ")" << endl;
  
  // convert angles from deg to rad
  for (unsigned int i = 3; i <= 5; i++) euler[i] = rad(euler[i]);
 
  // some noise for the pose estimations
  /*
  double t1 = (double)(rand() % 100);
  double t2 = (double)(rand() % 100);
  double t3 = (double)(rand() % 100);
  double t4 = (double)(rand() % 5) * 0.025;
  euler[4] = t4;

  cout << "dx " << t1 << " dy " << t2 << " dz " << t3 << " dtheta_y " << t4 << endl;
  */
  
  int dummy;
  scan_in >> dummy;
  while (scan_in.good()) {
    Point p;
    int intensity, red, green, blue;
    scan_in >> p.z >> p.x >> p.y >> intensity >> red >> green >> blue;
    
    p.z -= 3515165;
    p.x -= 5407003;
    p.y -= 254;
    
    p.x *= -100;
    p.y *= 100;
    p.z *= 100;

    /*
    p.z += t1;
    p.x += t2;
    p.y += t3;
    */
    //cout << p << endl;
    ptss.push_back(p);
  }
    
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
  return new ScanIO_ifp;
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
