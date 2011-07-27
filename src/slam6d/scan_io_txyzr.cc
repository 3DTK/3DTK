/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_txyzr.h"
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

static double X;
static double Y;
static double Z;
static bool FIRST_POINT = true;

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
int ScanIO_txyzr::readScans(int start, int end, string &dir, int maxDist, int mindist,
			  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;

  ifstream scan_in;

  double maxDist2 = sqr(maxDist);

  int my_fileNr = fileCounter;
  
  if (end > -1 && fileCounter > end) return -1; // 'nuf read
  scanFileName = dir + "scan" + to_string(fileCounter,3) + ".xyz";
  
  scan_in.open(scanFileName.c_str());

  // read 3D scan
  if (!scan_in.good()) return -1; // no more files in the directory
  cout << "Processing Scan " << scanFileName;
  
  for (unsigned int i = 0; i < 6; euler[i++] = 0.0);

  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << euler[3] << "," << euler[4] << ","  << euler[5] << ")" << endl;
  
  // overread the first line
  char dummy[255];
  double ddummy,x,y,z;
  scan_in.getline(dummy, 255);
  
  while (scan_in.good()) {
    Point p;
    try {
      scan_in >> ddummy; 
      scan_in >> x;
      scan_in >> y;
      scan_in >> z;
      p.x = 100.0*x;
      p.y = 100.0*z;
      p.z = 100.0*y;

      if (FIRST_POINT) {
        X = p.x;
        Y = p.y;
        Z = p.z;
        FIRST_POINT = false;
      }

//      scan_in >> p;

      scan_in >> p.reflectance;
    } catch (...) {
      break;
    }
    // load points up to a certain distance only
    // maxDist2 = -1 indicates no limitation
    if (maxDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) < maxDist2) {
      if (!ptss.empty()) {
        //p -= ptss[0];
        p.x -= X;
        p.y -= Y;
        p.z -= Z;
      } 
      ptss.push_back(p);
    }


  }
  if (!ptss.empty()) {
    ptss[0].x = ptss[0].y = ptss[0].z = 0.0;
  }
  scan_in.close();
  scan_in.clear();
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
  return new ScanIO_txyzr;
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
