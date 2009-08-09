/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "scan_io_ply.h"
#include "globals.icc"
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
int ScanIO_ply::readScans(int start, int end, string &dir, int maxDist, int mindist,
			  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;
  string poseFileName;
  int maxDist2 = (maxDist == -1 ? -1 : sqr(maxDist));

  int my_fileNr = fileCounter;
  
  ifstream scan_in, pose_in;

  for (int i=0; i < 6; i++) euler[i] = 0;
  
 if (end > -1 && fileCounter > end) return -1; // 'nuf read
  scanFileName = dir + "mesh_cps_pos" + to_string(fileCounter,0) + ".ply";   
  scan_in.open(scanFileName.c_str());
  
  // read 3D scan
  if (!scan_in.good()) return -1;  // no more scans to read
  
  cout << "Processing Scan " << scanFileName;
  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << euler[3] << "," << euler[4] << ","  << euler[5] << ")" << endl;
  
  bool binary = false;
  char dummy[256];
  for (int i=0; i < 9; i++) {
    scan_in.getline(dummy, 255);
    if (i == 1) { // check for format
	 if (dummy[7] == 'a') binary = false;
	 else if (dummy[7] == 'b') binary = true;
	 else { cerr << "Don't recognize the format!" << endl; exit(1); }
    }
  }
  char str[20]; // whatever size
  int nr;
  sscanf(dummy,"%s %*s %d",str,&nr);
  
  for (int i=0; i < 8; i++) {
    scan_in.getline(dummy, 255);
  }
  
  for (int i=0; i < nr; i++) {	 
    Point p;
    float dummyF;
    if (!binary) {
	 scan_in >> p.z >> p.x >> p.y >> dummyF >> dummyF;
    } else {
	 scan_in.read((char*)&dummyF, sizeof(float));
	 p.z = (double)dummyF;
	 scan_in.read((char*)&dummyF, sizeof(float));
	 p.x = (double)dummyF;
	 scan_in.read((char*)&dummyF, sizeof(float));
	 p.y = (double)dummyF;
	 scan_in.read((char*)&dummyF, sizeof(float));
	 scan_in.read((char*)&dummyF, sizeof(float));
    }
    
    p.x *= -100.0;
    p.y *= 100.0;
    p.z *= 100.0;
    
    if (maxDist2 == -1 || (int)(sqr(p.x) + sqr(p.y) + sqr(p.z)) < maxDist2)
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
  return new ScanIO_ply;
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
