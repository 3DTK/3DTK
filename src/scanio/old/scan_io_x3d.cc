/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_x3d.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;

#ifdef _MSC_VER
#include <windows.h>
#endif

using std::cerr;
/**
 * Reads specified scans from given directory in
 * the x3d file format. It will be compiled
 * as shared lib.
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
int ScanIO_x3d::readScans(int start, int end, string &dir,
			  int maxDist, int minDist,
			  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string scanFileName;

  ifstream scan_in;

  double maxDist2 = sqr(maxDist);

  int my_fileNr = fileCounter;
  
  if (end > -1 && fileCounter > end) return -1; // 'nuf read
  scanFileName = dir + "scan" + to_string(fileCounter,3) + ".x3d";
  scan_in.open(scanFileName.c_str());

  // read 3D scan
  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  cout << "Processing Scan " << scanFileName;
  
  for (unsigned int i = 0; i < 6; euler[i++] = 0.0);

  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << euler[3] << "," << euler[4] << ","  << euler[5] << ")" << endl;
  
  // convert angles from deg to rad
  for (unsigned int i = 3; i <= 5; i++) euler[i] = rad(euler[i]);
  
  // overread the first line
  char dummy[255];
  scan_in.getline(dummy, 255);
  scan_in.getline(dummy, 255);
  scan_in.getline(dummy, 255);
  scan_in.getline(dummy, 255);
  scan_in.getline(dummy, 255);
  scan_in.get(dummy, 255, '"'); // read to beginning of first point
  scan_in.get();                // and throw away the "
  
  while (scan_in.good()) {
    double f[3];
  
    scan_in.get(dummy, 255, ' ');
    scan_in.get();                // and throw away the ' '
    std::stringstream ssx ( dummy );
    ssx >> f[0];

    scan_in.get(dummy, 255, ' ');
    scan_in.get();                // and throw away the ' '
    std::stringstream ssy ( dummy );
    ssy >> f[1];
    
    scan_in.get(dummy, 255, ',');
    scan_in.get();                // and throw away the ','
    scan_in.get();                // and the ' '
    std::stringstream ssz ( dummy );
    ssz >> f[2];

    // scale to cm
    for (int i=0; i<3; i++) {
      f[i] *= 100.0;
    }
    
    Point p(f);

    // load points up to a certain distance only
    // maxDist2 = -1 indicates no limitation
    if (maxDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) < maxDist2)
      ptss.push_back(p);
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
  return new ScanIO_x3d;
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
