/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Jochen Sprickerhof
 */

#include "slam6d/scan_io_uos_map.h"
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
 * @param minDist Reads only Points from this Distance
 * @param euler Initital pose estimates (will not be applied to the points
 * @param ptss Vector containing the read points
 */
int ScanIO_uos_map::readScans(int start, int end, string &dir, int maxDist, int mindist,
			  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start - 1;
  string scanFileName;
  string poseFileName;

  ifstream scan_in, pose_in;

  double maxDist2 = sqr(maxDist);

  int my_fileNr = fileCounter;
  
  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  if (fileCounter == start - 1) {

    double c, s;
    
    // read map as first 3D scan
#define READ_MAP  
#ifdef READ_MAP    
    scanFileName = dir + "Uni_Hannover.map";
    scan_in.open(scanFileName.c_str());
    
    if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
    cout << "Processing 3D Map " << scanFileName << endl;

    euler[0] = euler[1] = euler[2] = euler[3] = euler[4] = euler[5] = 0.0;

    c = cos(rad(-191.0));
    s = sin(rad(-191.0));
  
    while (scan_in.good()) {
	 double x1, x2, z1, z2;
	 double x1_t, x2_t, z1_t, z2_t;

	 scan_in >> x1_t >> z1_t >> x2_t >> z2_t;
	 
	 x1_t = (x1_t - 113.07) * 100;
	 x2_t = (x2_t - 113.07) * 100;
	 z1_t = (z1_t + 64.05) * 100;
	 z2_t = (z2_t + 64.05) * 100;
	 
	 x1 = x1_t * c + z1_t * s;
	 z1 = - x1_t * s + z1_t * c;
	 
	 x2 = x2_t * c + z2_t * s;
	 z2 = - x2_t * s + z2_t * c;
	 
	 double length = sqrt(sqr(x2-x1) + sqr(z2-z1));
	 double n_x = (x2 - x1) / length;
	 double n_z = (z2 - z1) / length;

	 for (double t = 0.0; t < length; t += 25.0) {
	   Point p;
	   p.x = x1 + t * n_x - 713.832183017;
	   p.z = z1 + t * n_z - 1238.220671;
	   for (double y = -15.0; y < 1000; y += 25) {
		p.y = y + 13.8808;
		ptss.push_back(p);
	   }
	 }
    }
    cout << ptss.size() << endl;
    scan_in.close();
    scan_in.clear();
#endif

#define READ_AERIAL
#ifdef READ_AERIAL
    scanFileName = dir + "bereich_uni.txt";
    scan_in.open(scanFileName.c_str());
    
    if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
    cout << "Processing 3D Aerial Map " << scanFileName << endl;

    euler[0] = euler[1] = euler[2] = euler[3] = euler[4] = euler[5] = 0.0;

    c = cos(rad(-191.0));
    s = sin(rad(-191.0));
  
    while (scan_in.good()) {
	 double x1, z1;
	 double x1_t, y1_t, z1_t;

	 scan_in >> x1_t >> z1_t >> y1_t;
	 
	 x1_t = (x1_t - 3548500 - 113.07) * 100;
	 z1_t = (z1_t - 5806400 + 64.05) * 100;
	 y1_t = (y1_t - 55.0) * 100;
	 
	 x1 = x1_t * c + z1_t * s;
	 z1 = - x1_t * s + z1_t * c;
	 
	 Point p;
	 p.x = x1 - 713.832183017;
	 p.z = z1 - 1238.220671;
	 p.y = y1_t + 13.8808;
	 ptss.push_back(p);

    }
    cout << "Map containts " << ptss.size() << " points" << endl;
    scan_in.close();
    scan_in.clear();
#endif    
    cout << "DONE" << endl;
    fileCounter++;

    return 1;
  }

  scanFileName = dir + "scan" + to_string(fileCounter,3) + ".3d";
  poseFileName = dir + "scan" + to_string(fileCounter,3) + ".pose";
  
  scan_in.open(scanFileName.c_str());
  pose_in.open(poseFileName.c_str());

  // read 3D scan
  if (!pose_in.good() && !scan_in.good()) return -1; // no more files in the directory
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
  return new ScanIO_uos_map;
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
