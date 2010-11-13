/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_rts_map.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cerr;
using std::endl;

#ifdef _MSC_VER
#include <windows.h>
#endif

/** define invalid type */
#define TYPE_INVALID 0x10

/**
 * Reads specified scans from given directory in
 * the file format Uni Hannover uses. The first scan
 * is a 3D point cloud, computed from the 2D line map.
 * The class will be compiled as shared lib.
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
int ScanIO_rts_map::readScans(int start, int end, string &dir, int maxDist, int mindist,
						double *euler, vector<Point> &ptss)
{
  static int  fileCounter = start - 1;
  static ifstream pose_in;
  
  string scanFileName;
  string poseFileName;
  double timestamp;

  ifstream scan_in;

  int my_fileNr = fileCounter;
  
  if (end > -1 && fileCounter > end) return -1; // 'nuf read
  
  if (fileCounter == start - 1) {

    poseFileName = dir + "odometry_0_sync_interpol.dat";
    pose_in.open(poseFileName.c_str());
    if (!pose_in.good()) { cerr << "ERROR: Missing file " << poseFileName << endl; exit(1); }
    for (int i = 0; i < start; i++) {
	 pose_in >> timestamp
		    >> euler[2] >> euler[0] >> euler[1]                  // x, y, z
		    >> euler[3] >> euler[5] >> euler[4];                 // theta_x, theta_y, theta_z
    }

    double c, s;
    
    // read map as first 3D scan
    //
    // A given 2D ground truth map, is extrapolated to 3D.
    // Details are given in (jfr2008.pdf):
    //
    // Oliver Wulf, Andreas Nüchter, Joachim Hertzberg,
    // and Bernardo Wagner. Benchmarking Urban Six-Degree-
    // of-Freedom Simultaneous Localization and Mapping.
    // Journal of Field Robotics (JFR), Wiley & Son,
    // ISSN 1556-4959, Volume 25, Issue 3, pages 148 - 163,
    // March 2008
    
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
	   p.x = x1 + t * n_x;
	   p.z = z1 + t * n_z;
	   for (double y = -15.0; y < 1000; y += 25) {
		p.y = y;
		ptss.push_back(p);
	   }
	 }
    }
    cout << ptss.size() << endl;
    scan_in.close();
    scan_in.clear();
#endif

    // read map as first 3D scan
    // A aerial lidar scan is loaded
    // Details are given in (ecmr2009.pdf):
    //
    // Jochen Sprickerhof, Andreas Nüchter, Kai Lingemann,
    // Joachim Hertzberg. An Explicit Loop Closing Technique
    // for 6D SLAM, In Proceedings of the 4th European
    // Conference on Mobile Robots (ECMR '09), Mlini/Dubrovnic,
    // Croatia, September 2009
    
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
	 p.x = x1;
	 p.z = z1;
	 p.y = y1_t;
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
    
  // read remaining 3D scans
  pose_in >> timestamp
		>> euler[2] >> euler[0] >> euler[1]                  // x, y, z
		>> euler[3] >> euler[5] >> euler[4];                 // theta_x, theta_y, theta_z
  
  scanFileName = dir + "scan3d_0_" + to_string(fileCounter) + ".3d";
    
  scan_in.open(scanFileName.c_str());
  // read 3D scan

  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  cout << "Processing Scan " << scanFileName;

  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << deg(euler[3]) << "," << deg(euler[4]) << ","  << deg(euler[5]) << ")" << endl;

  for (unsigned int i = 0; i < 3; i++) euler[i] *= 0.1;
  
  while (scan_in.good()) {
    Point p;
    int type, dummy;
    scan_in >> p.z >> p.x >> p.y >> type >> dummy >> dummy;
    if (type & TYPE_INVALID) {
	 continue;
    } else {
	 p.x *= 0.1;
	 p.y *= -0.1;
	 p.z *= 0.1;
	 p.type = type; 
	 ptss.push_back(p);
    }
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
  return new ScanIO_rts_map;
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
