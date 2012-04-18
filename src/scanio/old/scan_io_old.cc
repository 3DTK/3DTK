/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_old.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;

#ifdef _MSC_VER
#include <windows.h>
#endif

/**
 * Reads specified scans from given directory in
 * the old file format Kurt3D uses. It will be compiled
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
int ScanIO_old::readScans(int start, int end, string &dir,
			  int maxDist, int minDist,
			  double *euler, vector<Point> &ptss)
{
  static int fileCounter = start;
  string poseFileName;
  string scanFileName;

  int my_fileNr = fileCounter;
 
  ifstream scan_in, pose_in;

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
  //  euler[4] *= -1.0;

  
  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << deg(euler[3]) << "," << deg(euler[4]) << ","  << deg(euler[5]) << ")" << endl;

  for (int i = 1; ; i++) {
    scanFileName = dir + to_string(fileCounter, 3) + "/scan" + to_string(i) + ".dat";
    scan_in.open(scanFileName.c_str());
    if (!scan_in.good()) {
	 scan_in.close();
	 scan_in.clear();
	 break;
    }

    int    Nr = 0, intensity_flag = 0;
    int    D;
    double current_angle;
    double X, Z, I;                     // x,z coordinate and intensity
	 
    char firstLine[81];
    scan_in.getline(firstLine, 80);
	 
    char cNr[4];
    cNr[0] = firstLine[2];                 
    cNr[1] = firstLine[3];                 
    cNr[2] = firstLine[4];
    cNr[3] = 0;           
    Nr = atoi(cNr);
	 
    // determine weather we have the new files with intensity information    
    if (firstLine[16] != 'i') {
	 intensity_flag = 1;
	 char cAngle[8];
	 cAngle[0] = firstLine[35];                 
	 cAngle[1] = firstLine[36];                 
	 cAngle[2] = firstLine[37];
	 cAngle[3] = firstLine[38];
	 cAngle[4] = firstLine[39];
	 cAngle[5] = firstLine[40];
	 cAngle[6] = firstLine[41];
	 cAngle[7] = 0;
	 current_angle = atof(cAngle);
	 cout << current_angle << endl;
    } else {
	 intensity_flag = 0;
	 char cAngle[8];
	 cAngle[0] = firstLine[54];                 
	 cAngle[1] = firstLine[55];                 
	 cAngle[2] = firstLine[56];
	 cAngle[3] = firstLine[57];
	 cAngle[4] = firstLine[58];
	 cAngle[5] = firstLine[59];
	 cAngle[6] = firstLine[60];
	 cAngle[7] = 0;
	 current_angle = atof(cAngle);
    }
	 
    double cos_currentAngle = cos(rad(current_angle)); 
    double sin_currentAngle = sin(rad(current_angle));

    for (int j = 0; j < Nr; j++) {
	 if (!intensity_flag) {
	   scan_in >> X >> Z >> D >> I;
	 } else {
	   scan_in >> X >> Z;
	   I = 1.0;
	 }
	 if (maxDist != -1 && D > maxDist) continue;
	 if (minDist != -1 && D < minDist) continue;
	 // calculate 3D coordinates (local coordinates)
	 Point p;
	 p.x = X;
	 p.y = Z * sin_currentAngle;
	 p.z = Z * cos_currentAngle;
	 
	 ptss.push_back(p);
    }
    scan_in.close();
    scan_in.clear();
  }

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
  return new ScanIO_old;
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
