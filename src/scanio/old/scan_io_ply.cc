/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_ply.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cerr;
using std::endl;
#include <string.h>

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
  scanFileName = dir + "scan" + to_string(fileCounter,3) + ".ply";   
  scan_in.open(scanFileName.c_str());
 
  // read 3D scan
  if (!scan_in.good()) return -1;  // no more scans to read  
  
  bool binary = false;
  char dummy[256];
  char str[20]; // whatever size
  double matrix[16];
  int matrixPos = 0;
  int nr;
  float d1,d2,d3,d4;
  sscanf(dummy,"%s %*s %d",str,&nr);

  // header
  int counter = -2;
  do {
    if(counter > -2) counter++;
    scan_in.getline(dummy, 255);
   if (strncmp(dummy, "format", 6) == 0) {
	 if (dummy[7] == 'a') binary = false;
	 else if (dummy[7] == 'b') binary = true;
	 else { cerr << "Don't recognize the format!" << endl; exit(1); }
    }
    else if (strncmp(dummy, "element vertex", 14) == 0) {
	    sscanf(dummy,"%s %*s %d",str,&nr);
      counter++;
    }
    else if (strncmp(dummy, "matrix", 6) == 0) {
	    sscanf(dummy,"%s %f %f %f %f", str, &d1, &d2, &d3, &d4);
	    matrix[matrixPos++] = d1;
	    matrix[matrixPos++] = d2;
	    matrix[matrixPos++] = d3;
	    matrix[matrixPos++] = d4;
    }    
  } while (!(strncmp(dummy, "end_header",10) == 0 || !scan_in.good()));
  
  if (matrixPos > 0) {
    double rPosTheta[3];
    double rPos[3];
    Matrix4ToEuler(matrix, rPosTheta, rPos);
  }

  for (int i=0; i < nr; i++) {	 
    Point p;
    float data, confidence, intensity;
    float dummy;
    int r, g, b;
    if (!binary) {
	    //scan_in >> p.z >> p.x >> p.y >> r >> g >> b;
	    switch(counter) {
        case 6:
        case 12:
          scan_in >> p.z >> p.y >> p.x >> r >> g >> b;
          break;
        case 9:
	    //scan_in >> p.z >> p.x >> p.y >> dummy >> dummy >> dummy >> r >> g >> b;
	        scan_in >> p.z >> p.y >> p.x >> dummy >> dummy >> dummy >> r >> g >> b;
          break;
        default: 
	        scan_in >> p.z >> p.x >> p.y >> confidence >> intensity;
          break;
      }
      if(counter == 6 || counter == 9 || counter == 12) {
        p.rgb[0] = (char)r;
        p.rgb[1] = (char)g;
        p.rgb[2] = (char)b;
      } else {
        p.reflectance = intensity;
      }
    } else {
	 scan_in.read((char*)&data, sizeof(float));
	 p.z = (double)data;
	 scan_in.read((char*)&data, sizeof(float));
	 p.x = (double)data;
	 scan_in.read((char*)&data, sizeof(float));
	 p.y = (double)data;
	 scan_in.read((char*)&confidence, sizeof(float));
	 scan_in.read((char*)&intensity, sizeof(float));
    }

    if (maxDist2 == -1 || (int)(sqr(p.x) + sqr(p.y) + sqr(p.z)) < maxDist2)
	 ptss.push_back(p);
  }

  cout << "Processing Scan " << scanFileName;
  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << euler[3] << "," << euler[4] << ","  << euler[5] << ")" << endl;


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
