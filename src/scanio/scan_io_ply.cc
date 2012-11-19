/*
 * scan_io_ply implementation
 *
 * Copyright (C) Dorit Borrmann, Thomas Escher, Kai Lingemann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Dorit Borrmann. Jacobs University Bremen, Germany.
 * @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Jacobs University Bremen, Germany.
 * @author Thomas Escher. Inst. of CS, University of Osnabrueck, Germany.
 */

#include "scanio/scan_io_ply.h"
#include "slam6d/point.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <vector>
#include <string.h>

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

#include "slam6d/globals.icc"

#define DATA_PATH_PREFIX "scan"
#define DATA_PATH_SUFFIX ".ply"

std::list<std::string> ScanIO_ply::readDirectory(const char* dir_path,
									    unsigned int start,
									    unsigned int end)
{
  std::list<std::string> identifiers;
  for (unsigned int i = start; i <= end; ++i) {
    // identifier is /d/d/d (000-999)
    std::string identifier(to_string(i,3));
    // scan consists of data (.3d) and pose (.pose) files
    path data(dir_path);
    data /= path(std::string(DATA_PATH_PREFIX) + identifier + DATA_PATH_SUFFIX);
    // stop if part of a scan is missing or end by absence is detected
    if (!exists(data))
      break;
    identifiers.push_back(identifier);
  }
  return identifiers;
}

void ScanIO_ply::readPose(const char* dir_path,
					 const char* identifier,
					 double* pose)
{
  for (unsigned int i = 0; i < 6; ++i) pose[i] = 0.0;
}

bool ScanIO_ply::supports(IODataType type)
{
  return !!(type & (DATA_XYZ | DATA_REFLECTANCE | DATA_RGB));
}

void ScanIO_ply::readScan(const char* dir_path,
					 const char* identifier,
					 PointFilter& filter,
					 std::vector<double>* xyz,
					 std::vector<unsigned char>* rgb,
					 std::vector<float>* reflectance,
					 std::vector<float>* temperature,
					 std::vector<float>* amplitude,
					 std::vector<int>* type,
					 std::vector<float>* deviation)
{
  unsigned int i;
  
  // error handling
  path data_path(dir_path);
  data_path /= path(std::string(DATA_PATH_PREFIX) +
				identifier + DATA_PATH_SUFFIX);
  if(!exists(data_path))
    throw std::runtime_error(std::string("There is no scan file for [")
					    + identifier + "] in [" + dir_path + "]");

  // open data file
  ifstream data_file;
  data_file.open(data_path);
  data_file.exceptions(ifstream::eofbit|ifstream::failbit|ifstream::badbit);
  
  if(xyz != 0 && rgb != 0 && reflectance != 0) {

    // read ply file
    bool binary = false;
    char dummy[256];
    char str[20]; // whatever size
    double matrix[16];
    int matrixPos = 0;
    int nr;
    float d1,d2,d3,d4;

    // header
    int counter = -2;
    do {
	 if (counter > -2) counter++;
	 if (data_file.good()) {
	   data_file.getline(dummy, 255);
	 }
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
    } while (!(strncmp(dummy, "end_header",10) == 0 || !data_file.good()));

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
	   switch(counter) {
        case 6:
        case 12:
          data_file >> p.z >> p.y >> p.x >> r >> g >> b;
          break;
        case 9:
		data_file >> p.z >> p.y >> p.x
				>> dummy >> dummy >> dummy
				>> r >> g >> b;
          break;
        default: 
		data_file >> p.z >> p.x >> p.y >> confidence >> intensity;
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
	   data_file.read((char*)&data, sizeof(float));
	   p.z = (double)data;
	   data_file.read((char*)&data, sizeof(float));
	   p.x = (double)data;
	   data_file.read((char*)&data, sizeof(float));
	   p.y = (double)data;
	   data_file.read((char*)&confidence, sizeof(float));
	   data_file.read((char*)&intensity, sizeof(float));
	 }

	 reflectance->push_back(p.reflectance);
	 xyz->push_back(p.x * 100);
	 xyz->push_back(p.y * 100);
	 xyz->push_back(p.z * 100);
	 rgb->push_back(static_cast<unsigned char>(p.rgb[0]));
	 rgb->push_back(static_cast<unsigned char>(p.rgb[1]));
	 rgb->push_back(static_cast<unsigned char>(p.rgb[2]));
    }
  }
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
