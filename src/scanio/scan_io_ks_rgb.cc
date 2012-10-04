/*
 * scan_io_ks_rgb implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Thomas Escher. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "scanio/scan_io_ks_rgb.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <vector>

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

#include "slam6d/globals.icc"



#define DATA_PATH_PREFIX "Color_ScanPos"
#define DATA_PATH_SUFFIX " - Scan001.txt"
#define POSE_PATH_PREFIX "scan"
#define POSE_PATH_SUFFIX ".pose"



std::list<std::string> ScanIO_ks_rgb::readDirectory(const char* dir_path, unsigned int start, unsigned int end)
{
  std::list<std::string> identifiers;
  for(unsigned int i = start; i <= end; ++i) {
    // identifier is /d/d/d (001-999)
    std::string identifier(to_string(i,3));
    // scan consists of data (.txt) and pose (.pose) files
    path data(dir_path);
    data /= path(std::string(DATA_PATH_PREFIX) + identifier + DATA_PATH_SUFFIX);
    path pose(dir_path);
    pose /= path(std::string(POSE_PATH_PREFIX) + identifier + POSE_PATH_SUFFIX);
    // stop if part of a scan is missing or end by absence is detected
    if(!exists(data) || !exists(pose))
      break;
    identifiers.push_back(identifier);
  }
  return identifiers;
}

void ScanIO_ks_rgb::readPose(const char* dir_path, const char* identifier, double* pose)
{
  unsigned int i;
  
  path pose_path(dir_path);
  pose_path /= path(std::string(POSE_PATH_PREFIX) + identifier + POSE_PATH_SUFFIX);
  if(!exists(pose_path))
    throw std::runtime_error(std::string("There is no pose file for [") + identifier + "] in [" + dir_path + "]");
  
  // open pose file
  ifstream pose_file(pose_path);
  
  // if the file is open, read contents
  if(pose_file.good()) {
    // read 6 plain doubles
    for(i = 0; i < 6; ++i) pose_file >> pose[i];
    pose_file.close();
/*
Don't use this part of io_ks because the original io_ks_rgb didn't
have it either, reasoning was "we never got ks_rgb scans with pose", so
the pose files created were already corrected in terms of offset and
scaling
    // CAD map -> pose correction [ks x/y/z -> slam -z/y/x]
    double tmp;
    tmp = pose[0];
    pose[0] = - pose[2];
    pose[2] = tmp;
    
    // convert coordinate to cm
    for(i = 0; i < 3; ++i) pose[i] *= 100.0;
*/
    // convert angles from deg to rad
    for(i = 3; i < 6; ++i) pose[i] = rad(pose[i]);
  } else {
    throw std::runtime_error(std::string("Pose file could not be opened for [") + identifier + "] in [" + dir_path + "]");
  }
}

bool ScanIO_ks_rgb::supports(IODataType type)
{
  return !!(type & (DATA_XYZ | DATA_RGB | DATA_REFLECTANCE | DATA_AMPLITUDE));
}

void ScanIO_ks_rgb::readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation)
{
  unsigned int i;
  
  // error handling
  path data_path(dir_path);
  data_path /= path(std::string(DATA_PATH_PREFIX) + identifier + DATA_PATH_SUFFIX);
  if(!exists(data_path))
    throw std::runtime_error(std::string("There is no scan file for [") + identifier + "] in [" + dir_path + "]");

  
  // TODO: support for amplitude and reflectance
  if(xyz != 0 || rgb != 0 || reflectance != 0 || amplitude != 0) {
    // open data file
    ifstream data_file(data_path);
    data_file.exceptions(ifstream::eofbit|ifstream::failbit|ifstream::badbit);
    
    // overread the first line
    char dummy[255];
    data_file.getline(dummy, 255);
    
    // read points
    double point[8];
    double tmp;
    while(data_file.good()) {
      try {
        for(i = 0; i < 8; ++i) data_file >> point[i];
      } catch(std::ios_base::failure& e) {
        break;
      }
      
      // still convert the points, needed for range check
      
      // the enemy's x/y/z is mapped to slam's x/z/y, shuffle time!
      tmp = point[1];
      point[1] = point[2];
      point[2] = tmp;
      
      // TODO: offset is application specific, handle with care
      // correct constant offset (in slam coordinates)
      point[0] -= 70000.0; // x
      point[2] -= 20000.0; // z
    
      // convert coordinate to cm
      for(i = 0; i < 3; ++i) point[i] *= 100.0;
      
      // apply filter and insert point
      if(filter.check(point)) {
        if(xyz != 0) {
          for(i = 0; i < 3; ++i) xyz->push_back(point[i]);
        }
        if(rgb != 0) {
          for(i = 3; i < 6; ++i) rgb->push_back(
            static_cast<unsigned char>(point[i] * 255.0));
        }
        if(reflectance != 0) {
          reflectance->push_back(point[7]);
        }
        if(amplitude != 0) {
          amplitude->push_back(point[6]);
        }
      }
    }
    data_file.close();
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
  return new ScanIO_ks_rgb;
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
