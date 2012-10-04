/*
 * scan_io_uos implementation
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

#include "scanio/scan_io_uos.h"

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



#define DATA_PATH_PREFIX "scan"
#define DATA_PATH_SUFFIX ".3d"
#define POSE_PATH_PREFIX "scan"
#define POSE_PATH_SUFFIX ".pose"



std::list<std::string> ScanIO_uos::readDirectory(const char* dir_path, unsigned int start, unsigned int end)
{
  std::list<std::string> identifiers;
  for(unsigned int i = start; i <= end; ++i) {
    // identifier is /d/d/d (000-999)
    std::string identifier(to_string(i,3));
    // scan consists of data (.3d) and pose (.pose) files
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

void ScanIO_uos::readPose(const char* dir_path, const char* identifier, double* pose)
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
    
    // convert angles from deg to rad
    for(i = 3; i < 6; ++i) pose[i] = rad(pose[i]);
  } else {
    throw std::runtime_error(std::string("Pose file could not be opened for [") + identifier + "] in [" + dir_path + "]");
  }
}

bool ScanIO_uos::supports(IODataType type)
{
  return !!(type & (DATA_XYZ));
}

void ScanIO_uos::readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation)
{
  unsigned int i;
  
  // error handling
  path data_path(dir_path);
  data_path /= path(std::string(DATA_PATH_PREFIX) + identifier + DATA_PATH_SUFFIX);
  if(!exists(data_path))
    throw std::runtime_error(std::string("There is no scan file for [") + identifier + "] in [" + dir_path + "]");
  
  if(xyz != 0) {
    // open data file
    ifstream data_file(data_path);
    data_file.exceptions(ifstream::eofbit|ifstream::failbit|ifstream::badbit);

    
    // aquire header informations
  /* OPTIONAL: the header isn't always there, would require a sanity check
    unsigned int n, m;
    char[3] dummy;
    data_file >> n;
    for(unsigned int i = 0; i < 3; ++i) data_file >> dummy[i];
    data_file >> m;
    values.reserve(n*m*3);
  */
    // overread the first line
    char dummy[255];
    data_file.getline(dummy, 255);
    
    // read points
    double point[3];
    while(data_file.good()) {
      try {
        for(i = 0; i < 3; ++i) data_file >> point[i];
      } catch(std::ios_base::failure& e) {
        break;
      }
      
      // apply filter and insert point
      if(filter.check(point)) {
        for(i = 0; i < 3; ++i) xyz->push_back(point[i]);
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
  return new ScanIO_uos;
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
