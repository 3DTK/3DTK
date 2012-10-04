/*
 * scan_io_rts implementation
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
 * @author Thomas Escher
 */

#include "scanio/scan_io_rts.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <vector>
using std::vector;
#include <sstream>
using std::stringstream;

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

#include "slam6d/globals.icc"



#define DATA_PATH_PREFIX "scan3d_0_"
#define DATA_PATH_SUFFIX ".3d"
#define POSE_PATH_FILE "odometry_0_sync_interpol.dat"

//! RTS type flag for invalid points
#define TYPE_INVALID 0x10



std::list<std::string> ScanIO_rts::readDirectory(const char* dir_path, unsigned int start, unsigned int end)
{
  std::list<std::string> identifiers;
  // only a single pose file, can't do without one
  path pose_path(dir_path);
  pose_path /= POSE_PATH_FILE;
  if(exists(pose_path)) {
    for(unsigned int i = start; i <= end; ++i) {
      // identifier is a number (0-\infty)
      std::string identifier(to_string(i));
      // scan consists of data (.3d) files
      path data(dir_path);
      data /= path(std::string(DATA_PATH_PREFIX) + identifier + DATA_PATH_SUFFIX);
      // stop if part of a scan is missing or end by absence is detected
      if(!exists(data))
        break;
      identifiers.push_back(identifier);
    }
  }
  return identifiers;
}

void ScanIO_rts::readPose(const char* dir_path, const char* identifier, double* pose)
{
  unsigned int i;
  
  // if directory doesn't match the cached one, rebuild pose cache
  if(cached_dir != dir_path) {
    // check for pose file
    path pose_path(dir_path);
    pose_path /= POSE_PATH_FILE;
    if(!exists(pose_path))
      throw std::runtime_error(std::string("There is no pose file in [") + dir_path + "]");

    // open pose file once and read all poses
    ifstream pose_file(pose_path);
    pose_file.exceptions(ifstream::eofbit|ifstream::failbit|ifstream::badbit);

    vector<double> poses;
    double p[6], timestamp;
    while(pose_file.good()) {
      try {
        pose_file >> timestamp
          >> p[2] >> p[0] >> p[1] // x, y, z
          >> p[3] >> p[5] >> p[4]; // theta_x, theta_y, theta_z
      } catch(std::ios_base::failure& e) {
        break;
      }
      
      // convert
      for(i = 0; i < 3; ++i) p[i] *= 0.1;
      
      // add in poses
      for(i = 0; i < 6; ++i) poses.push_back(p[i]);
    }

    // after success, set the cache
    cached_poses.swap(poses);
    cached_dir = dir_path;
  }
  
  // get index from the identifier and pick the pose
  stringstream str(identifier);
  unsigned int scan_index;
  str >> scan_index;
  if(cached_poses.size() < scan_index*6 + 6)
    throw std::runtime_error(std::string("There is no pose entry for scan [") + identifier + "]");
  
  for(i = 0; i < 6; ++i)
    pose[i] = cached_poses[scan_index*6 + i];
  return;
}

bool ScanIO_rts::supports(IODataType type)
{
  return !!(type & (DATA_XYZ));
}

void ScanIO_rts::readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation)
{
  // TODO: Type and other columns?
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

    // read points
    // z x y type ? ?
    double point[3];
    int type, dummy;
    while(data_file.good()) {
      try {
        data_file >> point[2] >> point[0] >> point[1];
        data_file >> type >> dummy >> dummy;
      } catch(std::ios_base::failure& e) {
        break;
      }
      
      // convert
      point[0] *= 0.1;
      point[1] *= -0.1;
      point[2] *= 0.1;
      
      // apply filter and insert point
      if(!(type & TYPE_INVALID)) {
        if(filter.check(point)) {
          for(i = 0; i < 3; ++i) xyz->push_back(point[i]);
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
  return new ScanIO_rts;
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
