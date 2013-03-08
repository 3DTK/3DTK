/*
 * scan_io_rxp implementation
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

#include "scanio/scan_io_rxp.h"
#include "riegl/scanlib.hpp"

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

using namespace scanlib;
using namespace std;
using namespace std::tr1;



#define DATA_PATH_PREFIX "scan"
#define DATA_PATH_SUFFIX ".rxp"
#define POSE_PATH_PREFIX "scan"
#define POSE_PATH_SUFFIX ".pose"

/*
TODO: this file is still work in progress for change to the new scanserver workflow
this ScanIO has to distinguish a multi scan file and a directory of single scan files and is currently very messy handling these with the importer class
*/



std::list<std::string> ScanIO_rxp::readDirectory(const char* dir_path, unsigned int start, unsigned int end)
{
  std::list<std::string> identifiers;
  
  path pose(dir_path);
  if(is_regular_file(pose)) {
    // TODO: create identifiers for this case
    // a) from start to end, requires sanity checks and goodwill of user
    // b) iterate through the file and get last index
    // c) check last index by other means through the riegl api?
    // TEMP: implementing a) without sanity checks :)
    for(unsigned int i = start; i < end; ++i) {
      identifiers.push_back(to_string(i,3));
    }
  } else {
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
  }
  return identifiers;
}

void ScanIO_rxp::readPose(const char* dir_path, const char* identifier, double* pose)
{
  unsigned int i;
  
  path pose_path(dir_path);
  
  // if the directory actually marks a (multi scan) file, return zero pose
  // TODO: test if pose_path gets constructed correctly, see removal of trailing / in the old code
  if(is_regular_file(pose_path.string())) {
    for(i = 0; i < 6; ++i) pose[i] = 0.0;
    return;
  }
  
  pose_path /= path(std::string(POSE_PATH_PREFIX) + identifier + POSE_PATH_SUFFIX);
  if(!exists(pose_path))
    throw std::runtime_error(std::string("There is no pose file for [") + identifier + "] in [" + dir_path + "]");

  // open pose file
  boost::filesystem::ifstream pose_file(pose_path);
  
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

bool ScanIO_rxp::supports(IODataType type)
{
  return !!(type & (DATA_XYZ | DATA_REFLECTANCE | DATA_AMPLITUDE | DATA_DEVIATION | DATA_TYPE));
}

void ScanIO_rxp::readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation)
{
  
  path data_path(dir_path);
  
  // distinguish file and directory
  if(is_regular_file(data_path)) {
    stringstream str(identifier);
    int scan_index;
    str >> scan_index;
    // first clean up if they "directory" has changed
    // second case of resetting: rewinding index in case of non-sequential index access
    if(old_path != dir_path || (imp != 0 && imp->getCurrentScan() > scan_index)) {
      // TODO: I'm assuming here that I can simply do this
      if(rc != 0) { rc->close(); }
      if(dec != 0) { delete dec; dec = 0; }
      if(imp != 0) { delete imp; imp = 0; }
      // TODO ^
    }
    // create directory-persistent decoder
    if (!dec) {
      // remember path
      old_path = dir_path;
      
      // create new connection
      rc = basic_rconnection::create(data_path.string());
      rc->open();
      
      // decoder splits the binary file into readable chunks
      dec = new decoder_rxpmarker(rc);
      // importer interprets the chunks
      // TODO: this won't really work because filter is gone and xyz too and everything breaks
      // probably set new filter and xyz in the importer
      imp = new importer(filter, scan_index, xyz, reflectance, amplitude, type, deviation);
    }
    
    // set new filter and vectors
    imp->set(filter, xyz, reflectance, amplitude, type, deviation);

    buffer buf;

    // skip the first scans
    if(imp->getCurrentScan() < scan_index) {
      for(dec->get(buf); !dec->eoi(); dec->get(buf)) {
        imp->dispatch(buf.begin(), buf.end());
        if(imp->getCurrentScan() >= scan_index) break;
      }
    }
    if(dec->eoi()) return;
    int cscan = imp->getCurrentScan();
    // iterate over chunks, until the next scan is reached
    for(dec->get(buf); !dec->eoi(); dec->get(buf)) {
      imp->dispatch(buf.begin(), buf.end());
      if(imp->getCurrentScan() != cscan) break;
    }
    
    return;
  }
  
  // error handling
  data_path /= path(std::string(DATA_PATH_PREFIX) + identifier + DATA_PATH_SUFFIX);
  if(!exists(data_path))
    throw std::runtime_error(std::string("There is no scan file for [") + identifier + "] in [" + dir_path + "]");
  
  if(xyz != 0) {
    string data_path_str;
    data_path_str = "file://" + data_path.string();
    rc = basic_rconnection::create(data_path_str);
    rc->open();

    // decoder splits the binary file into readable chunks
    dec = new decoder_rxpmarker(rc);
    // importer interprets the chunks
    imp = new importer(filter, 0, xyz, reflectance, amplitude, type, deviation);

    // iterate over chunks
    buffer buf;
    for(dec->get(buf); !dec->eoi(); dec->get(buf)) {
      imp->dispatch(buf.begin(), buf.end());
    }

    //done
    rc->close();
    // TODO: clean up all these pointers, aren't they dangling?
  }
}

void importer::on_echo_transformed(echo_type echo)
{
  // for multi scan files, ignore those before start
  if (currentscan < start) return;
  
  // targets is a member std::vector that contains all
  // echoes seen so far, i.e. the current echo is always
  // indexed by target_count-1.
  target& t(targets[target_count - 1]);

  double point[3];
  point[0] = t.vertex[1]*-100.0;
  point[1] = t.vertex[2]*100.0;
  point[2] = t.vertex[0]*100.0;
  
  if(t.deviation < 10.0 && filter->check(point)) {
    if(xyz) {
      for(unsigned int i = 0; i < 3; ++i) xyz->push_back(point[i]);
    }
    if(reflectance) reflectance->push_back(t.reflectance);
    if(amplitude) amplitude->push_back(t.amplitude);
    if(deviation) deviation->push_back(t.deviation);
    if(type) {
      if(pointcloud::first == echo) type->push_back(0);
      else if(pointcloud::interior == echo) type->push_back(1);
      else if(pointcloud::last == echo) type->push_back(10);
      else if(pointcloud::single == echo) type->push_back(9);
    }
  }

  /*
  // target.reflectance
  // target.amplitude
  // target.deviation
  // target.time
  // target.vertex  point coordinates
  */
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
  return new ScanIO_rxp;
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
