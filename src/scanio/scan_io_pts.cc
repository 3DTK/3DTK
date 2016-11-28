/*
 * scan_io_pts implementation
 *
 * Copyright (C) Andreas Nuechter, Dorit Borrmann 
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file scan_io_pts.cc
 * @brief IO of a 3D scan in pts file format (right-handed coordinate system,
 * with x from left to right, y from bottom to up, z from front to back)
 * @author Hamidreza Houshiar. DenkmalDaten Winkler KG. Co. Muenster, Germany
 */

#include "scanio/scan_io_pts.h"
#include "scanio/helper.h"

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
#define DATA_PATH_SUFFIX ".pts"
#define POSE_PATH_PREFIX "scan"
#define POSE_PATH_SUFFIX ".pose"

std::list<std::string> ScanIO_pts::readDirectory(const char* dir_path, 
						  unsigned int start, 
						  unsigned int end)
{
    const char* suffixes[2] = { DATA_PATH_SUFFIX, NULL };
    return readDirectoryHelper(dir_path, start, end, suffixes);
}

void ScanIO_pts::readPose(const char* dir_path, 
			   const char* identifier, 
			   double* pose)
{
    readPoseHelper(dir_path, identifier, pose);
}

time_t ScanIO_pts::lastModified(const char* dir_path, const char* identifier)
{
  const char* suffixes[2] = { DATA_PATH_SUFFIX, NULL };
  return lastModifiedHelper(dir_path, identifier, suffixes);
}

bool ScanIO_pts::supports(IODataType type)
{
  return !!(type & ( DATA_XYZ ));
}

void ScanIO_pts::readScan(const char* dir_path, 
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
    if(xyz == 0)
        return;

    IODataType spec[4] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR };
    ScanDataTransform_pts transform;

    // error handling
    path data_path(dir_path);
    data_path /= path(std::string(DATA_PATH_PREFIX) 
            + identifier 
            + DATA_PATH_SUFFIX);
    if (!open_path(data_path, open_uos_file(spec, transform, filter, xyz, 0, 0, 0, 0, 0, 0)))
        throw std::runtime_error(std::string("There is no scan file for [") 
                + identifier + "] in [" 
                + dir_path + "]");
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
  return new ScanIO_pts;
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

/* vim: set ts=4 sw=4 et: */
