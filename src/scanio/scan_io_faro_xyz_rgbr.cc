/*
 * scan_io_faro_xyz_rgbr implementation
 *
 * Copyright (C) Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file scan_io_faro_xyz_rgbr.cc
 * @brief IO of a 3D scan in xyz file format plus a reflectance/intensity
 * @author Andreas Nuechter. Jacobs University Bremen, Germany.
 */

#include "scanio/scan_io_faro_xyz_rgbr.h"
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
#define DATA_PATH_SUFFIX ".xyz"
#define POSE_PATH_PREFIX "scan"
#define POSE_PATH_SUFFIX ".pose"

std::list<std::string> ScanIO_faro_xyz_rgbr::readDirectory(const char* dir_path, 
						  unsigned int start, 
						  unsigned int end)
{
    const char* suffixes[2] = { DATA_PATH_SUFFIX, NULL };
    return readDirectoryHelper(dir_path, start, end, suffixes);
}

void ScanIO_faro_xyz_rgbr::readPose(const char* dir_path, 
			   const char* identifier, 
			   double* pose)
{
    readPoseHelper(dir_path, identifier, pose);
}

bool ScanIO_faro_xyz_rgbr::supports(IODataType type)
{
  return !!(type & ( DATA_REFLECTANCE | DATA_XYZ | DATA_RGB));
}

void ScanIO_faro_xyz_rgbr::readScan(const char* dir_path, 
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
    // error handling
    path data_path(dir_path);
    data_path /= path(std::string(DATA_PATH_PREFIX) 
            + identifier 
            + DATA_PATH_SUFFIX);
    if(!exists(data_path))
        throw std::runtime_error(std::string("There is no scan file for [") 
                + identifier + "] in [" 
                + dir_path + "]");

    if(xyz != 0 && rgb != 0 && reflectance != 0) {
        // open data file
        ifstream data_file(data_path);
        data_file.exceptions(ifstream::eofbit|ifstream::failbit|ifstream::badbit);

        uosHeaderTest(data_file);

        IODataType spec[8] = { DATA_XYZ, DATA_XYZ, DATA_XYZ,
            DATA_RGB, DATA_RGB, DATA_RGB, DATA_REFLECTANCE, DATA_TERMINATOR };
        ScanDataTransform_xyz transform;
        readASCII(data_file, spec, transform, filter, xyz, rgb, reflectance);
        
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
  return new ScanIO_faro_xyz_rgbr;
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

