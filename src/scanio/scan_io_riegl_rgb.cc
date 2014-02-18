/*
 * scan_io_riegl_rgb implementation
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

#include "scanio/scan_io_riegl_rgb.h"
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
#define DATA_PATH_SUFFIX ".rgb"
#define POSE_PATH_PREFIX "scan"
#define POSE_PATH_SUFFIX ".dat"



std::list<std::string> ScanIO_riegl_rgb::readDirectory(const char* dir_path, unsigned int start, unsigned int end)
{
    const char* suffixes[2] = { DATA_PATH_SUFFIX, NULL };
    return readDirectoryHelper(dir_path, start, end, suffixes);
}

void ScanIO_riegl_rgb::readPose(const char* dir_path, const char* identifier, double* pose)
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
    double rPos[3], rPosTheta[16];
    double inMatrix[16], tMatrix[16];
    
    for (i = 0; i < 16; ++i)
      pose_file >> inMatrix[i];
    pose_file.close();
    
    // transform input pose
    tMatrix[0] = inMatrix[5];
    tMatrix[1] = -inMatrix[9];
    tMatrix[2] = -inMatrix[1];
    tMatrix[3] = -inMatrix[13];
    tMatrix[4] = -inMatrix[6];
    tMatrix[5] = inMatrix[10];
    tMatrix[6] = inMatrix[2];
    tMatrix[7] = inMatrix[14];
    tMatrix[8] = -inMatrix[4];
    tMatrix[9] = inMatrix[8];
    tMatrix[10] = inMatrix[0];
    tMatrix[11] = inMatrix[12];
    tMatrix[12] = -inMatrix[7];
    tMatrix[13] = inMatrix[11];
    tMatrix[14] = inMatrix[3];
    tMatrix[15] = inMatrix[15];
    
    Matrix4ToEuler(tMatrix, rPosTheta, rPos);
    
    pose[0] = 100*rPos[0];
    pose[1] = 100*rPos[1];
    pose[2] = 100*rPos[2];
    pose[3] = rPosTheta[0];
    pose[4] = rPosTheta[1];
    pose[5] = rPosTheta[2];
  } else {
    throw std::runtime_error(std::string("Pose file could not be opened for [") + identifier + "] in [" + dir_path + "]");
  }
}

bool ScanIO_riegl_rgb::supports(IODataType type)
{
  return !!(type & (DATA_XYZ | DATA_RGB | DATA_REFLECTANCE));
}

void ScanIO_riegl_rgb::readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation)
{
    // error handling
    path data_path(dir_path);
    data_path /= path(std::string(DATA_PATH_PREFIX) + identifier + DATA_PATH_SUFFIX);
    if(!exists(data_path))
        throw std::runtime_error(std::string("There is no scan file for [") + identifier + "] in [" + dir_path + "]");

    if(xyz != 0 || rgb != 0 || reflectance != 0) {
        // open data file
        ifstream data_file(data_path);
        data_file.exceptions(ifstream::eofbit|ifstream::failbit|ifstream::badbit);

        // read the point count
        // TODO: read this in properly, check for errors
        unsigned int count;
        data_file >> count;

        // reserve enough space for faster reading
        if(xyz != 0) xyz->reserve(3*count);
        if(rgb != 0) rgb->reserve(3*count);

        // read points
        // z x y range theta phi r g b reflectance
        IODataType spec[11] = { DATA_XYZ, DATA_XYZ, DATA_XYZ,
            DATA_DUMMY, DATA_DUMMY, DATA_DUMMY,
            DATA_RGB, DATA_RGB, DATA_RGB, DATA_REFLECTANCE,
            DATA_TERMINATOR };
        ScanDataTransform_riegl transform;
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
  return new ScanIO_riegl_rgb;
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
