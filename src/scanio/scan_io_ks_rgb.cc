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



#define DATA_PATH_PREFIX "Color_ScanPos"
#define DATA_PATH_SUFFIX " - Scan001.txt"

std::list<std::string> ScanIO_ks_rgb::readDirectory(const char* dir_path, unsigned int start, unsigned int end)
{
    const char* suffixes[2] = { DATA_PATH_SUFFIX, NULL };
    return readDirectoryHelper(dir_path, start, end, suffixes, DATA_PATH_PREFIX);
}

void ScanIO_ks_rgb::readPose(const char* dir_path, const char* identifier, double* pose)
{
    readPoseHelper(dir_path, identifier, pose);
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
    for(unsigned int i = 0; i < 3; ++i) pose[i] *= 100.0;
*/
}

bool ScanIO_ks_rgb::supports(IODataType type)
{
  return !!(type & (DATA_XYZ | DATA_RGB | DATA_REFLECTANCE | DATA_AMPLITUDE));
}

void ScanIO_ks_rgb::readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation)
{
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
        // TODO: how does the first line look like?
        //       can we use uosHeaderTest() here?
        char dummy[255];
        data_file.getline(dummy, 255);

        IODataType spec[9] = { DATA_XYZ, DATA_XYZ, DATA_XYZ,
            DATA_RGB, DATA_RGB, DATA_RGB,
            DATA_AMPLITUDE, DATA_REFLECTANCE, DATA_TERMINATOR };
        ScanDataTransform_ks transform;
        readASCII(data_file, spec, transform, filter, xyz, rgb, reflectance, 0, amplitude);

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
