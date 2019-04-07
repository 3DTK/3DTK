/*
 * scan_io_ks implementation
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

#include "scanio/scan_io_ks.h"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

const char* ScanIO_ks::data_prefix = "ScanPos";
const char* ScanIO_ks::data_suffix = " - Scan001.txt";
IODataType ScanIO_ks::spec[] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR };
ScanDataTransform_ks scanio_ks_tf;
ScanDataTransform& ScanIO_ks::transform2uos = scanio_ks_tf;

void ScanIO_ks::readPose(const char* dir_path, const char* identifier, double* pose)
{
    readPoseHelper(dir_path, identifier, pose);
    // CAD map -> pose correction [ks x/y/z -> slam -z/y/x]
    double tmp;
    tmp = pose[0];
    pose[0] = - pose[2];
    pose[2] = tmp;
    // convert coordinate to cm
    for(unsigned int i = 0; i < 3; ++i) pose[i] *= 100.0;
}


std::function<bool (std::istream &data_file)> read_data(PointFilter& filter,
        std::vector<double>* xyz, std::vector<unsigned char>* rgb,
        std::vector<float>* reflectance, std::vector<float>* temperature,
        std::vector<float>* amplitude, std::vector<int>* type,
        std::vector<float>* deviation)
{
    return [=,&filter](std::istream &data_file) -> bool {
        // overread the first line
        // TODO: what does the first line look like?
        //       can we use uosHeaderTest() here?
        char dummy[255];
        data_file.getline(dummy, 255);

        IODataType spec[4] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR };
        ScanDataTransform_ks transform;

        readASCII(data_file, spec, transform, filter, xyz);

        return true;
    };
}


void ScanIO_ks::readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation,
               std::vector<double>* normal)
{
    if(xyz == 0)
        return;

    // error handling
    path data_path(dir_path);
    data_path /= path(std::string(dataPrefix()) + identifier + dataSuffix());
    if (!open_path(data_path, read_data(filter, xyz, rgb, reflectance, temperature, amplitude, type, deviation)))
        throw std::runtime_error(std::string("There is no scan file for [") + identifier + "] in [" + dir_path + "]");
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
  return new ScanIO_ks;
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
