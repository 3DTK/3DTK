/*
 * slam6D implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany.
 * @author Thomas Escher. Inst. of CS, University of Osnabrueck, Germany.
 */

#include "scanio/scan_io_riegl_txt.h"

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

#include "slam6d/globals.icc"


const char* ScanIO_riegl_txt::data_suffix = ".txt";
const char* ScanIO_riegl_txt::pose_suffix = ".dat";
IODataType ScanIO_riegl_txt::spec[] = { DATA_XYZ, DATA_XYZ, DATA_XYZ,
            DATA_DUMMY, DATA_DUMMY, DATA_DUMMY, DATA_REFLECTANCE,
            DATA_TERMINATOR };
ScanDataTransform_riegl scanio_riegl_txt_tf;
ScanDataTransform& ScanIO_riegl_txt::transform2uos = scanio_riegl_txt_tf;


void ScanIO_riegl_txt::readPose(const char* dir_path,
                                const char* identifier,
                                double* pose)
{
  unsigned int i;

  path pose_path(dir_path);
  pose_path /= path(std::string(posePrefix())
                    + identifier
                    + poseSuffix());
  if(!exists(pose_path))
    throw std::runtime_error(std::string("There is no pose file for [")
                             + identifier + "] in [" + dir_path + "]");

  // open pose file
  ifstream pose_file(pose_path);
    double rPos[3], rPosTheta[16];
    double inMatrix[16], tMatrix[16];

    if (pose_file.fail()) {
      throw std::runtime_error(std::string("Pose file could not be opened for [")
                               + identifier + "] in [" + dir_path + "]");
    }

    for (i = 0; i < 16; ++i)
      pose_file >> inMatrix[i];
    pose_file.close();

    if (pose_file.fail()) {
      throw std::runtime_error(std::string("Pose file could not be read for [")
                               + identifier + "] in [" + dir_path + "]");
    }

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
}

std::function<bool (std::istream &data_file)> read_data(PointFilter& filter,
        std::vector<double>* xyz, std::vector<unsigned char>* rgb,
        std::vector<float>* reflectance, std::vector<float>* temperature,
        std::vector<float>* amplitude, std::vector<int>* type,
        std::vector<float>* deviation)
{
    return [=,&filter](std::istream &data_file) -> bool {
        unsigned int count;
        if (!(data_file >> count)) {
          return false;
        }

        // reserve enough space for faster reading
        try {
            xyz->reserve(3*count);
        } catch(...) {
            std::cerr << "failed allocating space for " << count << " points" << std::endl;
            throw;
        }
        if (reflectance != nullptr) {
            reflectance->reserve(count);
        }

        // read points
        // z x y range theta phi reflectance
        IODataType spec[8] = { DATA_XYZ, DATA_XYZ, DATA_XYZ,
            DATA_DUMMY, DATA_DUMMY, DATA_DUMMY, DATA_REFLECTANCE,
            DATA_TERMINATOR };
        ScanDataTransform_riegl transform;
        return readASCII(data_file, spec, transform, filter, xyz, nullptr, reflectance);
    };
}

void ScanIO_riegl_txt::readScan(const char* dir_path,
                                const char* identifier,
                                PointFilter& filter,
                                std::vector<double>* xyz,
                                std::vector<unsigned char>* rgb,
                                std::vector<float>* reflectance,
                                std::vector<float>* temperature,
                                std::vector<float>* amplitude,
                                std::vector<int>* type,
                                std::vector<float>* deviation,
                                std::vector<double>* normal)
{
    if(xyz == 0 && reflectance == 0)
        return;

    // error handling
    path data_path(dir_path);
    data_path /= path(std::string(dataPrefix())
            + identifier
            + dataSuffix());
    if (!open_path(data_path, read_data(filter, xyz, rgb, reflectance, temperature, amplitude, type, deviation)))
        throw std::runtime_error(std::string("There is no scan file for [")
                + identifier + "] in [" + dir_path + "]");
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
  return new ScanIO_riegl_txt;
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
