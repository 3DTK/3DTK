/*
 * scan_io implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

#include "scanio/scan_io.h"

using std::map;
using std::pair;
using std::string;
#include <stdexcept>
using std::runtime_error;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#ifdef _WIN32
#include <windows.h>
#else
#include <dlfcn.h>
#endif

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

#include "slam6d/globals.icc"

const char* ScanIO::data_prefix = "scan";
const char* ScanIO::data_suffix = ".3d";
const char* ScanIO::pose_prefix = "scan";
const char* ScanIO::pose_suffix = ".pose";

IODataType ScanIO::spec[] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR };
ScanDataTransform_identity scanio_tf;
ScanDataTransform& ScanIO::transform2uos = scanio_tf;

map<IOType, ScanIO *> ScanIO::m_scanIOs;

ScanIO * ScanIO::getScanIO(IOType iotype)
{
  // get the ScanIO from the map
  map<IOType, ScanIO*>::iterator it = m_scanIOs.find(iotype);
  if(it != m_scanIOs.end())
    return it->second;

  // figure out the full and correct library name
  string libname(io_type_to_libname(iotype));
#if defined(__MINGW32__)
  libname = "lib" + libname + ".dll";
#elif defined(_WIN32)
  libname += ".dll";
#elif defined(__APPLE__)
  libname = "lib" + libname + ".dylib";
#elif defined(__CYGWIN__)
  libname = "cyg" + libname + ".dll";
#else
  libname = "lib" + libname + ".so";
#endif

  //cout << "Loading shared library " << libname << " ... " << std::flush;

  // load the library and symbols
#ifdef _WIN32
  HINSTANCE hinstLib = LoadLibrary(libname.c_str());
  if (!hinstLib)
    throw runtime_error(string("Cannot load library ") + libname);

  //cout << "done." << endl;

  create_sio* create_ScanIO = (create_sio*)GetProcAddress(hinstLib, "create");

  if (!create_ScanIO) {
    FreeLibrary(hinstLib);
    throw runtime_error("Cannot load symbol create_ScanIO");
  }

#else
  void *ptrScanIO = dlopen(libname.c_str(), RTLD_LAZY);

  if (!ptrScanIO)
    throw runtime_error(string("Cannot load library ") + libname + string(": ") + dlerror());

  //cerr << "done." << endl;

  // reset the errors
  dlerror();

  // load the symbols
  create_sio* create_ScanIO = (create_sio*)dlsym(ptrScanIO, "create");
  const char* dlsym_error = dlerror();
  if (dlsym_error) {
    dlclose(ptrScanIO);
    throw runtime_error(string("Cannot load symbol create_ScanIO: ") + dlsym_error);
  }
#endif

  // create an instance of ScanIO, save it in the map and return it
  ScanIO* sio = create_ScanIO();
  m_scanIOs.insert(pair<IOType, ScanIO*>(iotype, sio));
  return sio;
}

void ScanIO::clearScanIOs()
{
  if (m_scanIOs.size()){
    for (map<IOType, ScanIO*>::iterator it = m_scanIOs.begin(); it != m_scanIOs.end(); ++it) {
      // figure out the full and correct library name
      string libname(io_type_to_libname(it->first));
#if defined(__MINGW32__)
      libname = "lib" + libname + ".dll";
#elif defined(_WIN32)
      libname += ".dll";
#elif defined(__APPLE__)
      libname = "lib" + libname + ".dylib";
#elif defined(__CYGWIN__)
      libname = "cyg" + libname + ".dll";
#else
      libname = "lib" + libname + ".so";
#endif

      // load library, destroy the allocated ScanIO and then remove it
#ifdef _WIN32
      HINSTANCE hinstLib = LoadLibrary(libname.c_str());

      destroy_sio* destroy_ScanIO = (destroy_sio*)GetProcAddress(hinstLib, "destroy");
      destroy_ScanIO(it->second);

      FreeLibrary(hinstLib);
#else
      void *ptrScanIO = dlopen(libname.c_str(), RTLD_LAZY);

      destroy_sio* destroy_ScanIO = (destroy_sio*)dlsym(ptrScanIO, "destroy");
      destroy_ScanIO(it->second);

      dlclose(ptrScanIO);
#endif
    }
  m_scanIOs.clear();
  }

}

std::list<std::string> ScanIO::readDirectory(const char* dir_path, unsigned int start, unsigned int end)
{
  const char* data_path_suffixes[2] = { dataSuffix(), NULL };
  return readDirectoryHelper(dir_path, start, end, data_path_suffixes, dataPrefix());
}

std::list<std::string> ScanIO::readDirectory(dataset_settings& dss)
{
  //return readDirectory(dss.data_source.c_str(), dss.scan_numbers.min, dss.scan_numbers.max);
  const char* data_path_suffixes[2] = { dataSuffix(), NULL };
  return readDirectoryHelper(dss, data_path_suffixes, dataPrefix());
}

void ScanIO::readPose(const char* dir_path, const char* identifier, double* pose)
{
  readPoseHelper(dir_path, identifier, pose, poseSuffix(), posePrefix());
}

void ScanIO::readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation,
  std::vector<double>* normal)
{
  if (supports(DATA_XYZ)) { if (xyz == 0) return; } else xyz = 0;
  if (supports(DATA_RGB)) { if (rgb == 0) return; } else rgb = 0;
  if (supports(DATA_REFLECTANCE)) { if (reflectance == 0) return; } else reflectance = 0;
  if (supports(DATA_TEMPERATURE)) { if (temperature == 0) return; } else temperature = 0;
  if (supports(DATA_AMPLITUDE)) { if (amplitude == 0) return; } else amplitude = 0;
  if (supports(DATA_TYPE)) { if (type == 0) return; } else type = 0;
  if (supports(DATA_DEVIATION)) { if (deviation == 0) return; } else deviation = 0;
  if (supports(DATA_NORMAL)) { if (normal == 0) return; } else normal = 0;

  ScanDataTransform transform;

  std::string subscan_id(identifier);
  bool is_single_scan = subscan_id.find_first_of(':') == std::string::npos;

  if (!is_single_scan) {
    multi_range<range<int> > mr;
    mr.set(identifier);
    mr.merged = true;
    double pose[6], first_pose[6];
    double P[16], FPinv[16], Pdiff[16];
    auto it = mr.begin();
    readPose(dir_path, to_string(*it, 3).c_str(), first_pose);
    EulerToMatrix4(first_pose, first_pose + 3, P);
    M4inv(P, FPinv);

    for (; !it.done(); ++it) {
  path data_path(dir_path);
      subscan_id = to_string(*it, 3);
      if (it != mr.begin()) {
        readPose(dir_path, subscan_id.c_str(), pose);
        EulerToMatrix4(pose, pose + 3, P);
        MMult(FPinv, P, Pdiff);
        transform = ScanDataTransform_combined(getTransform(), ScanDataTransform_matrix(Pdiff));
      }
      else {
        transform = getTransform();
      }
      data_path /= path(std::string(dataPrefix()) + subscan_id + dataSuffix());
      if (!open_path(data_path, open_uos_file(getSpec(), transform, filter, xyz, rgb, reflectance, temperature, amplitude, type, deviation, normal)))
        throw std::runtime_error(std::string("There is no scan file for [") + identifier + "] in [" + dir_path + "]");
    }
  }
  else {
    path data_path(dir_path);
    data_path /= path(std::string(dataPrefix()) + subscan_id + dataSuffix());
  if (!open_path(data_path, open_uos_file(getSpec(), getTransform(), filter, xyz, rgb, reflectance, temperature, amplitude, type, deviation, normal)))
    throw std::runtime_error(std::string("There is no scan file for [") + identifier + "] in [" + dir_path + "]");
}
}

bool ScanIO::supports(IODataType type)
{
  unsigned int supported = 0U;
  IODataType *specification = getSpec();
  for (int i = 0; specification[i] != DATA_TERMINATOR; i++) supported |= specification[i];
  return !!(type & supported);
}

time_t ScanIO::lastModified(const char* dir_path, const char* identifier)
{
  const char* suffixes[2] = { this->data_suffix, NULL };
  return lastModifiedHelper(dir_path, identifier, suffixes, dataPrefix());
}

