/*
 * scan_io_laz implementation
 *
 * Copyright (C) Dorit Borrmann, Thomas Escher, Kai Lingemann, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file
 * @brief Implementation of reading 3D scans
 * @author Dorit Borrmann. Jacobs University Bremen, Germany.
 * @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Jacobs University Bremen, Germany.
 * @author Thomas Escher. Inst. of CS, University of Osnabrueck, Germany.
 * @author Razvan-George Mihalyi. Jacobs University Bremen, Germany.
 */

#include "scanio/scan_io_laz.h"
#include "scanio/helper.h"
#include "slam6d/point.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <vector>
#include <string.h>

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

#include "slam6d/globals.icc"

#define LAZ_SUFFIX ".laz"
#define LAS_SUFFIX ".las"
#define LAS_OPTIONS ".options"
#define DATA_PATH_PREFIX "scan"

std::list<std::string> ScanIO_laz::readDirectory(const char* dir_path,
						 unsigned int start,
						 unsigned int end)
{
  const char* suffixes[3] = { LAZ_SUFFIX, LAS_SUFFIX, NULL };
  return readDirectoryHelper(dir_path, start, end, suffixes);
}

void ScanIO_laz::readPose(const char* dir_path,
			  const char* identifier,
			  double* pose)
{
  for (unsigned int i = 0; i < 6; ++i) pose[i] = 0.0;
}

time_t ScanIO_laz::lastModified(const char* dir_path, const char* identifier)
{
  const char* suffixes[3] = { LAZ_SUFFIX, LAS_SUFFIX, NULL };
  return lastModifiedHelper(dir_path, identifier, suffixes);
}

bool ScanIO_laz::supports(IODataType type)
{
  return !!(type & (DATA_XYZ | DATA_REFLECTANCE | DATA_RGB));
}

void ScanIO_laz::readScan(const char* dir_path,
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
  // error handling
  path data_path(dir_path);
  data_path /= path(std::string(DATA_PATH_PREFIX) + identifier + LAZ_SUFFIX);
  if(!exists(data_path)) {
    // look for LAS suffix if LAZ suffix not present
    data_path = dir_path;
    data_path /= path(std::string(DATA_PATH_PREFIX) + identifier + LAS_SUFFIX);
    if(!exists(data_path))
      throw std::runtime_error(std::string("There is no scan file for [")
			       + identifier + "] in [" + dir_path + "]");
  }
  // open data file
  LASreadOpener lasreadopener;
  lasreadopener.set_file_name(data_path.string().c_str());

  //check and read options file
  path options_path(dir_path);
  options_path /= path(std::string(DATA_PATH_PREFIX) + identifier + LAS_OPTIONS);

  if(exists(options_path)) {

    std::ifstream opt_in;
    opt_in.open(options_path.c_str());

    std::string opts;
    getline(opt_in, opts);
    opt_in.close();
    opt_in.clear();

    char **opts_array;
    unsigned int optcount = strtoarray(opts, opts_array);
    lasreadopener.parse(optcount + 1, opts_array);

    for(unsigned int i = 1; i < optcount; i++) {
      delete[] opts_array[i];
    }
    delete[] opts_array;
  }

  LASreader* lasreader = lasreadopener.open();

  while (lasreader->read_point()) {
    double point[3] = { lasreader->point.get_x(),
			lasreader->point.get_y(),
			lasreader->point.get_z() };
    double point_tmp[3];

    if(xyz != 0) {
      //las and laz are usually in pts coordiante system (x is left to right, y is bottom to up, z is front to back)
      //otherwise use options (e.g. "-switch_y_z")
      point_tmp[0] = point[0];
      point_tmp[1] = point[1];
      point_tmp[2] = -1 * point[2];

      // apply filter
      if (filter.check(point_tmp)) {
	// push point
	xyz->push_back(point_tmp[0]);
	xyz->push_back(point_tmp[1]);
	xyz->push_back(point_tmp[2]);
      }
    }
    if (reflectance != 0) {
      /// if intensity doesn't exist, it's automatically set to 0.
      reflectance->push_back((float) lasreader->point.intensity);
    }
    if (rgb != 0) {
      if (lasreader->point.have_rgb) {
        unsigned int R = lasreader->point.rgb[0];
        unsigned int G = lasreader->point.rgb[1];
        unsigned int B = lasreader->point.rgb[2];

        rgb->push_back(static_cast<unsigned char>(R));
        rgb->push_back(static_cast<unsigned char>(G));
        rgb->push_back(static_cast<unsigned char>(B));
      }
    }
  }

  lasreader->close();
  delete lasreader;
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
  return new ScanIO_laz;
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
