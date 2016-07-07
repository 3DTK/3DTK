/*
 * scan_io_ply implementation
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
 */

#include "scanio/scan_io_ply.h"
#include "scanio/helper.h"
#include "slam6d/point.h"
#include "rply.h"

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

#define DATA_PATH_PREFIX "scan"
#define DATA_PATH_SUFFIX ".ply"

std::list<std::string> ScanIO_ply::readDirectory(const char* dir_path,
									    unsigned int start,
									    unsigned int end)
{
    const char* suffixes[2] = { DATA_PATH_SUFFIX, NULL };
    return readDirectoryHelper(dir_path, start, end, suffixes);
}

void ScanIO_ply::readPose(const char* dir_path,
					 const char* identifier,
					 double* pose)
{
  for (unsigned int i = 0; i < 6; ++i) pose[i] = 0.0;
}

time_t ScanIO_ply::lastModified(const char* dir_path, const char* identifier)
{
  const char* suffixes[2] = { DATA_PATH_SUFFIX, NULL };
  return lastModifiedHelper(dir_path, identifier, suffixes);
}

bool ScanIO_ply::supports(IODataType type)
{
  return !!(type & (DATA_XYZ | DATA_REFLECTANCE | DATA_RGB));
}

int vertex_cb(p_ply_argument argument) {
	std::vector<double> *data;
	ply_get_argument_user_data(argument, (void**)&data, NULL);
	double value = ply_get_argument_value(argument);
	data->push_back(value);
	return 1;
}

int rgb_cb(p_ply_argument argument) {
	std::vector<unsigned char> *data;
	ply_get_argument_user_data(argument, (void**)&data, NULL);
	double value = ply_get_argument_value(argument);
	data->push_back(value);
	return 1;
}

void ScanIO_ply::readScan(const char* dir_path,
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
  data_path /= path(std::string(DATA_PATH_PREFIX) +
				identifier + DATA_PATH_SUFFIX);
  if(!exists(data_path))
    throw std::runtime_error(std::string("There is no scan file for [")
					    + identifier + "] in [" + dir_path + "]");

  p_ply ply = ply_open(data_path.string().c_str(), NULL, 0, NULL);
  if (!ply) {
	  throw std::runtime_error("ply_open failed");
  }
  if (!ply_read_header(ply)) {
	  throw std::runtime_error("ply_read_header failed");
  }
  long num = ply_set_read_cb(ply, "vertex", "x", vertex_cb, xyz, 0);
  ply_set_read_cb(ply, "vertex", "y", vertex_cb, xyz, 0);
  ply_set_read_cb(ply, "vertex", "z", vertex_cb, xyz, 0);
  ply_set_read_cb(ply, "vertex", "red", rgb_cb, rgb, 0);
  ply_set_read_cb(ply, "vertex", "green", rgb_cb, rgb, 0);
  ply_set_read_cb(ply, "vertex", "blue", rgb_cb, rgb, 0);
  if (!ply_read(ply)) {
	  throw std::runtime_error("ply_read failed");
  }
  ply_close(ply);
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
  return new ScanIO_ply;
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
