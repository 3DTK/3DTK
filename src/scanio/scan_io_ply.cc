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
#include "slam6d/point.h"
#include "rply.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <vector>
#include <string.h>
#include <map>

#ifdef _MSC_VER
#include <windows.h>
#endif

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

#include "slam6d/globals.icc"

const char* ScanIO_ply::data_suffix = ".ply";
IODataType ScanIO_ply::spec[] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_RGB, DATA_RGB, DATA_RGB, DATA_TERMINATOR };

void ScanIO_ply::readPose(const char* dir_path,
					 const char* identifier,
					 double* pose)
{
  for (unsigned int i = 0; i < 6; ++i) pose[i] = 0.0;
}

int vertex_cb(p_ply_argument argument) {
	std::vector<double> *data;
	ply_get_argument_user_data(argument, (void**)&data, NULL);
	double value = ply_get_argument_value(argument);
  data->push_back(value);
	return 1;
}

int vertex_cb_i(p_ply_argument argument) {
	std::vector<double> *data;
	ply_get_argument_user_data(argument, (void**)&data, NULL);
	double value = ply_get_argument_value(argument);
  value = -value;
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
           std::vector<float>* deviation,
           std::vector<double>* normal)
{
  // error handling
  path data_path(dir_path);
  data_path /= path(std::string(dataPrefix()) +
				identifier + dataSuffix());
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

  p_ply_element vertex_el = ply_get_next_element(ply, NULL);
  const char *vertex_el_name;
  if (!ply_get_element_info(vertex_el, &vertex_el_name, NULL)) {
	  throw std::runtime_error("ply_get_element_info failed");
  }

  if (strncmp(vertex_el_name, "vertex", 6)) {
	  throw std::runtime_error("expected first ply element to be vertex");
  }

  if (ply_get_next_element(ply, vertex_el) != NULL) {
	  throw std::runtime_error("expected only one element in ply file");
  }

  // make a map of the property names and their types
  std::map<std::string,e_ply_type> properties;
  p_ply_property prop = NULL;
  for (;;) {
	  prop = ply_get_next_property(vertex_el, prop);
	  if (prop == NULL) {
		  break;
	  }
	  const char *prop_name;
	  e_ply_type prop_type;
	  if (!ply_get_property_info(prop, &prop_name, &prop_type, NULL, NULL)) {
		  throw std::runtime_error("ply_get_property_info failed");
	  }
	  properties.insert(std::pair<std::string,e_ply_type>(std::string(prop_name), prop_type));
  }

  if (properties.find(std::string("x")) == properties.end()
		  || properties.find(std::string("y")) == properties.end()
		  || properties.find(std::string("z")) == properties.end()) {
	  throw std::runtime_error("ply file does not contain x/y/z coordinates");
  }

  ply_set_read_cb(ply, "vertex", "x", vertex_cb, xyz, 0);
  ply_set_read_cb(ply, "vertex", "y", vertex_cb, xyz, 0);
  ply_set_read_cb(ply, "vertex", "z", vertex_cb, xyz, 0);

  // We somehow want to extract color information from the PLY file, so we are
  // looking for properties named "red", "green" and "blue" or named
  // "diffuse_red", "diffuse_green" and "diffuse_blue". We use whichever is
  // found but default to the former if both are present.
  if (properties.find(std::string("red")) != properties.end()
		  && properties.find(std::string("green")) != properties.end()
		  && properties.find(std::string("blue")) != properties.end()) {
	  if (properties[std::string("red")] != PLY_UCHAR
	   || properties[std::string("green")] != PLY_UCHAR
	   || properties[std::string("blue")] != PLY_UCHAR) {
		  throw std::runtime_error("ply color values must be of type uchar");
	  }
	  ply_set_read_cb(ply, "vertex", "red", rgb_cb, rgb, 0);
	  ply_set_read_cb(ply, "vertex", "green", rgb_cb, rgb, 0);
	  ply_set_read_cb(ply, "vertex", "blue", rgb_cb, rgb, 0);
  } else if (properties.find(std::string("diffuse_red")) != properties.end()
		  && properties.find(std::string("diffuse_green")) != properties.end()
		  && properties.find(std::string("diffuse_blue")) != properties.end()) {
	  if (properties[std::string("diffuse_red")] != PLY_UCHAR
	   || properties[std::string("diffuse_green")] != PLY_UCHAR
	   || properties[std::string("diffuse_blue")] != PLY_UCHAR) {
		  throw std::runtime_error("ply color values must be of type uchar");
	  }
	  ply_set_read_cb(ply, "vertex", "diffuse_red", rgb_cb, rgb, 0);
	  ply_set_read_cb(ply, "vertex", "diffuse_green", rgb_cb, rgb, 0);
	  ply_set_read_cb(ply, "vertex", "diffuse_blue", rgb_cb, rgb, 0);
  } else {
	  throw std::runtime_error("ply file contains no color information");
  }

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
