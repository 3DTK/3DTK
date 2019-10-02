/*
 * scan_io_xyzr implementation
 *
 * Copyright (C) Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file scan_io_xyzr.cc
 * @brief IO of a 3D scan in xyz file format plus a reflectance/intensity
 * @author Andreas Nuechter. Jacobs University Bremen, Germany.
 */

#include "scanio/scan_io_leica_xyzr.h"

#include <boost/filesystem/operations.hpp>
#include <boost/filesystem/fstream.hpp>
using namespace boost::filesystem;

const char* ScanIO_leica_xyzr::data_suffix = ".xyz";
IODataType ScanIO_leica_xyzr::spec[] = { DATA_XYZ, DATA_XYZ, DATA_XYZ,
          DATA_RGB, DATA_RGB, DATA_REFLECTANCE, DATA_TERMINATOR };
ScanDataTransform_xyz scanio_leica_xyzr_tf;
ScanDataTransform& ScanIO_leica_xyzr::transform2uos = scanio_leica_xyzr_tf;


void ScanIO_leica_xyzr::readScan(const char* dir_path,
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
  unsigned int i;

  // error handling
  path data_path(dir_path);
  data_path /= path(std::string(dataPrefix())
		    + identifier
		    + dataSuffix());
  if(!exists(data_path))
    throw std::runtime_error(std::string("There is no scan file for [")
			     + identifier + "] in ["
			     + dir_path + "]");

  if(xyz != 0) {
    // open data file
    ifstream data_file(data_path);
    // overread the first line ignoring the header information
    char dummy[255];
    data_file.getline(dummy, 255);

    // read points and reflectance/intensity/temperature value
    double point[3];
    float reflection;
    while(data_file.good()) {
      try {
        data_file >> point[2];
        data_file >> point[0];
        data_file >> point[1];
        point[0] *= -1.0;
	point[0] *= 100.0;
	point[1] *= 100.0;
	point[2] *= 100.0;
        data_file >> reflection;
/*
	point[0] -= 485531.0;
	point[1] -= 5882078.400;
	point[2] -= 52;
*/
      } catch(std::ios_base::failure& e) {
        break;
      }

      // apply filter then insert point and reflectance
      if(filter.check(point)) {
        for(i = 0; i < 3; ++i) xyz->push_back(point[i]);
        reflectance->push_back(reflection);
      }
    }
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
  return new ScanIO_leica_xyzr;
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

