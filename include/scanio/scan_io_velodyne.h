/**
 * @file
 * @brief IO of a 3D scan in velodyne raw file format
 * @author Liwei. Computer School of Wuhan University, China.
 */

#ifndef __SCAN_IO_VELODYNE_H__
#define __SCAN_IO_VELODYNE_H__

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "scan_io.h"

/**
 * @brief 3D scan loader for VELODYNE scans
 *
 * The compiled class is available as shared object file
 */
class ScanIO_velodyne : public ScanIO {
public:
  virtual std::list<std::string> readDirectory(const char* dir_path, unsigned int start, unsigned int end);
  virtual void readPose(const char* dir_path, const char* identifier, double* pose);
  virtual void readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation);
  virtual bool supports(IODataType type);

  int fileCounter;
};

#endif
