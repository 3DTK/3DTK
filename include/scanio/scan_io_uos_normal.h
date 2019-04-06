/**
 * @file scan_io_uos_normal.h
 * @brief IO of a 3D scan in uos file format
 * withnormals
 * @author Helge A. Lauterbach
 */

#ifndef __SCAN_IO_UOS_NORMAL_H__
#define __SCAN_IO_UOS_NORMAL_H__

#include "scan_io.h"



/**
 * @brief IO of a 3D scan in uos file format plus a
 *        reflectance/intensity/temperature value
 *
 * The compiled class is available as shared object file
 */
class ScanIO_uos_normal : public ScanIO {
public:
  virtual std::list<std::string> readDirectory(const char* dir_path, unsigned int start, unsigned int end);
  virtual void readPose(const char* dir_path, const char* identifier, double* pose);
  virtual time_t lastModified(const char* dir_path, const char* identifier);
  virtual void readScan(const char* dir_path,
      const char* identifier, 
      PointFilter& filter, 
      std::vector<double>* xyz, 
      std::vector<unsigned char>* 
      rgb, std::vector<float>* reflectance, 
      std::vector<float>* temperature, 
      std::vector<float>* amplitude, 
      std::vector<int>* type, 
      std::vector<float>* deviation, 
      std::vector<double>* normal);
  virtual bool supports(IODataType type);
};

#endif
