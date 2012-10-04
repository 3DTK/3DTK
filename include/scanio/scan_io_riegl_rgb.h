/**
 * @file
 * @brief IO of a 3D scan in riegl_txt file format with added color information
 * @author Thomas Escher
 */

#ifndef __SCAN_IO_RIEGL_RGB_H__
#define __SCAN_IO_RIEGL_RGB_H__

#include "scan_io.h"



/**
 * @brief 3D scan loader for Riegl scans
 *
 * The compiled class is available as shared object file
 */
class ScanIO_riegl_rgb : public ScanIO {
public:
  virtual std::list<std::string> readDirectory(const char* dir_path, unsigned int start, unsigned int end);
  virtual void readPose(const char* dir_path, const char* identifier, double* pose);
  virtual void readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation);
  virtual bool supports(IODataType type);
};

#endif
