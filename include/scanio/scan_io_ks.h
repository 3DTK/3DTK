/**
 * @file
 * @brief IO of a 3D scan in uos file format
 * @author Thomas Escher
 */

#ifndef __SCAN_IO_KS_H__
#define __SCAN_IO_KS_H__

#include "scan_io.h"

/**
 * @brief 3D scan loader for KS scans
 *
 * The compiled class is available as shared object file
 */
class ScanIO_ks : public ScanIO {
public:
  virtual void readPose(const char* dir_path, const char* identifier, double* pose);
  virtual void readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz = 0, std::vector<unsigned char>* rgb = 0, std::vector<float>* reflectance = 0, std::vector<float>* temperature = 0, std::vector<float>* amplitude = 0, std::vector<int>* type = 0, std::vector<float>* deviation = 0, std::vector<double>* normal = 0);

protected:
  static const char* data_prefix;
  static const char* data_suffix;
  static IODataType spec[];
  static ScanDataTransform& transform2uos;

  virtual const char* dataPrefix() { return data_prefix; }
  virtual const char* dataSuffix() { return data_suffix; }
  virtual IODataType* getSpec() { return spec; }
  virtual ScanDataTransform& getTransform() { return transform2uos; }
};

#endif
