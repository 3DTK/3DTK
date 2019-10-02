/**
 * @file scan_io_leica_xyzr.h
 * @brief IO of a 3D scan in xyz file format plus an intensity

 * @author Billy Okal
 */

#ifndef __SCAN_IO_LEICA_XYZR_H__
#define __SCAN_IO_LEICA_XYZR_H__

#include "scan_io.h"



/**
 * @brief IO of a 3D scan in uos file format plus a
 *        reflectance/intensity/temperature value
 *
 * The compiled class is available as shared object file
 */
class ScanIO_leica_xyzr : public ScanIO {
public:
  virtual void readScan(const char* dir_path,
			const char* identifier,
			PointFilter& filter,
			std::vector<double>* xyz,
			std::vector<unsigned char>* rgb,
			std::vector<float>* reflectance,
			std::vector<float>* temperature,
			std::vector<float>* amplitude,
			std::vector<int>* type,
			std::vector<float>* deviation,
      std::vector<double>* normal);

protected:
  static const char* data_suffix;
  static IODataType spec[];
  static ScanDataTransform& transform2uos;

  virtual const char* dataSuffix() { return data_suffix; }
  virtual IODataType* getSpec() { return spec; }
  virtual ScanDataTransform& getTransform() { return transform2uos; }
};

#endif
