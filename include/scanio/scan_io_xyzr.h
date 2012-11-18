/**
 * @file scan_io_uosr.h
 * @brief IO of a 3D scan in xyz file format plus an intensity

 * @author Billy Okal
 */

#ifndef __SCAN_IO_XYZR_H__
#define __SCAN_IO_XYZR_H__

#include "scan_io.h"



/**
 * @brief IO of a 3D scan in uos file format plus a
 *        reflectance/intensity/temperature value
 *
 * The compiled class is available as shared object file
 */
class ScanIO_xyzr : public ScanIO {
public:
  virtual std::list<std::string> readDirectory(const char* dir_path, 
					       unsigned int start, 
					       unsigned int end);
  virtual void readPose(const char* dir_path, 
			const char* identifier, 
			double* pose);
  virtual void readScan(const char* dir_path, 
			const char* identifier, 
			PointFilter& filter, 
			std::vector<double>* xyz, 
			std::vector<unsigned char>* rgb, 
			std::vector<float>* reflectance, 
			std::vector<float>* temperature, 
			std::vector<float>* amplitude, 
			std::vector<int>* type, 
			std::vector<float>* deviation);
  virtual bool supports(IODataType type);
};

#endif
