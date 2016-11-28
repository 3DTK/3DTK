/**
 * @file scan_io_uosr.h
 * @brief IO of a 3D scan in xyz file format plus an intensity

 * @author Hamidreza Houshiar
 */

#ifndef __SCAN_IO_XYZ_RGBR_H__
#define __SCAN_IO_XYZ_RGBR_H__

#include "scan_io.h"



/**
 * @brief IO of a 3D scan in xyz file format plus a
 *        RGB/reflectance value
 *
 * The compiled class is available as shared object file
 */
class ScanIO_xyz_rgbr : public ScanIO {
public:
  virtual std::list<std::string> readDirectory(const char* dir_path, 
					       unsigned int start, 
					       unsigned int end);
  virtual void readPose(const char* dir_path, 
			const char* identifier, 
			double* pose);
  virtual time_t lastModified(const char* dir_path, const char* identifier);
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
