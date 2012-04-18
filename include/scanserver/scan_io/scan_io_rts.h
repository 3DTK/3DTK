/**
 * @file
 * @brief IO of a 3D scan in rts file format
 * @author Thomas Escher
 */

#ifndef __SCAN_IO_UOS_H__
#define __SCAN_IO_UOS_H__

#include "scan_io.h"



/**
 * @brief 3D scan loader for RTS scans
 *
 * The compiled class is available as shared object file
 */
class ScanIO_rts : public ScanIO {
public:
  virtual std::list<std::string> readDirectory(const char* dir_path, unsigned int start, unsigned int end);
  virtual void readPose(const char* dir_path, const char* identifier, double* pose);
  virtual void readScan(const char* dir_path, const char* identifier, PointFilter&& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation);
  virtual bool supports(IODataType type);
private:
  std::string cached_dir;
  std::vector<double> cached_poses;
};

#endif
