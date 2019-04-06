/**
 * @file
 * @brief IO of a 3D scan in rts file format
 * @author Thomas Escher
 */

#ifndef __SCAN_IO_RTS_H__
#define __SCAN_IO_RTS_H__

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
private:
  std::string cached_dir;
  std::vector<double> cached_poses;
protected:
  static const char* data_prefix;
  static IODataType spec[];
  static ScanDataTransform& transform2uos;

  virtual const char* dataPrefix() { return data_prefix; }
  virtual IODataType* getSpec() { return spec; }
  virtual ScanDataTransform& getTransform() { return transform2uos; }
};

#endif
