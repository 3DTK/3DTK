/**
 * @file scan_io_uosr.h
 * @brief IO of a 3D scan in xyz file format plus an intensity

 * @author Dorit Borrmann
 */

#ifndef __SCAN_IO_XYZ_H__
#define __SCAN_IO_XYZ_H__

#include "scan_io.h"

/**
 * @brief IO of a 3D scan in uos file format
 *
 * The compiled class is available as shared object file
 */
class ScanIO_xyz : public ScanIO {
protected:
  static const char* data_suffix;
  static ScanDataTransform& transform2uos;

  virtual const char* dataSuffix() { return data_suffix; }
  virtual ScanDataTransform& getTransform() { return transform2uos; }
};

#endif
