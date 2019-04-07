/**
 * @file scan_io_faro_xyz_rgbr.h
 * @brief IO of a 3D scan in xyz file format plus an intensity

 * @author Hamidreza Houshiar
 */

#ifndef __SCAN_IO_FARO_XYZ_RGBR_H__
#define __SCAN_IO_FARO_XYZ_RGBR_H__

#include "scan_io.h"



/**
 * @brief IO of a 3D scan in uos file format plus a
 *        reflectance/intensity/temperature value
 *
 * The compiled class is available as shared object file
 */
class ScanIO_faro_xyz_rgbr : public ScanIO {
protected:
  static const char* data_suffix;
  static IODataType spec[];
  static ScanDataTransform& transform2uos;

  virtual const char* dataSuffix() { return data_suffix; }
  virtual IODataType* getSpec() { return spec; }
  virtual ScanDataTransform& getTransform() { return transform2uos; }
};

#endif
