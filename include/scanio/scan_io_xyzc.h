/**
 * @file scan_io_uosc.h
 * @brief IO of a 3D scan in xyz file format plus a class (type)

 * @author Michael Neumann. University of Wuerzburg, Germany.
 * @author Billy Okal
 */

#ifndef __SCAN_IO_XYZC_H__
#define __SCAN_IO_XYZC_H__

#include "scan_io.h"



/**
 * @brief IO of a 3D scan in uos file format plus a
 *        reflectance/intensity/temperature value
 *
 * The compiled class is available as shared object file
 */
class ScanIO_xyzc : public ScanIO {
protected:
  static const char* data_suffix;
  static IODataType spec[];
  static ScanDataTransform& transform2uos;

  virtual const char* dataSuffix() { return data_suffix; }
  virtual IODataType* getSpec() { return spec; }
  virtual ScanDataTransform& getTransform() { return transform2uos; }
};

#endif
