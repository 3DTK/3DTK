/**
 * @file scan_io_uos_normal.h
 * @brief IO of a 3D scan in uos file format
 * withnormals
 * @author Helge A. Lauterbach
 */

#ifndef __SCAN_IO_UOS_NORMAL_H__
#define __SCAN_IO_UOS_NORMAL_H__

#include "scan_io.h"



/**
 * @brief IO of a 3D scan in uos file format plus a
 *        reflectance/intensity/temperature value
 *
 * The compiled class is available as shared object file
 */
class ScanIO_uos_normal : public ScanIO {
protected:
  static IODataType spec[];
  virtual IODataType* getSpec() { return spec; }
};

#endif
