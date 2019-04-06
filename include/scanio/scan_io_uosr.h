/**
 * @file scan_io_uosr.h
 * @brief IO of a 3D scan in uos file format plus a
 * reflectance/intensity/temperature value
 * @author Billy Okal
 */

#ifndef __SCAN_IO_UOSR_H__
#define __SCAN_IO_UOSR_H__

#include "scan_io.h"



/**
 * @brief IO of a 3D scan in uos file format plus a
 *        reflectance/intensity/temperature value
 *
 * The compiled class is available as shared object file
 */
class ScanIO_uosr : public ScanIO {
protected:
  static IODataType spec[];
  virtual IODataType* getSpec() { return spec; }
};

#endif
