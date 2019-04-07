/**
 * @file
 * @brief IO of a 3D scan in uos file format
 * @author Thomas Escher
 */

#ifndef __SCAN_IO_UOS_RGB_H__
#define __SCAN_IO_UOS_RGB_H__

#include "scan_io.h"



/**
 * @brief 3D scan loader for UOS scans with color information
 *
 * The compiled class is available as shared object file
 */
class ScanIO_uos_rgb : public ScanIO {
protected:
  static IODataType spec[];
  virtual IODataType* getSpec() { return spec; }
  virtual ScanDataTransform& getTransform() { return transform2uos; }
};

#endif
