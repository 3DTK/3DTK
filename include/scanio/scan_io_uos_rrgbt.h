/**
 * @file
 * @brief IO of a 3D scan in uos file format with reflectance, color and
 * temperature information
 * @author Dorit Borrmann
 */

#ifndef __SCAN_IO_UOS_RRGBT_H__
#define __SCAN_IO_UOS_RRGBT_H__

#include "scan_io.h"



/**
 * @brief 3D scan loader for UOS scans
 *
 * The compiled class is available as shared object file
 */
class ScanIO_uos_rrgbt : public ScanIO {
protected:
  static IODataType spec[];
  virtual IODataType* getSpec() { return spec; }
};

#endif
