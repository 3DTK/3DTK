/**
 * @file scan_io_uosc.h
 * @brief IO of a 3D scan in uos file format plus a class (type)
 * @author Michael Neumann. University of Wuerzburg, Germany.
 * @author Billy Okal
 */

#ifndef __SCAN_IO_UOSC_H__
#define __SCAN_IO_UOSC_H__

#include "scan_io.h"



/**
 * @brief IO of a 3D scan in uos file format plus a
 *        type value
 *
 * The compiled class is available as shared object file
 */
class ScanIO_uosc : public ScanIO {
protected:
  static IODataType spec[];
  virtual IODataType* getSpec() { return spec; }
};

#endif
