/**
 * @file scan_io_pts.h
 * @brief IO of a 3D scan in pts file format (right-handed coordinate system,
 * with x from left to right, y from bottom to up, z from front to back)
 * @author Hamidreza Houshiar. DenkmalDaten Winkler KG. Co. Muenster, Germany
 */

#ifndef __SCAN_IO_PTS_H__
#define __SCAN_IO_PTS_H__

#include "scan_io.h"

/**
 * @brief IO of a 3D scan in pts file format
 *
 * The compiled class is available as shared object file
 */
class ScanIO_pts : public ScanIO {
protected:
  static const char* data_suffix;
  static ScanDataTransform& transform2uos;

  virtual const char* dataSuffix() { return data_suffix; }
  virtual ScanDataTransform& getTransform() { return transform2uos; }
};

#endif
