/**
 * @file
 * @brief IO of 3D scans in XYZ file format (i.e., pure 3D data points only)
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SCAN_IO_XYZ_RGB_H__
#define __SCAN_IO_XYZ_RGB_H__

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "scan_io.h"

/**
 * @brief 3D scan loader for XYZ scans (i.e., pure 3D data points only)
 *
 * The compiled class is available as shared object file
 */
class ScanIO_xyz_rgb : public ScanIO {
public:
  virtual int readScans(int start, int end, string &dir, int maxDist, int mindist,
				    double *euler, vector<Point> &ptss);
};

// Since this shared object file is  loaded on the fly, we
// need class factories

// the types of the class factories
typedef ScanIO* create_sio();
typedef void destroy_sio(ScanIO*);

#endif
