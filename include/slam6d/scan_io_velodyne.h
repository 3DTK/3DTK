/**
 * @file
 * @brief IO of a 3D scan in velodyne raw file format
 * @author Liwei. Computer School of Wuhan University, China.
 */

#ifndef __SCAN_IO_VELODYNE_H__
#define __SCAN_IO_VELODYNE_H__

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "scan_io.h"

/**
 * @brief 3D scan loader for VELODYNE scans
 *
 * The compiled class is available as shared object file
 */
class ScanIO_velodyne : public ScanIO {
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
