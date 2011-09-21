/**
 * @file
 * @brief IO of a 3D scan for leica ASCII file format with relectance values.
 * @author Sven Albrecht. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SCAN_IO_LEICA_H__
#define __SCAN_IO_LEICA_H__

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "scan_io.h"

/**
 * @brief 3D scan loader for leica ASCII + reflectance scans
 *
 * The compiled class is available as shared object file
 */
class ScanIO_leica_txt : public ScanIO {
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
