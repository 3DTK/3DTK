/**
 * @file
 * @brief IO of a 3D scan
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SCAN_IO_RTS_H__
#define __SCAN_IO_RTS_H__

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "scan_io.h"

/**
 * @brief 3D scan representation
 */
class ScanIO_rts : public ScanIO {
public:
  virtual int readScans(int start, int end, string &dir, int maxDist, int mindist,
				    double *euler, vector<Point> &ptss); 
};

// the types of the class factories
typedef ScanIO* create_sio();
typedef void destroy_sio(ScanIO*);

#endif
