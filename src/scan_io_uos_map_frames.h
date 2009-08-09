/**
 * @file
 * @brief IO of a 3D scan in uos file format
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SCAN_IO_UOS_MAP_FRAMES_H__
#define __SCAN_IO_UOS_MAP_FRAMES_H__

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "scan_io.h"

/**
 * @brief 3D scan loader for UOS scans
 *
 * The compiled class is available as shared object file
 */
class ScanIO_uos_map_frames : public ScanIO {
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
