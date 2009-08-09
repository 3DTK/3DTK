/**
 * @file
 * @brief IO of a 3D scan
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SCAN_IO_H__
#define __SCAN_IO_H__

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "point.h"

/**
 * @brief IO of a 3D scan
 *
 * This class needs to be instantiated by a class loading 
 * 3D scans from different file formats.
 */
class ScanIO {
public:
  /**
   *	Default constructor
   */
  ScanIO() {}
  /**
   *	Destructor
   */
  virtual ~ScanIO() {}

  /**
   * Reads specified scans from given directory.
   *
   * Scan poses will NOT be initialized after a call
   * to this function.
   *
   * This function is pure virtual. The actual functions
   * for loading a 3D scan are called scan_io_*
   * 
   * @param start Starts to read with this scan
   * @param end Stops with this scan
   * @param dir The directory from which to read
   * @param maxDist Reads only Points up to this Distance
   * @param minDist Reads only Points from this Distance
   * @param euler Initital pose estimates (will not be applied to the points
   * @param ptss Vector containing the read points
   */
  virtual int readScans(int start, int end, string &dir, int maxDist, int mindist,
				    double *euler, vector<Point> &ptss) = 0; 
};

// Since the shared object files are loaded on the fly, we
// need class factories

// the types of the class factories
typedef ScanIO* create_sio();
typedef void destroy_sio(ScanIO*);

#endif
