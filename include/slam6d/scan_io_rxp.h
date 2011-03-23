/**
 * @file
 * @brief IO of 3D scans in Riegl file format
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SCAN_IO_RXP_H__
#define __SCAN_IO_RXP_H__

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "scan_io.h"
#include "riegl/scanlib.hpp"
using namespace scanlib;
using namespace std;
using namespace std::tr1;

/**
 * @brief 3D scan loader for Riegl scans in the binary rxp format
 *
 * The compiled class is available as shared object file
 */
class ScanIO_rxp : public ScanIO {
public:
  virtual int readScans(int start, int end, string &dir, int maxDist, int mindist,
				    double *euler, vector<Point> &ptss);
};


/**
 * The importer class is the interface to riegl's pointcloud class, and will convert their point struct to slam6d's point class.
 *
 * Code adapted from rivlib/example/pointcloudcpp.cpp available from http://www.riegl.com .
 */
class importer
    : public pointcloud
{
    vector<Point> *o;

public:
    importer(vector<Point> *o_, int maxDist, int minDist)
        : pointcloud(false) // set this to true if you need gps aligned timing
        , o(o_)
    {
      maxD = maxDist;
      minD = minDist;
    }

protected:
    int maxD;
    int minD;
    // overridden from pointcloud class
    void on_echo_transformed(echo_type echo);
};

// Since this shared object file is  loaded on the fly, we
// need class factories

// the types of the class factories
typedef ScanIO* create_sio();
typedef void destroy_sio(ScanIO*);

#endif
