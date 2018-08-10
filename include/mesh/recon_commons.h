#ifndef __RECON_COMMONS_H__
#define __RECON_COMMONS_H__

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>

#include <slam6d/io_types.h>
#include <slam6d/globals.icc>
#include <slam6d/scan.h>
#include <scanserver/clientInterface.h>
#include <slam6d/point.h>
#include <slam6d/globals.icc>

#include "mesh/calc_normals.h"
#include "mesh/parse_options.h"
#include "mesh/poisson.h"
#include "mesh/auto_scan_red.h"

// read frams used by join scans
void readFrames(std::string dir, int start, int end, int frame, bool use_pose=false);
// convert vector of Points to vector of vector of float
void convert(std::vector<Point> &src, std::vector<std::vector<float>> &dst);

#endif
