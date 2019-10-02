/*
 * get appropriate scan reduce parameter to reduce scan
 * for better performance in poisson surface reconstruction
 *
 */
#ifndef __AUTO_SCAN_RED_H__
#define __AUTO_SCAN_RED_H__

#include <vector>
#include <float.h>
#include <slam6d/globals.icc>
#include <slam6d/scan.h>

struct RedParam {
  double voxelSize = -1.0;
  int ptsPerVerxel = 1;
};

// get reduction parameters for joined scans
int getRedParam(RedParam &rp);

// get reduction parameters for individual scans
int getRedParam(RedParam &rp, Scan* scan);

// get memory info
unsigned long getTotalMem();
unsigned long getFreeMem();

#endif
