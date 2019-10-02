#include <string>
#include <fstream>
#include <numeric>
#include <algorithm>
#include "mesh/auto_scan_red.h"

// estimate reduction factors for joined scans
int
#ifndef _MSC_VER
__attribute__((optimize(0)))
#endif
getRedParam(RedParam &params) {
  unsigned long memBefore = 0, memAfter = 0, numPts = 0;

  // #ifdef __linux__
  // memBefore = getFreeMem();
  // #endif

  // retrieve all scans data for a rough memory estimation
  for (unsigned int i = 0; i < Scan::allScans.size(); ++i) {
    DataXYZ xyz = Scan::allScans[i]->get("xyz");
    numPts += xyz.size();
  }

  params.voxelSize = 10 * numPts / 1E7;


  // #ifdef __linux__
  // memAfter = getFreeMem();
  // #endif

  // // assume for scans take up to m GB data
  // // 5 x m is an appropriate voxel size for reduction
  // float redFactor = (memBefore - memAfter) / 1E6;
  // params.voxelSize = redFactor > 0.2 ? redFactor * 5 : -1.0;

  return 1;
}

// estimate reduction factors for indivisual scan
int
#ifndef _MSC_VER
__attribute__((optimize(0)))
#endif
getRedParam(RedParam &rp, Scan* scan) {
  unsigned long memBefore = 0, memAfter = 0, numPts = 0;

  // #ifdef __linux__
  // memBefore = getFreeMem();
  // #endif

  // retrieve all scans data for a rough memory estimation
  DataXYZ xyz = scan->get("xyz");
  numPts= xyz.size();
  rp.voxelSize = 10 * numPts / 1E7;
  // #ifdef __linux__
  // memAfter = getFreeMem();
  // #endif

  // // assume for scans take up to m GB data
  // // 5 x m is an appropriate voxel size for reduction
  // float redFactor = (memBefore - memAfter) / 1E6;
  // rp.voxelSize = redFactor > 0.2 ? redFactor * 5 : -1.0;

  return 1;
}

// unused so far
unsigned long getTotalMem() {
  std::string token;
  std::ifstream file("/proc/meminfo");
  while(file >> token) {
    if(token == "MemTotal:") {
      unsigned long mem;
      if(file >> mem) {
        return mem;
      } else {
        return 0;
      }
    }
    // ignore rest of the line
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  file.close();
  return 0; // nothing found
}

// get free memory size on linux
// FIXME: to be imprved with mac os and windows
unsigned long getFreeMem() {
  std::string token;
  std::ifstream file("/proc/meminfo");
  while(file >> token) {
    if(token == "MemAvailable:") {
      unsigned long mem;
      if(file >> mem) {
        return mem;
      } else {
        return 0;
      }
    }
    // ignore rest of the line
    file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
  }
  file.close();
  return 0; // nothing found
}
