#include <fstream>
#include <iostream>
#include <map>
using std::map;
using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;

#include "slam6d/globals.icc"
#include <string.h>

#ifndef _MSC_VER
#include <unistd.h>
#else
#include "XGetopt.h"
#endif

#if WIN32
#define snprintf sprintf_s
#endif 
#include "b3dpsreader.h"

int main(int argc, char **argv)
{
  std::string bpsfn = argv[1];
  std::string txtfn = argv[2];

  std::map<uint64_t, double*> poses;
  uint64_t minpose, maxpose;
  minpose = UINT64_MAX;
  maxpose = 0;
  
  B3DPSReader::readTrajectory(bpsfn, poses, minpose, maxpose);

  FILE *file = fopen(txtfn.c_str(), "w");
//  for (auto pose : poses) {
  for(uint64_t i = minpose; i < maxpose; i++) {
    auto pose = poses[i];
    fprintf(file, "%lf %lf %lf %lf\n", pose[0], pose[1], pose[2], pose[3]);
  }
  fclose(file);
}

