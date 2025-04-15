#ifndef __SRR_PROGRAM_OPTIONS_H__
#define __SRR_PROGRAM_OPTIONS_H__

#include "slam6d/io_types.h"

struct SRRSettings {

  std::string dir;

  // general settings
  int start;
  int end;
  IOType format;

  double filterMinDist;
  double filterMaxDist;
  std::string customFilter;
  bool continue_processing;
  int anim;

  // reduction and searchtree parameters
  double voxelsize;
  int octree;

  // preregistration settings
  double mdm;
  bool prereg;
  int preRegInterval;
  int mni;
  double epsICP;
  int preRegIter;
  bool meta;

  // continuous time slam settings
  int mnj;
  int slamiterations;
  double mdml;
  int size;
  int interval;
  double odomWeightFactor;
  int clPairs;

  // segment options
  std::string segment_filename;
  double gapOdomWeightFactor;
  double overlap;

};

void parseArgs(int argc, char **argv, SRRSettings &settings);

#endif
