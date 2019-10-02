#ifndef __PARSE_OPTIONS_H__
#define __PARSE_OPTIONS_H__

#include <iostream>
#include <string>
#include <boost/program_options.hpp>
#include <slam6d/io_types.h>

#include "mesh/calc_normals.h"

// Parse commandline options and assign to parameters
void parse_options(
  int argc, char **argv, int &start, int &end,
  bool &scanserver, int &max_dist, int &min_dist,
  std::string &dir, std::string &odir, IOType &iotype,
  bool &in_color,  bool &reflectance, double &min_refl, double &max_refl,
  bool &no_normal, bool &join, double &red, int &rand, bool &use_pose,
  int &octree, bool &rangeFilterActive, bool &customFilterActive,
  std::string &customFilter, double &scaleFac, bool &autoRed,
  int &k1, int &k2, normal_method &ntype, int &width, int &height,
  bool &inward, int &depth, float &samplesPerNode, float &trimVal
);
// validate function is needed to parse user defined types
// validate normmal_method type (needed boost program_option)
void validate(boost::any& v, const std::vector<std::string>& values, normal_method*, int);
// validate IO types (needed boost program_option)
void validate(boost::any& v, const std::vector<std::string>& values, IOType*, int);
#endif
