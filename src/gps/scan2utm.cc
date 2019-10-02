/**
  * Exporting points to UTM coordinates
  *
  * Copyright (C) Dorit Borrmann
  *
  * Released under the GPL version 3.
  *
  */

/**
  * @file
  * @author Dorit Borrmann. Julius-Maximilian University Würzburg, Germany.
  */

#include <string>
#include <iostream>
#include <fstream>

#include "gps/gps.h"
#include "scanio/writer.h"

#ifndef _MSC_VER
#include <getopt.h>
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif


#include <boost/program_options.hpp>
namespace po = boost::program_options;

using namespace std;

void usage(char* prog)
{
#ifndef _MSC_VER
  const std::string bold("\033[1m");
  const std::string normal("\033[m");
#else
  const std::string bold("");
  const std::string normal("");
#endif
  std::cout << std::endl
	  << bold << "USAGE " << normal << std::endl
	  << "   " << prog << " [options] directory" << std::endl << std::endl;
  std::cout << bold << "OPTIONS" << normal << std::endl
	  << std::endl;
  exit(1);
}

/*
 * validates input type specification
 */
void validate(boost::any& v, const std::vector<std::string>& values,
              IOType*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  std::string arg = values.at(0);
  try {
    v = formatname_to_io_type(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}

int parse_options(int argc, char **argv, std::string &dir, IOType &iotype, int &start, int &end, bool &use_color, bool &use_reflectance) {

  po::options_description generic("Generic options");
  generic.add_options()
    ("help,h", "output this help message");

  po::options_description input("Input options");

  input.add_options()
    ("start,s", po::value<int>(&start)->default_value(0),
     "start at scan <arg> (i.e., neglects the first <arg> scans) "
     "[ATTENTION: counting naturally starts with 0]")
    ("end,e", po::value<int>(&end)->default_value(-1),
     "end after scan <arg>")
    ("format,f", po::value<IOType>(&iotype)->default_value(UOS, "uos"),
     "using shared library <arg> for input. (chose F from {uos, uos_map, "
     "uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, "
     "riegl_txt, riegl_rgb, riegl_bin, zahn, ply, las})")
    ("color,c", po::bool_switch(&use_color)->default_value(false),
     "end after scan <arg>")
    ("reflectance,R", po::bool_switch(&use_reflectance)->default_value(false),
     "end after scan <arg>");

  po::options_description hidden("Hidden options");
  hidden.add_options()
    ("input-dir", po::value<std::string>(&dir), "input dir");


    // all options
  po::options_description all;
  all.add(generic).add(input).add(hidden);

  // options visible with --help
  po::options_description cmdline_options;
  cmdline_options.add(generic).add(input);

  // positional argument
  po::positional_options_description pd;
  pd.add("input-dir", 1);

  // process options
  po::variables_map vm;
  po::store(po::command_line_parser(argc, argv).
            options(all).positional(pd).run(), vm);

  // display help
  if (vm.count("help")) {
    std::cout << cmdline_options;
    std::cout << std::endl
         << "Example usage:" << std::endl
         << "\t./bin/scan_red -s 0 -e 0 -f uos --reduction OCTREE --voxel 10 --octree 0 dat"
         << std::endl;
    exit(0);
  }
  po::notify(vm);

#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif

  return 0;
}


/**
 * Converts scans in ECEF coordinates to UTM coordinates in meters (East, North, Height).
 * The standard ECEF coordinate system is a right-handed cartesian coordinate
 * system. The origin corresponds to the center of mass of the earth. The z-axis
 * goes through the geodetic north pole. The x-axis goes through the
 * intersection between the equator (0° latitude) and the prime meridian (0°
 * longitude).
 * In the internal left-handed 3DTK coordinate system the y-axis goes through
 * the geodetic north pole and the z-axis intersects (0°, 0°).
 * Output will be: EAST NORTH HEIGHT ...
 */
int main(int argc, char **argv)
{
  if (argc <= 1) {
    usage(argv[0]);
  }

  std::string dir;
  IOType iotype    = UOS;
  int    start = 0,   end = -1;
  bool use_color = false, use_reflectance = false;
  bool high_precision = false;
  bool hexfloat = false;

  parse_options(argc, argv, dir, iotype, start, end, use_color, use_reflectance);

  Scan::openDirectory(false, dir, iotype, start, end);

  if(Scan::allScans.size() == 0) {
    std::cerr << "No scans found. Did you use the correct format?" << std::endl;
    exit(-1);
  }

  unsigned int types = PointType::USE_NONE;
  if(supportsReflectance(iotype)) types |= PointType::USE_REFLECTANCE;
  if(supportsColor(iotype)) types |= PointType::USE_COLOR;

  //No reduction for now
  int red = 0;

  char filename[255];
#ifndef _MSC_VER
    snprintf(filename,255,"%spoints.utm",dir.c_str());
# else
    sprintf_s(filename,255,"%spoints.utm",dir.c_str());
#endif

  FILE *redptsout = fopen(filename, "w");

  for(unsigned int i = 0; i < Scan::allScans.size(); i++) {
    Scan *source = Scan::allScans[i];
    std::string red_string = red > 0 ? " reduced" : "";

    DataXYZ xyz  = source->get("xyz" + red_string);

    ECEFtoUTM(xyz);

    if(use_reflectance) {
      DataReflectance xyz_reflectance =
          (((DataReflectance)source->get("reflectance" + red_string)).size() == 0) ?

          source->create("reflectance" + red_string, sizeof(float)*xyz.size()) :
          source->get("reflectance" + red_string);

      if (!(types & PointType::USE_REFLECTANCE)) {
        for(unsigned int i = 0; i < xyz.size(); i++) xyz_reflectance[i] = 255;
      }
      write_uosr(xyz, xyz_reflectance, redptsout, 1.0, hexfloat, high_precision);

    } else if(use_color) {
      std::string data_string = red > 0 ? "color reduced" : "rgb";
      DataRGB xyz_color =
          (((DataRGB)source->get(data_string)).size() == 0) ?
          source->create(data_string, sizeof(unsigned char)*3*xyz.size()) :
          source->get(data_string);
      if (!(types & PointType::USE_COLOR)) {
          for(unsigned int i = 0; i < xyz.size(); i++) {
            xyz_color[i][0] = 0;
            xyz_color[i][1] = 0;
            xyz_color[i][2] = 0;
        }
      }
      write_uos_rgb(xyz, xyz_color, redptsout, hexfloat, high_precision);

    } else {
    write_uos(xyz, redptsout, hexfloat, high_precision);

    }


  }

  fclose(redptsout);
}
