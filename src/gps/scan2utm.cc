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

#include "slam6d/scan.h"
#include "gps/gps.h"
#include "scanio/writer.h"

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif

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

int parseArgs(int argc, char **argv, std::string &dir, IOType &type, int &start, int &end, bool &use_color, bool &use_reflectance) {
  int  c;
  // from unistd.h:
  extern char *optarg;
  extern int optind;

  /* options descriptor */
  // 0: no arguments, 1: required argument, 2: optional argument
  static struct option longopts[] = {
    { "format",          required_argument,   0,  'f' },  
    { "start",           required_argument,   0,  's' },
    { "end",             required_argument,   0,  'e' },
    { "reflectance",     no_argument,         0,  'R' },
    { "color",           no_argument,         0,  'c' },
    { "reflectivity",    no_argument,         0,  'R' },
    { "help",            no_argument,         0,  'h' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };
  
  int option_index = 0;
  while ((c = getopt_long(argc, argv, "f:s:e:cRh", longopts, &option_index)) != -1)
    switch (c)
     {
       case 'R':
	    use_reflectance = true; 
       break;
	  case 'c':
	    use_color = true;
	    break;
     case 's':
       start = atoi(optarg);
       if (start < 0) { std::cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
       break;
     case 'e':
       end = atoi(optarg);
       if (end < 0)     { std::cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
       if (end < start) { std::cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
       break;
     case 'f':
    try {
      type = formatname_to_io_type(optarg);
    } catch (...) { // runtime_error
      std::cerr << "Format " << optarg << " unknown." << std::endl;
      abort();
    }
    break;
      case 'h':
      case '?':
       usage(argv[0]);
       return 1;
      default:
       abort ();
      }

  if (optind != argc-1) {
    std::cerr << "\n*** Directory missing ***" << std::endl;
    usage(argv[0]);
  }
  dir = argv[optind];

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

  parseArgs(argc, argv, dir, iotype, start, end, use_color, use_reflectance);
  
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
    sprintf(filename,255,"%spoints.utm",dir.c_str());
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
      write_uosr(xyz, xyz_reflectance, redptsout, hexfloat, high_precision);
      
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
