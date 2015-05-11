/*
 * exportPoints implementation
 *
 * Copyright (C) Jochen Sprickerhof
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file
 * @author Jochen Sprickerhof. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ifstream;
#include <stdexcept>
using std::exception;

#include <vector>
using std::vector;
#include <map>

#include "slam6d/point.h"
#include "slam6d/scan.h"
#include "scanio/writer.h"
#include "slam6d/globals.icc"

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

/**
 * Explains the usage of this program's command line parameters
 */
void usage(char* prog)
{
#ifndef _MSC_VER
  const string bold("\033[1m");
  const string normal("\033[m");
#else
  const string bold("");
  const string normal("");
#endif
  cout << endl
      << bold << "USAGE " << normal << endl
      << "   " << prog << " [options] directory" << endl << endl;
  cout << bold << "OPTIONS" << normal << endl

      << endl
      << bold << "  -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << endl
      << "         end after scan NR" << endl
      << endl
      << bold << "  -f" << normal << " F, " << bold << "--format=" << normal << "F" << endl
      << "         using shared library F for input" << endl
      << "         (chose F from {uos, uos_map, uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply})" << endl
      << endl
      << bold << "  -m" << normal << " NR, " << bold << "--max=" << normal << "NR" << endl
      << "         neglegt all data points with a distance larger than NR 'units'" << endl
      << endl
      << bold << "  -M" << normal << " NR, " << bold << "--min=" << normal << "NR" << endl
      << "         neglegt all data points with a distance smaller than NR 'units'" << endl
      << endl
      << bold << "  -u" << normal << " STR, " << bold << "--customFilter=" << normal << "STR" << endl
      << "         apply custom filter, filter mode and data are specified as semicolon-seperated string:" << endl
      << "         STR: '{filterMode};{nrOfParams}[;param1][;param2][...]'" << endl
      << "         see filter implementation in pointfilter.cc for more detail." << endl
      << endl
      << bold << "  -O" << normal << " NR (optional), " << bold << "--octree=" << normal << "NR (optional)" << endl
      << "         use randomized octree based point reduction (pts per voxel=<NR>)" << endl
      << "         requires -r or --reduce" << endl
      << endl
      << bold << "  -p, --trustpose" << normal << endl
      << "         Trust the pose file, do not extrapolate the last transformation." << endl
      << "         (just for testing purposes, or gps input.)" << endl
      << endl
      << endl
      << bold << "  -r" << normal << " NR, " << bold << "--reduce=" << normal << "NR" << endl
      << "         turns on octree based point reduction (voxel size=<NR>)" << endl
      << endl
      /*
    << bold << "  -R" << normal << " NR, " << bold << "--random=" << normal << "NR" << endl
      << "         turns on randomized reduction, using about every <NR>-th point only" << endl
      << endl
      */
      << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
      << "         start at scan NR (i.e., neglects the first NR scans)" << endl
      << "         [ATTENTION: counting naturally starts with 0]" << endl
      << bold << "  -x, --xyz" << endl
      << "         export in xyz format (right handed coordinate system in m)" << endl
      << bold << "  -y" << normal << " NR, " << bold << "--scale=" << normal << "NR" << endl
      << "         scale factor for export in XYZ format (default value is 0.01, so output will be in [m])" << endl
   
      << endl
          << endl << endl;
  
  cout << bold << "EXAMPLES " << normal << endl
      << "   " << prog << " -s 2 -e 3 dat" << endl << endl;
  exit(1);
}


/** A function that parses the command-line arguments and sets the respective flags.
 * @param argc the number of arguments
 * @param argv the arguments
 * @param dir the directory
 * @param red using point reduction?
 * @param mdm maximal distance match
 * @param mdml maximal distance match for SLAM
 * @param mni maximal number of iterations
 * @param start starting at scan number 'start'
 * @param end stopping at scan number 'end'
 * @param maxDist - maximal distance of points being loaded
 * @param minDist - minimal distance of points being loaded
 * @param quiet switches on/off the quiet mode
 * @param veryQuiet switches on/off the 'very quiet' mode
 * @param extrapolate_pose - i.e., extrapolating the odometry by the last transformation
 *        (vs. taking the pose file as <b>exact</b>)
 * @param meta match against all scans (= meta scan), or against the last scan only???
 * @param anim selects the rotation representation for the matching algorithm
 * @param mni_lum sets the maximal number of iterations for SLAM
 * @param net specifies the file that includes the net structure for SLAM
 * @param cldist specifies the maximal distance for closed loops
 * @param epsilonICP stop ICP iteration if difference is smaller than this value
 * @param epsilonSLAM stop SLAM iteration if average difference is smaller than this value
 * @param algo specfies the used algorithm for rotation computation
 * @param lum6DAlgo specifies the used algorithm for global SLAM correction
 * @param loopsize defines the minimal loop size
 * @return 0, if the parsing was successful. 1 otherwise
 */
int parseArgs(int argc, char **argv, string &dir, double &red, int &rand,
            int &start, int &end, int &maxDist, int &minDist, bool &extrapolate_pose,
            bool &use_xyz, bool &use_reflectance, bool &use_color, int &octree, IOType &type, string& customFilter, double &scaleFac)
{
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
    { "reduce",          required_argument,   0,  'r' },
    { "octree",          optional_argument,   0,  'O' },
    { "trustpose",       no_argument,         0,  'p' },
    { "reflectance",     no_argument,         0,  'R' },
    { "reflectivity",    no_argument,         0,  'R' },
    { "color",           no_argument,         0,  'c' },
    { "xyz",             no_argument,         0,  'x' },
    { "scale",           required_argument,   0,  'y' },
    { "customFilter",    required_argument,   0,  'u' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "f:s:e:r:O:Rm:y:M:u:pxc", longopts, NULL)) != -1)
    switch (c)
     {
     case 'r':
       red = atof(optarg);
       break;
     case 'O':
       if (optarg) {
         octree = atoi(optarg);
       } else {
         octree = 1;
       }
       break;
     case 'R':
     use_reflectance = true; 
       break;
   case 'c':
     use_color = true;
     break;
     case 's':
       start = atoi(optarg);
       if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
       break;
     case 'e':
       end = atoi(optarg);
       if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
       if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
       break;
     case 'm':
       maxDist = atoi(optarg);
       break;
     case 'M':
       minDist = atoi(optarg);
       break;
     case 'p':
       extrapolate_pose = false;
       break;
     case 'x':
       use_xyz = true;
       break;
     case 'y':
       scaleFac = atof(optarg);
       break;
     case 'u':
       customFilter = optarg;
       break;
     case 'f':
    try {
      type = formatname_to_io_type(optarg);
    } catch (...) { // runtime_error
      cerr << "Format " << optarg << " unknown." << endl;
      abort();
    }
    break;
      case '?':
       usage(argv[0]);
       return 1;
      default:
       abort ();
      }

  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***" << endl;
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

void readFrames(string dir, int start, int end, bool extrapolate_pose=true)
{
  ifstream frame_in;
  int  fileCounter = start;
  string frameFileName;
  for (;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read
    frameFileName = dir + "scan" + to_string(fileCounter++,3) + ".frames";

    frame_in.open(frameFileName.c_str());

    // read 3D scan
    if (!frame_in.good()) break; // no more files in the directory

    cout << "Reading Frames for 3D Scan " << frameFileName << "..." << endl;

 //   vector <double*> Matrices;
//    vector <Scan::AlgoType> algoTypes;
    double transMat[16];
    double transMatOrig[16];
    int algoTypeInt;

    frame_in >> transMatOrig >> algoTypeInt;
    for (size_t i = 0; i < 16; i++) {
        transMat[i] = transMatOrig[i];
    }

    while (frame_in.good()) {
      try {
        frame_in >> transMat >> algoTypeInt;
      }
      catch (const exception &e) {   
        break;
      }
    }

    // calculate RELATIVE transformation
    double tinv[16];
    M4inv(transMatOrig, tinv);

    double tfin[16];
    MMult(transMat, tinv, tfin);

    if(!extrapolate_pose) {
      Scan::allScans[fileCounter - start - 1]->transformAll(transMatOrig);
      //Scan::allScans[fileCounter - start - 1]->transformMatrix(tinv);
    } else {
      Scan::allScans[fileCounter - start - 1]->transformAll(transMat);
      // save final pose in scan
      Scan::allScans[fileCounter - start - 1]->transformMatrix(tfin);
    }
    frame_in.close();
    frame_in.clear();
  }
}

/**
 * program for point export
 * Usage: bin/exportPoints 'dir',
 * with 'dir' the directory of a set of scans
 * ...
 */
int main(int argc, char **argv)
{
  if (argc <= 1) {
    usage(argv[0]);
  }

  // parsing the command line parameters
  // init, default values if not specified
  string dir;
  double red   = -1.0;
  int    rand  = -1;
  int    start = 0,   end = -1;
  int    maxDist    = -1;
  int    minDist    = -1;
  bool   eP         = true;  // should we extrapolate the pose??
  bool   use_xyz = false;
  bool   use_color = false;
  bool   use_reflectance = false;
  int octree       = 0;  // employ randomized octree reduction?
  IOType iotype    = UOS;
  bool rangeFilterActive = false;
  bool customFilterActive = false;
  string customFilter;
  double scaleFac = 0.01;

  parseArgs(argc, argv, dir, red, rand, start, end,
      maxDist, minDist, eP, use_xyz, use_reflectance, use_color, octree, iotype, customFilter, scaleFac);


  rangeFilterActive = minDist > 0 || maxDist > 0;

  // custom filter set? quick check, needs to contain at least one ';' 
  // (proper checking will be done case specific in pointfilter.cc)
  size_t pos = customFilter.find_first_of(";");
  if (pos != std::string::npos){
      customFilterActive = true;

      // check if customFilter is specified in file
      if (customFilter.find("FILE;") == 0){
          string selection_file_name = customFilter.substr(5, customFilter.length());
          ifstream selectionfile;
          // open the input file
          selectionfile.open(selection_file_name, ios::in);

          if (!selectionfile.good()){
              cerr << "Error loading custom filter file " << selection_file_name << "!" << endl;
              cerr << "Data will NOT be filtered!" << endl;
              customFilterActive = false;
          }
          else {
              string line;
              string custFilt;
              while (std::getline(selectionfile, line)){
                  if (line.find("#") == 0) continue;
                  custFilt = custFilt.append(line);
                  custFilt = custFilt.append("/");
              }
              if (custFilt.length() > 0) {
                  // last '/'
                  customFilter = custFilt.substr(0, custFilt.length() - 1);
              }
          }
          selectionfile.close();
      }
  }
  else {
      // give a warning if custom filter has been inproperly specified
      if (customFilter.length() > 0){
          cerr << "Custom filter: specifying string has not been set properly, data will NOT be filtered." << endl;
      }
  }

  // Get Scans
  Scan::openDirectory(false, dir, iotype, start, end);
  if(Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);
  }
  
  unsigned int types = PointType::USE_NONE;
  //TODO check if all file formats are included
  if(use_reflectance) {
    switch (iotype) {
      case UOSR:
      case UOS_RRGBT:
      case UOS_RRGB:
      case RIEGL_TXT:
      case RIEGL_PROJECT:
      case RXP:
      case TXYZR:
      case XYZR:
      case XYZ_RRGB:
      case FARO_XYZ_RGBR:
      case LEICA_XYZR:
        types |= PointType::USE_REFLECTANCE;
        break;
      default:
        break;
    }
  }
  
  if(use_color) {
    switch (iotype) {
      case UOS_RGB:
      case UOS_RRGBT:
      case RIEGL_RGB:
      case XYZ_RGB:
      case KS_RGB:
        types |= PointType::USE_COLOR;
        break;
      default:
         break;
    }
  }

  // if specified, filter scans
  for (size_t i = 0; i < Scan::allScans.size(); i++)  {
     if(rangeFilterActive) Scan::allScans[i]->setRangeFilter(maxDist, minDist);
     if(customFilterActive) Scan::allScans[i]->setCustomFilter(customFilter);
  }
  
//
  int end_reduction = (int)Scan::allScans.size();
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
  for (int iterator = 0; iterator < end_reduction; iterator++) {
    if (red > 0) {
      PointType pointtype(types);
      cout << "Reducing Scan No. " << iterator << endl;
      Scan::allScans[iterator]->setReductionParameter(red, octree, pointtype);
      Scan::allScans[iterator]->calcReducedPoints();
    } else {
      cout << "Copying Scan No. " << iterator << endl;
    }
    // reduction filter for current scan!
  }

  if(eP) {
  }
  readFrames(dir, start, end, eP);
  
 cout << "Export all 3D Points to file \"points.pts\"" << endl;
 cout << "Export all 6DoF poses to file \"positions.txt\"" << endl;
 cout << "Export all 6DoF matrices to file \"poses.txt\"" << endl;
 ofstream redptsout("points.pts");
 ofstream posesout("positions.txt");
 ofstream matricesout("poses.txt");
  
  for(unsigned int i = 0; i < Scan::allScans.size(); i++) {
    Scan *source = Scan::allScans[i];
    string red_string = red > 0 ? " reduced" : "";
      
    DataXYZ xyz  = source->get("xyz" + red_string);
    
    if(use_reflectance) {
      DataReflectance xyz_reflectance = 
          (((DataReflectance)source->get("reflectance" + red_string)).size() == 0) ? 
          source->create("reflectance" + red_string, sizeof(float)*xyz.size()) : 
          source->get("reflectance" + red_string);  
      if (!(types & PointType::USE_REFLECTANCE)) {
        for(unsigned int i = 0; i < xyz.size(); i++) xyz_reflectance[i] = 255;
      }
      if(use_xyz) {
        write_xyzr(xyz, xyz_reflectance, redptsout, scaleFac);
      } else {
        write_uosr(xyz, xyz_reflectance, redptsout);
      }
      
    } else if(use_color) {
      string data_string = red > 0 ? "color reduced" : "rgb";
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
      if(use_xyz) {
        write_xyz_rgb(xyz, xyz_color, redptsout, scaleFac);
      } else {
        write_uos_rgb(xyz, xyz_color, redptsout);
      }

    } else {
      if(use_xyz) {
        write_xyz(xyz, redptsout, scaleFac);
      } else {
        write_uos(xyz, redptsout);
      }
    
    }
    if(use_xyz) {
      writeTrajectoryXYZ(posesout, source->get_transMat(), false, scaleFac);
      writeTrajectoryXYZ(matricesout, source->get_transMat(), true, scaleFac);
    } else {
      writeTrajectoryUOS(posesout, source->get_transMat(), false);
      writeTrajectoryUOS(matricesout, source->get_transMat(), true);
    }

  }
   
  redptsout.close();
  redptsout.clear();
  posesout.close();
  posesout.clear();
  matricesout.close();
  matricesout.clear();
}
