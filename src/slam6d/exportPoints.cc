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
#include <map>

#include "slam6d/scan.h"
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
	  << bold << "  -R" << normal << " NR, " << bold << "--random=" << normal << "NR" << endl
	  << "         turns on randomized reduction, using about every <NR>-th point only" << endl
	  << endl
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << endl
	  << "         [ATTENTION: counting naturally starts with 0]" << endl
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
 * @param rand use randomized point reduction?
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
		    int &octree, IOType &type)
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
    { "random",          required_argument,   0,  'R' },
    { "trustpose",       no_argument,         0,  'p' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "f:s:e:r:O:R:p", longopts, NULL)) != -1)
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
	   rand = atoi(optarg);
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

void readFrames(string dir, int start, int end, IOType &type)
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
    int algoTypeInt;

    while (frame_in.good()) {
      try {
        frame_in >> transMat >> algoTypeInt;
      }
      catch (const exception &e) {   
        break;
      }
    }

    if(type == UOS_MAP || type == UOS_MAP_FRAMES || type == RTS_MAP) {
      Scan::allScans[fileCounter - start]->transformAll(transMat);
      if(fileCounter == start+1) {
        Scan::allScans[0]->transformAll(transMat);
      }
    } else {
      Scan::allScans[fileCounter - start - 1]->transformAll(transMat);
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
  int octree       = 0;  // employ randomized octree reduction?
  IOType type    = UOS;

//  parseArgs(argc, argv, dir, red, rand, start, end,
//      maxDist, minDist, eP, octree, type);
//
//  // Get Scans
//  Scan::readScans(type, start, end, dir, maxDist, minDist, true);
//
//  int end_reduction = (int)Scan::allScans.size();
//#ifdef _OPENMP
//#pragma omp parallel for schedule(dynamic)
//#endif
//  for (int iterator = 0; iterator < end_reduction; iterator++) {
//    if (red > 0) {
//      cout << "Reducing Scan No. " << iterator << endl;
//    } else {
//      cout << "Copying Scan No. " << iterator << endl;
//    }
//    // reduction filter for current scan!
//    Scan::allScans[iterator]->calcReducedPoints(red, octree);
//  }
//
//  if(eP) {
//    readFrames(dir, start, end, type);
//  }
//
//  cout << "Export all 3D Points to file \"points.pts\"" << endl;
//  ofstream redptsout("points.pts");
//  for(unsigned int i = 0; i < Scan::allScans.size(); i++) {
//    const vector <Point> *points = Scan::allScans[i]->get_points();
//    for(unsigned int j = 0; j < points->size(); j++) {
//      redptsout << points->at(j) << endl;
//    }
//  }
//  redptsout.close();
//  redptsout.clear();
}
