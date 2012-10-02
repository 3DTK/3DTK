/*
 * scan_diff implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file
 * @brief Main program calculating difference of 3D scans.
 * 
 * Program for calculating the difference between scans after matching with slam6d 
 * Usage: bin/scan_diff -s <START> -e <END> -d <NR> 'dir',
 * Use -s for the first scan, -e for the second scan
 *  'dir' the directory of a set of scans
 * The result is a scan in the UOS format (.3d) that contains all points from
 * the first scan that do NOT have a corresponding point in the second scan 
 * within a distance of less than NR units.
 * Difference scans will be written to 'dir/diff'
 * ATTENTION: All scans between START and END will be loaded!
 * @author Dorit Borrmann. Automation Group, Jacobs University Bremen gGmbH, Germany. 
 */
#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#define WANT_STREAM ///< define the WANT stream :)
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <fstream>
using std::ofstream;
using std::ifstream;
#include <errno.h>
#include <vector>

#include "slam6d/globals.icc"
#include "slam6d/io_utils.h"
#include "slam6d/scan.h"

#include "scanserver/clientInterface.h"

#ifdef _OPENMP
#include <omp.h>
#endif

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#ifdef _MSC_VER
  #define strcasecmp _stricmp
  #define strncasecmp _strnicmp
  #include <windows.h>
  #include <direct.h>
#else
  #include <sys/stat.h>
  #include <sys/types.h>
  #include <strings.h>
  #include <dlfcn.h>
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
	  << "   " << prog << "-s <START> -e <END> [options] directory" << endl << endl;
  cout << bold << "OPTIONS" << normal << endl
	  << endl
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << endl
	  << "         [ATTENTION: counting naturally starts with 0]" << endl
	  << endl
	  << bold << "  -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << endl
	  << "         end after scan NR" << endl
	  << endl
	  << bold << "  -f" << normal << " F, " << bold << "--format=" << normal << "F" << endl
	  << "         using shared library F for input" << endl
	  << "         (chose F from {uos, uos_map, uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, rxp,zahn, ply})" << endl
	  << endl
	  << bold << "  -m" << normal << " NR, " << bold << "--max=" << normal << "NR" << endl
	  << "         neglegt all data points with a distance larger than NR 'units'" << endl
	  << endl
	  << bold << "  -M" << normal << " NR, " << bold << "--min=" << normal << "NR" << endl
	  << "         neglegt all data points with a distance smaller than NR 'units'" << endl
	  << endl
	  << bold << "  -d" << normal << " NR, " << bold << "--dist=" << normal << "NR" << endl
	  << "         write all points that have no corresponding point closer than NR 'units'" << endl
	  << bold << "  -S, --scanserver" << normal << endl
	  << "         Use the scanserver as an input method and handling of scan data" << endl
    << endl << endl;
  
  cout << bold << "EXAMPLES " << normal << endl
	  << "   " << prog << " -m 500 -d 5 dat" << endl
	  << "   " << prog << " --max=5000 -d 10.2 dat" << endl
	  << endl;
  exit(1);
}

/** A function that parses the command-line arguments and sets the respective flags.
 * @param argc the number of arguments
 * @param argv the arguments
 * @param dir the directory
 * @param start first scan number 'start'
 * @param end last scan number 'end'
 * @param maxDist - maximal distance of points being loaded
 * @param minDist - minimal distance of points being loaded
 * @param dist the maximal distance for a point pair
 * @param type the scan format
 * @param desc true if start is greater than end
 * @return 0, if the parsing was successful. 1 otherwise
 */
int parseArgs(int argc, char **argv, string &dir, 
		    int &start, int &end, int &maxDist, int &minDist, double &dist, 
		    IOType &type, bool &desc, bool scanserver)
{
  int  c;
  // from unistd.h:
  extern char *optarg;
  extern int optind;

  WriteOnce<IOType> w_type(type);
  WriteOnce<int> w_start(start), w_end(end);

  /* options descriptor */
  // 0: no arguments, 1: required argument, 2: optional argument
  static struct option longopts[] = {
    { "format",          required_argument,   0,  'f' },  
    { "max",             required_argument,   0,  'm' },
    { "min",             required_argument,   0,  'M' },
    { "start",           required_argument,   0,  's' },
    { "end",             required_argument,   0,  'e' },
    { "dist",            required_argument,   0,  'd' },
    { "scanserver",      no_argument,         0,  'S' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "f:d:s:e:m:M:", longopts, NULL)) != -1)
    switch (c)
	 {
	 case 'd':
	   dist = atof(optarg);
	   break;
	 case 's':
	   w_start = atoi(optarg);
	   if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
	   break;
	 case 'e':
	   w_end = atoi(optarg);
	   if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
	   break;
	 case 'f': 
    try {
      type = formatname_to_io_type(optarg);
    } catch (...) { // runtime_error
      cerr << "Format " << optarg << " unknown." << endl;
      abort();
    }
    break;
	 case 'm':
	   maxDist = atoi(optarg);
	   break;
	 case 'M':
	   minDist = atoi(optarg);
	   break;
	 case '?':
	   usage(argv[0]);
	   return 1;
	 case 'S':
        scanserver = true;
        break;
      default:
	   abort ();
      }

  if(start < 0 || end < 0 || start == end) {
    cerr << "\n*** You need two different scans for difference calculations ***" << endl;
    usage(argv[0]);
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
  if(start > end) {
    double tmp = start;
    start = end; 
    end = tmp;
    desc = true;
  } 

  parseFormatFile(dir, w_type, w_start, w_end);

  return 0;
}


/**
 * Main program for calculating the difference of two scans.
 * Usage: bin/scan_diff -d <NR> -s <NR> -e <NR> 'dir',
 * Use -s and -e for the two scans, 
 * -d 
 * and 'dir' the directory of a set of scans
 * Difference scans will be written to 'dir/diff/scan[00]s.3d'
 * 
 */
int main(int argc, char **argv)
{

  cout << "(c) Jacobs University Bremen, gGmbH, 2012" << endl << endl;
  
  if (argc <= 1) {
    usage(argv[0]);
  }

  // parsing the command line parameters
  // init, default values if not specified
  string dir;
  double    dist   = 0;
  int    start = -1,   end = -1;
  int    maxDist    = -1;
  int    minDist    = -1;
  IOType type    = RIEGL_TXT;
  bool desc = false;  
  bool scanserver = false;

  parseArgs(argc, argv, dir, start, end, maxDist, minDist, dist, type, desc, scanserver);

  if (scanserver) {
    try {
	 ClientInterface::create();
    } catch(std::runtime_error& e) {
	 cerr << "ClientInterface could not be created: " << e.what() << endl;
	 cerr << "Start the scanserver first." << endl;
	 exit(-1);
    }
  }

  string diffdir = dir + "diff"; 
 
#ifdef _MSC_VER
  int success = mkdir(diffdir.c_str());
#else
  int success = mkdir(diffdir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif
  if(success == 0) { 
    cout << "Writing scans to " << diffdir << endl;
  } else if(errno == EEXIST) {
    cout << "Directory " << diffdir << " exists already.  CONTINUE" << endl; 
  } else {
    cerr << "Creating directory " << diffdir << " failed" << endl;
    exit(1);
  }
  string scanFileName;
  string framesFileName;
  ifstream frames_in;
  double inMatrix0[17];
  double inMatrix1[17];
  
  cout << "Reading Scan No. " << start << endl;
  framesFileName = dir  + "scan" + to_string(start,3) + ".frames";
  frames_in.open(framesFileName.c_str());
  
  if(!frames_in.good()) {
    cerr << "Couldn't read frames " << end << endl;
    exit(1);
  }
  while(frames_in.good()) {
    for (unsigned int i = 0; i < 17; frames_in >> inMatrix0[i++]);
  } 
  frames_in.close();
  frames_in.clear();
  for(int i = 0; i < 16; i++) {
    cout << inMatrix0[i] << " ";
  }
  cout << endl; 
  
  cout << "Reading Scan No. " << end << endl;
  framesFileName = dir  + "scan" + to_string(end,3) + ".frames";
  frames_in.open(framesFileName.c_str());
  
  if(!frames_in.good()) {
    cerr << "Couldn't read frames " << end << endl;
    exit(1);
  }
  while(frames_in.good()) {
    for (unsigned int i = 0; i < 17; frames_in >> inMatrix1[i++]);
  }
  frames_in.close();
  frames_in.clear();
  for(int i = 0; i < 16; i++) {
    cout << inMatrix1[i] << " ";
  }
  cout << endl;

  Scan::openDirectory(scanserver, dir, type, start, end);

  if(Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);
  }  

  Scan* scan_first = *(Scan::allScans.begin());
  Scan* scan_second = *(Scan::allScans.end()-1);

  scan_first->transform(inMatrix0, Scan::INVALID);
  scan_second->transform(inMatrix1, Scan::INVALID);

  DataXYZ xyz_first(scan_first->get("xyz"));
  DataXYZ xyz_second(scan_second->get("xyz"));
  
  cout << "Scan " << scan_first->getIdentifier() << " with " << xyz_first.size() << " points" << endl;
  cout << "Scan " << scan_second->getIdentifier() << " with " << xyz_second.size() << " points" << endl;

  int thread_num = 0;
  std::vector<double*> diff;
  double transMat[16];
  
  if(desc) {
    Scan::getNoPairsSimple(diff, scan_second, scan_first, thread_num, dist);
    M4inv(inMatrix1, transMat);
    scanFileName = dir + "diff/scan" + to_string(end,3) + ".3d";
  } else {
    Scan::getNoPairsSimple(diff, scan_first, scan_second, thread_num, dist);
    M4inv(inMatrix0, transMat);
    scanFileName = dir + "diff/scan" + to_string(start,3) + ".3d";
  }

  cout << endl; 
  for(int i = 0; i < 16; i++) {
    cout << transMat[i] << " ";
  }
  cout << endl; 
  for(int i = 0; i < 16; i++) {
    cout << inMatrix0[i] << " ";
  }
  cout << endl; 
 
  ofstream ptsout(scanFileName.c_str());

  for(unsigned int i = 0; i < diff.size(); i++) {
    Point p(diff[i]);
    p.transform(transMat); 
    ptsout << p.x << " " << p.y << " " << p.z << " " << endl;    
  }
 
  ptsout.close();
  ptsout.clear();

  if (scanserver) {
    scan_first->clear("xyz");
    scan_second->clear("xyz");
  }

  cout << endl << endl;
  cout << "Normal program end." << endl << endl;

 if (scanserver) {
  Scan::closeDirectory();
 }

  return 0;
}
