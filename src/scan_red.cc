/**
 * @file
 * @brief Main program for reducing 3D scans.
 * 
 * Program to reduce scans for use with slam6d 
 * Usage: bin/scan_red -r <NR> 'dir',
 * Use -r for octree based reduction  (voxel size=<NR>)
 * and 'dir' the directory of a set of scans
 * Reduced scans will be written to 'dir/reduced'
 *
 * @author Dorit Borrmann. Smart Systems Group, Jacobs University Bremen gGmbH, Germany. 
 */
#ifdef _MSC_VER
#ifdef OPENMP
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
#include <errno.h>

#include "scan.h"

#include "scan_io_riegl_txt.h"
#include "globals.icc"

#ifdef _OPENMP
#include <omp.h>
#endif


#ifndef _MSC_VER
#include <getopt.h>
#else
#include "..\Visual_Studio_Projects\6DSLAM\6D_SLAM\XGetopt.h"
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
	  << "   " << prog << " [options] -r <NR> directory" << endl << endl;
  cout << bold << "OPTIONS" << normal << endl

	  << bold << "  -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << endl
	  << "         end after scan NR" << endl
	  << endl
	  << bold << "  -f" << normal << " F, " << bold << "--format=" << normal << "F" << endl
	  << "         using shared library F for input" << endl
	  << "         (chose F from {uos, uos_map, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_txt, riegl_bin, zahn, ply})" << endl
	  << endl
	  << bold << "  -m" << normal << " NR, " << bold << "--max=" << normal << "NR" << endl
	  << "         neglegt all data points with a distance larger than NR 'units'" << endl
	  << endl
	  << bold << "  -M" << normal << " NR, " << bold << "--min=" << normal << "NR" << endl
	  << "         neglegt all data points with a distance smaller than NR 'units'" << endl
	  << endl
	  << bold << "  -r" << normal << " NR, " << bold << "--reduce=" << normal << "NR" << endl
	  << "         turns on octree based point reduction (voxel size=<NR>)" << endl
	  << endl
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << endl
	  << "         [ATTENTION: counting naturally starts with 0]" << endl
	  << endl
    	  << endl << endl;
  
  cout << bold << "EXAMPLES " << normal << endl
	  << "   " << prog << " -m 500 -r 5 dat" << endl
	  << "   " << prog << " --max=5000 -r 10.2 dat" << endl
	  << "   " << prog << " -s 2 -e 10 -r dat" << endl << endl;
  exit(1);
}

/** A function that parses the command-line arguments and sets the respective flags.
 * @param argc the number of arguments
 * @param argv the arguments
 * @param dir the directory
 * @param red using point reduction?
 * @param rand use randomized point reduction?
 * @param start starting at scan number 'start'
 * @param end stopping at scan number 'end'
 * @param maxDist - maximal distance of points being loaded
 * @param minDist - minimal distance of points being loaded
 * @param quiet switches on/off the quiet mode
 * @param veryQuiet switches on/off the 'very quiet' mode
 * @return 0, if the parsing was successful. 1 otherwise
 */
int parseArgs(int argc, char **argv, string &dir, double &red, 
		    int &start, int &end, int &maxDist, int &minDist, int &octree, 
		    reader_type &type)
{
  bool reduced = false;
  int  c;
  // from unistd.h:
  extern char *optarg;
  extern int optind;

  /* options descriptor */
  // 0: no arguments, 1: required argument, 2: optional argument
  static struct option longopts[] = {
    { "format",          required_argument,   0,  'f' },  
    { "max",             required_argument,   0,  'm' },
    { "min",             required_argument,   0,  'M' },
    { "start",           required_argument,   0,  's' },
    { "end",             required_argument,   0,  'e' },
    { "reduce",          required_argument,   0,  'r' },
    { "octree",          optional_argument,   0,  'O' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "f:r:s:e:m:M:", longopts, NULL)) != -1)
    switch (c)
	 {
	 case 'r':
	   red = atof(optarg);
     reduced = true;
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
	 case 'f': 
	   if (strcasecmp(optarg, "uos") == 0) type = UOS;
	   else if (strcasecmp(optarg, "uos_map") == 0) type = UOS_MAP;
	   else if (strcasecmp(optarg, "uos_frames") == 0) type = UOS_FRAMES;
	   else if (strcasecmp(optarg, "uos_map_frames") == 0) type = UOS_MAP_FRAMES;
	   else if (strcasecmp(optarg, "old") == 0) type = OLD;
	   else if (strcasecmp(optarg, "rts") == 0) type = RTS;
	   else if (strcasecmp(optarg, "rts_map") == 0) type = RTS_MAP;
	   else if (strcasecmp(optarg, "ifp") == 0) type = IFP;
	   else if (strcasecmp(optarg, "riegl_txt") == 0) type = RIEGL_TXT;
	   else if (strcasecmp(optarg, "riegl_bin") == 0) type = RIEGL_BIN;
	   else if (strcasecmp(optarg, "zahn") == 0) type = ZAHN;
	   else if (strcasecmp(optarg, "ply") == 0) type = PLY;
	   else if (strcasecmp(optarg, "wrl") == 0) type = WRL;
	   else if (strcasecmp(optarg, "xyz") == 0) type = XYZ;
	   else if (strcasecmp(optarg, "zuf") == 0) type = ZUF;
	   else if (strcasecmp(optarg, "asc") == 0) type = ASC;
	   else if (strcasecmp(optarg, "iais") == 0) type = IAIS;
	   else if (strcasecmp(optarg, "front") == 0) type = FRONT;
	   else if (strcasecmp(optarg, "x3d") == 0) type = X3D;
	   else {
		 abort ();
	   }
	   break;
	 case 'm':
	   maxDist = atoi(optarg);
	   break;
	 case 'O':
     if (optarg) {
       octree = atoi(optarg);
     } else {
       octree = 1;
     }
	   break;
	 case 'M':
	   minDist = atoi(optarg);
	   break;
   case '?':
	   usage(argv[0]);
	   return 1;
      default:
	   abort ();
      }

  if(!reduced) {
    cerr << "\n*** Reduction method missind ***" << endl;
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

  return 0;
}


/**
 * Main program for reducing scans.
 * Usage: bin/scan_red -r <NR> 'dir',
 * Use -r for octree based reduction  (voxel size=<NR>)
 * and 'dir' the directory of a set of scans
 * Reduced scans will be written to 'dir/reduced'
 * 
 */
int main(int argc, char **argv)
{

  cout << "(c) University of Osnabrueck, 2006 - 2010" << endl << endl
	  << "Restricted Usage" << endl
	  << "Don't use without permission" << endl;
  
  if (argc <= 1) {
    usage(argv[0]);
  }

  // parsing the command line parameters
  // init, default values if not specified
  string dir;
  double red   = -1.0;
  int    start = 0,   end = -1;
  int    maxDist    = -1;
  int    minDist    = -1;
  int    octree     = 0;
  reader_type type    = RIEGL_TXT;
  
  parseArgs(argc, argv, dir, red, start, end, maxDist, minDist, octree, type);

  //@@@ to do :-)

  // Get Scans
  Scan::dir = dir;
  int fileNr = start;
  string reddir = dir + "reduced"; 
 
  int success = mkdir(reddir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
  if(success == 0) { 
    cout << "Writing scans to " << reddir << endl;
  } else if(errno == EEXIST) {
    cout << "Directory " << reddir << " exists already.  CONTINUE" << endl; 
  } else {
    cerr << "Creating directory " << reddir << " failed" << endl;
    exit(1);
  }

  while (fileNr <= end) {
    Scan::readScans(type, fileNr, fileNr, dir, maxDist, minDist, 0);
      // reduction filter for current scan!
    Scan::allScans[0]->calcReducedPoints(red, octree);
    
	  cout << "Writing Scan No. " << fileNr ;
    cout << " with " << Scan::allScans[0]->get_points_red_size() << " points" << endl; 
    string scanFileName;
    string poseFileName;
  
    poseFileName = dir  + "reduced/scan" + to_string(fileNr,3) + ".pose";
    scanFileName = dir + "reduced/scan" + to_string(fileNr,3) + ".3d";
   
     
    ofstream redptsout(scanFileName.c_str());
	  for (int j = 0; j < Scan::allScans[0]->get_points_red_size(); j++) {
	       redptsout << Scan::allScans[0]->get_points_red()[j][0] << " "
			   << Scan::allScans[0]->get_points_red()[j][1] << " "
			   << Scan::allScans[0]->get_points_red()[j][2] << endl;
	  }
    redptsout.close();
    redptsout.clear();
      

    ofstream posout(poseFileName.c_str());
    const double* rPos = Scan::allScans[0]->get_rPos();
    const double* rPosTheta = Scan::allScans[0]->get_rPosTheta();
    
    posout << rPos[0] << " " 
           << rPos[1] << " " 
           << rPos[2] << endl   
           << deg(rPosTheta[0]) << " " 
           << deg(rPosTheta[1]) << " " 
           << deg(rPosTheta[2]) << endl;  
    
    posout.close();
    posout.clear();

    delete Scan::allScans[0];
    Scan::allScans.clear();
    
    fileNr++;
  }   

  cout << endl << endl;
  cout << "Normal program end." << endl << endl;
}
