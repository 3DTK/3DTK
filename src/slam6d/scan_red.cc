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
 * @author Dorit Borrmann. Automation Group, Jacobs University Bremen gGmbH, Germany. 
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

#include "slam6d/scan.h"

#include "slam6d/scan_io.h"
#include "slam6d/globals.icc"

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
	  << "   " << prog << " [options] -r <NR> directory" << endl << endl;
  cout << bold << "OPTIONS" << normal << endl

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
  while ((c = getopt_long(argc, argv, "f:r:s:e:m:M:O:", longopts, NULL)) != -1)
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
     if (!Scan::toType(optarg, type))
       abort ();
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
    cerr << "\n*** Reduction method missed ***" << endl;
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

  cout << "(c) Jacobs University Bremen, gGmbH, 2010" << endl << endl;
  
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
 
#ifdef _MSC_VER
  int success = mkdir(reddir.c_str());
#else
  int success = mkdir(reddir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif
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
    const double* rPos = Scan::allScans[0]->get_rPos();
    const double* rPosTheta = Scan::allScans[0]->get_rPosTheta();
    
    // reduction filter for current scan!
    
    cout << "Reducing Scan No. " << fileNr << endl;
    Scan::allScans[0]->calcReducedPoints(red, octree);
    
    
    cout << "Writing Scan No. " << fileNr ;
    cout << " with " << Scan::allScans[0]->get_points_red_size() << " points" << endl; 
    string scanFileName;
    string poseFileName;
  
    scanFileName = dir + "reduced/scan" + to_string(fileNr,3) + ".3d";
    poseFileName = dir  + "reduced/scan" + to_string(fileNr,3) + ".pose";
   
     
    ofstream redptsout(scanFileName.c_str());
    for (int j = 0; j < Scan::allScans[0]->get_points_red_size(); j++) {
         Point p(Scan::allScans[0]->get_points_red()[j]);
         //Points in global coordinate system
         redptsout << p.x << " " << p.y << " " << p.z << endl;
         
	  }
    redptsout.close();
    redptsout.clear();
      

    ofstream posout(poseFileName.c_str());
    
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
