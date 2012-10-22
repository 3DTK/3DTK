/*
 * plane implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file 
 * @author Dorit Borrmann. Institute of Computer Science, University of Osnabrueck, Germany.
*/

#include <cfloat>
#include <fstream>
#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#include <iostream>
using std::ofstream;
using std::flush;
using std::cout;
using std::string;
using std::cerr;
using std::endl;
#include <errno.h>

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

#include "shapes/hough.h"
#include "shapes/shape.h"
#include "shapes/ransac.h"

enum plane_alg { 
  RHT, SHT, PHT, PPHT, APHT, RANSAC
};

void usage(char* prog) {
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
	  << bold << "  -p" << normal << " P, " << bold << "--plane=" << normal << "P" << endl
	  << "         using algorithm P for plane detection" << endl
	  << "         (chose P from {rht, sht, pht, ppht, apht, ran})" << endl
	  << endl
	  << bold << "  -r" << normal << " NR, " << bold << "--reduce=" << normal << "NR" << endl
	  << "         turns on octree based point reduction (voxel size=<NR>)" << endl
	  << endl
	  << bold << "  -O" << normal << " NR (optional), " << bold << "--octree=" << normal << "NR (optional)" << endl
	  << "         use randomized octree based point reduction (pts per voxel=<NR>)" << endl
	  << "         requires " << bold << "-r" << normal <<" or " << bold << "--reduce" << endl
	  << endl
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << endl
	  << "         [ATTENTION: counting naturally starts with 0]" << endl
	  << endl
	  << bold << "  -S, --scanserver" << normal << endl
	  << "         Use the scanserver as an input method and handling of scan data" << endl
	  << endl
    	  << endl << endl;
  
  cout << bold << "EXAMPLES " << normal << endl
	  << "   " << prog << " -m 500 -r 5 dat" << endl
	  << "   " << prog << " --max=5000 -r 10.2 dat" << endl
	  << "   " << prog << " -s 2 -e 10 -r dat" << endl << endl;
  exit(1);

}

/**
  * Parses command line arguments needed for plane detection. For details about
  * the argument see usage().
  */

int parseArgs(int argc, char **argv, string &dir, double &red, int &start, int
		    &maxDist, int&minDist, int &octree, IOType &type, plane_alg &alg, bool
		    &quiet, bool& scanserver)
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
    { "reduce",          required_argument,   0,  'r' },
    { "plane",           required_argument,   0,  'p' },
    { "quiet",            no_argument,         0,  'q' },
    { "octree",          optional_argument,   0,  'O' },
    { "scanserver",      no_argument,         0,  'S' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "f:r:s:e:m:M:p:O:q", longopts, NULL)) != -1) 
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
	 case 'f':
     try {
       type = formatname_to_io_type(optarg);
     } catch (...) { // runtime_error
       cerr << "Format " << optarg << " unknown." << endl;
       abort();
     }
     break;
   case 'p': 
      if(strcasecmp(optarg, "rht") == 0) alg = RHT;
      else if(strcasecmp(optarg, "sht") == 0) alg = SHT;
      else if(strcasecmp(optarg, "pht") == 0) alg = PHT;
      else if(strcasecmp(optarg, "ppht") == 0) alg = PPHT;
      else if(strcasecmp(optarg, "apht") == 0) alg = APHT;
      else if(strcasecmp(optarg, "ran") == 0) alg = RANSAC;
      else abort();
      break;
	 case 'q':
	   quiet = true;
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
	 case 'S':
        scanserver = true;
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

/**
 * Main function. The Hough Transform or RANSAC are called for the scan indicated as
 * argument.
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
  int    start = 0;
  int    maxDist    = -1;
  int    minDist    = -1;
  int    octree     = 0;
  bool   quiet = false;
  IOType type    = UOS;
  plane_alg alg    = RHT;
  bool   scanserver = false;
  
  cout << "Parse args" << endl;
  parseArgs(argc, argv, dir, red, start, maxDist, minDist, octree, type, alg, quiet, scanserver);
  int fileNr = start;
  string planedir = dir + "planes"; 

#ifdef _MSC_VER
  int success = mkdir(planedir.c_str());
#else
  int success = mkdir(planedir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif
  if(success == 0) { 
    if(!quiet) {
      cout << "Writing planes to " << planedir << endl;
    }
  } else if(errno == EEXIST) {
    cout << "Directory " << planedir << " exists already.  CONTINUE" << endl; 
  } else {
    cerr << "Creating directory " << planedir << " failed" << endl;
    exit(1);
  }

  Scan::openDirectory(scanserver, dir, type, fileNr, fileNr);
  Scan* scan = Scan::allScans.front();
  scan->setRangeFilter(maxDist, minDist);
  scan->setReductionParameter(red, octree);
  //    scan->setSearchTreeParameter(nns_method, use_cuda);
  scan->toGlobal();

  double id[16];
  M4identity(id);
  for(int i = 0; i < 10; i++) {
    scan->transform(id, Scan::ICP, 0);  // write end pose
  }

  if (!quiet) cout << "start plane detection" << endl;
  long starttime = GetCurrentTimeInMilliSec(); 
  if(alg >= RANSAC) {
    vector<double *> points;
    CollisionPlane<double> * plane;
    plane = new CollisionPlane<double>(1.0); // 1.0 cm maxdist
    Ransac(*plane, Scan::allScans[0], &points);
    starttime = (GetCurrentTimeInMilliSec() - starttime);

    cout << "nr points " << points.size() << endl;
    double nx,ny,nz,d;
    plane->getPlane(nx,ny,nz,d);
    if(!quiet) cout << "DONE " << endl;

    if(!quiet) cout << nx << " " << ny << " " << nz << " " << d << endl;

    /* 
    for (unsigned int i = 0; i < points.size(); i++) {
      cerr << points[i][0] << " " << points[i][1] << " " << points[i][2] << endl;
    }
    */
    for(int i = points.size() - 1; i > -1; i--) {
      delete[] points[i];
    }
    
    delete plane;
  } else {
    Hough hough(Scan::allScans[0], quiet);
    starttime = (GetCurrentTimeInMilliSec() - starttime);
    cout << "Time for Constructor call: " << starttime << endl;

    starttime = GetCurrentTimeInMilliSec(); 
    if (!quiet) cout << "algorithm: " << alg << endl;
    // choose Hough method here
    switch(alg) {
      case RHT: hough.RHT();
                break;
      case SHT: hough.SHT();
                break;
      case PHT: hough.PHT();
                break;
      case PPHT:  hough.PPHT();
                  break;
      case APHT:  hough.APHT();
                  break;
      default:  usage(argv[0]);
                exit(1);
                break;
    }

    hough.writePlanes(0);
    cout << "Write Planes done" << endl;
    starttime = (GetCurrentTimeInMilliSec() - starttime);
  }

  cout << "Time for Plane Detection " << starttime << endl;
  delete Scan::allScans[0];
  Scan::allScans.clear();
}

