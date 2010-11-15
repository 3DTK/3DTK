/**
 * @file 
 * @author Dorit Borrmann. Institute of Computer Science, University of Osnabrueck, Germany.
*/

//#include "sparse/csparse.h"

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

#include "slam6d/hough.h"

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
	  << "         (chose P from {rht, sht, pht, ppht, apht})" << endl
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
int parseArgs(int argc, char **argv, string &dir, double &red, int &start, int
  &maxDist, int&minDist, int &octree, reader_type &type, plane_alg &alg, bool
  &quiet) {

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
    { "quit",            no_argument,         0,  'q' },
    { "octree",          optional_argument,   0,  'O' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "f:r:s:e:m:M:p:Oq", longopts, NULL)) != -1) 
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
	   if (strcasecmp(optarg, "uos") == 0) type = UOS;
	   else if (strcasecmp(optarg, "uos_map") == 0) type = UOS_MAP;
	   else if (strcasecmp(optarg, "uos_frames") == 0) type = UOS_FRAMES;
	   else if (strcasecmp(optarg, "uos_map_frames") == 0) type = UOS_MAP_FRAMES;
	   else if (strcasecmp(optarg, "uos_rgb") == 0) type = UOS_RGB;
	   else if (strcasecmp(optarg, "old") == 0) type = OLD;
	   else if (strcasecmp(optarg, "rts") == 0) type = RTS;
	   else if (strcasecmp(optarg, "rts_map") == 0) type = RTS_MAP;
	   else if (strcasecmp(optarg, "ifp") == 0) type = IFP;
	   else if (strcasecmp(optarg, "riegl_txt") == 0) type = RIEGL_TXT;
	   else if (strcasecmp(optarg, "riegl_rgb") == 0) type = RIEGL_RGB;
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
	   else if (strcasecmp(optarg, "rxp") == 0) type = RXP;
	   else {
		 abort ();
	   }
	   break;
   case 'p': 
      if(strcasecmp(optarg, "sht") == 0) alg = RHT;
      else if(strcasecmp(optarg, "rht") == 0) alg = SHT;
      else if(strcasecmp(optarg, "pht") == 0) alg = PHT;
      else if(strcasecmp(optarg, "ppht") == 0) alg = PPHT;
      else if(strcasecmp(optarg, "apht") == 0) alg = APHT;
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
 * Main function. The Hough Transform is called for the scan indicated as
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
  reader_type type    = UOS;
  plane_alg alg    = RHT;
  
  parseArgs(argc, argv, dir, red, start, maxDist, minDist, octree, type, alg, quiet);
  Scan::dir = dir;
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
  Scan::readScans(type, fileNr, fileNr, dir, maxDist, minDist, 0);
  // reduction filter for current scan!
  Scan::allScans[0]->calcReducedPoints(red, octree);
  
  long starttime = GetCurrentTimeInMilliSec(); 
  double id[16];
  M4identity(id);
  for(int i = 0; i < 10; i++) {
    Scan::allScans[0]->transform(id, Scan::ICP, 0);  // write end pose
  }
  Hough hough(Scan::allScans[0], quiet);

  starttime = (GetCurrentTimeInMilliSec() - starttime);
  cout << "Time for Constructor call: " << starttime << endl;

  starttime = GetCurrentTimeInMilliSec(); 
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

  hough.writePlanes();
  starttime = (GetCurrentTimeInMilliSec() - starttime);
  cout << "Time for Hough Transform: " << starttime << endl;
  delete Scan::allScans[0];
  Scan::allScans.clear();
}

