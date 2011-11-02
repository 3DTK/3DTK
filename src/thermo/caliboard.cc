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
#include "slam6d/icp6D.h"
#include "slam6d/icp6Dsvd.h"
#include "slam6d/icp6Dquat.h"

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
	  << bold << "  -p" << normal << " NR, " << bold << "--pattern=" << normal << "NR" << endl
	  << "         use pattern NR for plane detection" << endl
	  << "         0: lightbulb pattern" << endl
	  << "         1: chess pattern on cardboard" << endl
	  << "         0: chess pattern on wooden board" << endl
	  << endl
	  << bold << "  -c" << normal << " P, " << bold << "--output=" << normal << "P" << endl
	  << "         write the output data to file P" << endl
    << endl
    << bold << "  -r" << normal << " NR, " << bold << "--reduce=" << normal << "NR" << endl
	  << "         turns on octree based point reduction (voxel size=<NR>)" << endl
	  << endl
	  << bold << "  -O" << normal << "NR (optional), " << bold << "--octree=" << normal << "NR (optional)" << endl
	  << "         use randomized octree based point reduction (pts per voxel=<NR>)" << endl
	  << "         requires " << bold << "-r" << normal <<" or " << bold << "--reduce" << endl
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

vector<double *> * matchPlaneToBoard(vector<double *> &points, double *alignxf, int pattern, string output) {
  double rPos[3] = {0.0,0.0,0.0};
  double rPosTheta[3] = {0.0,0.0,0.0};

  vector<double *> boardpoints;
  double halfwidth; 
  double halfheight; 
  
  switch(pattern) {
    case 0: 
      halfheight = 28.5;
      halfwidth = 25.0;
      break;
    case 1:
      halfwidth = 18.3;
      halfheight = 18.5;
      break;
    case 2:
      halfwidth = 19.0;
      halfheight = 38.0;
      break;
  }
  double step = 0.5;

  for(double i = -halfwidth; i <= halfwidth; i+=step) {
    for(double j = -halfheight; j <= halfheight; j+=step) {
      double * p = new double[3];
      p[0] = i;
      p[1] = j;
      p[2] = 0.0;
      //cout << p[0] << " " << p[1] << " " << p[2] << endl;
      boardpoints.push_back(p);
    }
  }
  
  Scan * plane = new Scan(rPos, rPosTheta, points);
  Scan * board = new Scan(rPos, rPosTheta, boardpoints);
  board->transform(alignxf, Scan::INVALID, 0);
  
  bool quiet = true;
  icp6Dminimizer *my_icp6Dminimizer = 0;
  my_icp6Dminimizer = new icp6D_SVD(quiet);

  icp6D *my_icp = 0;
  double mdm = 50;
  int mni = 1500;
  my_icp = new icp6D(my_icp6Dminimizer, mdm, mni, quiet, false, -1, false, 1, 0.00, false, false);

  plane->createTree(false,false);
  board->createTree(false,false);

  my_icp->match(plane, board);

  const double * pos = board->get_rPos();
  const double * postheta = board->get_rPosTheta();
  const double * transMat = board->get_transMat();
  for(int i = 0; i < 16; i++) {
    cout << transMat[i] << " ";
  }

  cout << endl << endl;
  
  for(int i = 0; i < 3; i++) {
    cout << pos[i] << " ";
  }
  cout << endl;
  for(int i = 0; i < 3; i++) {
    cout << deg(postheta[i]) << " ";
  }
  cout << endl;
  vector<double *> * result = new vector<double *>();
  cout << "Calipoints Start" << endl;
  
  ofstream caliout(output.c_str());
  switch(pattern) {
  case 0:
    for(double x = -20; x < 25; x+=10.0) {
      for(double y = -25; y < 30; y+=10.0) {
        double * p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = 0.0;
        transform3(transMat, p);
        result->push_back(p);
        caliout << p[0] << " " << p[1] << " " << p[2] << endl;
      }
    }
    break;
  case 1:
    for(double x = -7.8; x < 10; x+=5.2) {
      for(double y = 4.1; y < 33.0; y+=5.2) {
        double * p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = 0.0;
        transform3(transMat, p);
        result->push_back(p);
        caliout << p[0] << " " << p[1] << " " << p[2] << endl;
      }
    }
    break;
  case 2:
    for(double x = -12; x < 16; x+=4.0) {
      for(double y = -12; y < 16; y+=4.0) {
        double * p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = 0.0;
        transform3(transMat, p);
        result->push_back(p);
        caliout << p[0] << " " << p[1] << " " << p[2] << endl;
      }
    }
    break;
  } 
  caliout.close();
  caliout.clear();

  cout << "Calipoints End" << endl;
  return result;
}

int parseArgs(int argc, char **argv, string &dir, double &red, int &start, int pattern, 
string &output, int &maxDist, int&minDist, int &octree, reader_type &type, bool &quiet) {

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
    { "pattern",         required_argument,   0,  'p' },
    { "quiet",           no_argument,         0,  'q' },
    { "octree",          optional_argument,   0,  'O' },
    { "output",          required_argument,   0,  'c' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "f:r:s:e:m:M:O:qp:c:", longopts, NULL)) != -1) 
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
     if (!Scan::toType(optarg, type))
       abort ();
     break;
   case 'p': 
      pattern = atoi(optarg);
      if(pattern < 0 || pattern > 2) { cerr << "Error: choose pattern between 0 and 2!\n"; exit(1); }
      break;
   case 'c':
     output = optarg;
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
  int    pattern = 0;
  string output = "cali.3d";
  reader_type type    = UOS;
  
  cout << "Parse args" << endl;
  parseArgs(argc, argv, dir, red, start, pattern, output, maxDist, minDist, octree, type, quiet);
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
  Scan::allScans[0]->toGlobal(red, octree);
  double id[16];
  M4identity(id);
  for(int i = 0; i < 10; i++) {
    Scan::allScans[0]->transform(id, Scan::ICP, 0);  // write end pose
  }
  cout << "start plane detection" << endl;
  long starttime = GetCurrentTimeInMilliSec(); 
  vector<double *> points;
  CollisionPlane<double> * plane;
  plane = new LightBulbPlane<double>(5.0,120);
  Ransac(*plane, Scan::allScans[0], &points);
  starttime = (GetCurrentTimeInMilliSec() - starttime);

  cout << "nr points " << points.size() << endl;
  double nx,ny,nz,d;
  plane->getPlane(nx,ny,nz,d);
  cout << "DONE " << endl;

  cout << nx << " " << ny << " " << nz << " " << d << endl;
  double rPos[3];
  double rPosTheta[3];
  for(int i = 0; i < 3; i++) {
    rPosTheta[i] = 0.0;
  }
  ((LightBulbPlane<double> *)plane)->getCenter(rPos[0], rPos[1], rPos[2]);
  double alignxf[16];
  EulerToMatrix4(rPos, rPosTheta, alignxf);
  matchPlaneToBoard(points, alignxf, pattern, output);
  for(int i = points.size() - 1; i > -1; i++) {
    delete[] points[i];
  }
  
  delete plane;

  cout << "Time for Plane Detection " << starttime << endl;
  delete Scan::allScans[0];
  Scan::allScans.clear();
}

