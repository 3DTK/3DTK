/*
 * caliboard implementation
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
#include <float.h>
#include <windows.h>
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <strings.h>
#include <dlfcn.h>
#include <cmath>
#endif

#include "shapes/hough.h"
#include "slam6d/basicScan.h"

#include "shapes/shape.h"
#include "shapes/ransac.h"
#include "slam6d/icp6D.h"
#include "slam6d/icp6Dsvd.h"
#include "slam6d/icp6Dquat.h"

#ifdef WITH_SCANSERVER
#include "scanserver/clientInterface.h"
#endif



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
	  << bold << "  -b" << normal << " NR, " << bold << "--bottom=" << normal << "NR" << endl
	  << "         trim the scan with lower boundary NR" << endl
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
	  << bold << "  -p" << normal << " NR, " << bold << "--pattern=" << normal << "NR" << endl
	  << "         use pattern NR for plane detection" << endl
	  << "         0: lightbulb pattern" << endl
	  << "         1: chess pattern on cardboard" << endl
	  << "         2: chess pattern on wooden board" << endl
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
	  << bold << "  -t" << normal << " NR, " << bold << "--top=" << normal << "NR" << endl
	  << "         trim the scan with upper boundary NR" << endl
	  << endl
    	  << endl << endl;
  
  cout << bold << "EXAMPLES " << normal << endl
	  << "   " << prog << " -m 500 -r 5 dat" << endl
	  << "   " << prog << " --max=5000 -r 10.2 dat" << endl
	  << "   " << prog << " -s 2 -e 10 -r dat" << endl << endl;
  exit(1);

}

void writeFalse(string output) {
  ofstream caliout(output.c_str());
  
  caliout << "failed" << endl;
  caliout.close();
  caliout.clear();
}

bool matchPlaneToBoard(vector<double *> &points, double *alignxf, int pattern, string output) {
  double rPos[3] = {0.0,0.0,0.0};
  double rPosTheta[3] = {0.0,0.0,0.0};

  vector<double *> boardpoints;
  double halfwidth; 
  double halfheight; 
  double w_step = 0.5;
  double h_step = 0.5;

  switch(pattern) {
    case 0: 
      halfheight = 28.5;
      halfwidth = 25.0;
      break;
    case 1:
      halfwidth = 18.3;
      halfheight = 18.5;
      w_step = 0.6;
      break;
    case 2:
    case 3:
      halfwidth = 19.0;
      halfheight = 38.0;
      break;
    case 4: //Ostia
      /*
      halfwidth = 14.85; 
      halfheight = 21;
      */
      halfwidth = 22.5;
      halfheight = 30.5;
      break;
    case 5: // Ostia large
      halfwidth = 22.5;
      halfheight = 30.5;
      break;
  }

  for(double i = -halfwidth; i <= halfwidth; i+=w_step) {
    for(double j = -halfheight; j <= halfheight; j+=h_step) {
      double * p = new double[3];
      p[0] = i;
      p[1] = j;
      p[2] = 0.0;
      //cout << p[0] << " " << p[1] << " " << p[2] << " 1" << endl;
      boardpoints.push_back(p);
    }
  }
 
  int nr_points = boardpoints.size();
  int nr_points2 = points.size();
  Scan * plane = new BasicScan(rPos, rPosTheta, points);
  Scan * board = new BasicScan(rPos, rPosTheta, boardpoints);
  
  for(int i = 0; i < boardpoints.size(); i++) {
    delete[] boardpoints[i];
  }
  
  plane->setRangeFilter(-1, -1);
  plane->setReductionParameter(-1, 0);
  plane->setSearchTreeParameter(simpleKD);
  board->setRangeFilter(-1, -1);
  board->setReductionParameter(-1, 0);
  board->setSearchTreeParameter(simpleKD);
  
  board->transform(alignxf, Scan::ICP, 0);
  
  bool quiet = true;
  icp6Dminimizer *my_icp6Dminimizer = 0;
  my_icp6Dminimizer = new icp6D_SVD(quiet);

  icp6D *my_icp = 0;
  double mdm = 50;
  int mni = 50;
  my_icp = new icp6D(my_icp6Dminimizer, mdm, mni, quiet, false, -1, false, 1, 0.00, false, false);
  my_icp->match(plane, board);
  delete my_icp;
  
  mdm = 2;
  mni = 300;
  my_icp = new icp6D(my_icp6Dminimizer, mdm, mni, quiet, false, -1, false, 1, 0.00, false, false);
  my_icp->match(plane, board);
  delete my_icp;
  delete my_icp6Dminimizer;

  double sum; 
  double centroid_s[3] = {0.0, 0.0, 0.0};
  double centroid_t[3] = {0.0, 0.0, 0.0};
  vector<PtPair> pairs_out;
  Scan::getPtPairs(&pairs_out, plane, board, 1, 0, 3.0, sum, centroid_s, centroid_t);  
  int nr_matches = pairs_out.size();
  
  cout << "Result " << nr_matches << " " << nr_points << " " << nr_points2 << endl;
  const double * pos = board->get_rPos();
  const double * postheta = board->get_rPosTheta();
  const double * transMat = board->get_transMat();
  for(int i = 0; i < 16; i++) {
    cout << transMat[i] << " ";
  }

  cout << endl << endl;
  cout << "Transform new: " << endl; 
  for(int i = 0; i < 3; i++) {
    cout << pos[i] << " ";
  }
  cout << endl;
  for(int i = 0; i < 3; i++) {
    cout << deg(postheta[i]) << " ";
  }
  cout << endl;
  cout << "Calipoints Start" << endl;
  
  ofstream caliout(output.c_str());
  
  if(nr_matches < nr_points) {
    caliout << "failed" << endl;
  } else {
    caliout << "Calibration" << endl; 
  }
 
  /**
   * write FRAMES
   */
  /*
  string filename = "tmp.frames";
    
  ofstream fout(filename.c_str());
  if (!fout.good()) {
	 cerr << "ERROR: Cannot open file " << filename << endl;
	 exit(1);
  }

  // write into file

  //fout << "frames for scan" << endl;
  //fout << plane->sout.str() << endl;
  //fout << "frames for lightbulbs" << endl;
  fout << board ->sout.str() << endl;
  fout.close();
  fout.clear();
  */

  /*
  board->saveFrames();
  */
  /**
   * end write frames
   */

  switch(pattern) {
  // lightbulb
  case 0:
    for(double y = -25; y < 30; y+=10.0) {
      for(double x = 20; x > -25; x-=10.0) {
        double * p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = 0.0;
        transform3(transMat, p);
        //result->push_back(p);
        caliout << p[0] << " " << p[1] << " " << p[2] << endl;
        delete[] p;
      }
    }
    break;
  // chessboard on wooden board pattern top
  case 2:
    for(double x = -7.8; x < 10; x+=5.2) {
      for(double y = 4.1; y < 33.0; y+=5.2) {
        double * p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = 0.0;
        transform3(transMat, p);
        caliout << p[0] << " " << p[1] << " " << p[2] << endl;
        delete[] p;
      }
    }
    break;
  // chessboard on wooden board pattern bottom 
  case 3:
    for(double y = -4.1; y > -33.0; y-=5.2) {
    for(double x = -8.1; x < 10; x+=5.2) {
        double * p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = 0.0;
        transform3(transMat, p);
        caliout << p[0] << " " << p[1] << " " << p[2] << endl;
        delete[] p;
      }
    }
    break;
  // chessboard on cardboard
  case 1:
    for(double x = -12; x < 16; x+=4.0) {
      for(double y = -12; y < 16; y+=4.0) {
        double * p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = 0.0;
        transform3(transMat, p);
        caliout << p[0] << " " << p[1] << " " << p[2] << endl;
        delete[] p;
      }
    }
    break;
    // Ostia
  case 4:
    for(double x = -12; x < 16; x+=8.0) {
      for(double y = -20; y < 24; y+=8.0) {
        double * p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = 0.0;
        transform3(transMat, p);
        caliout << p[0] << " " << p[1] << " " << p[2] << endl;
        delete[] p;
      }
    }
    break;
    break;

  //_|_|_|_|_|_|_
  //4: 1.5*8 = 12
  //6: 2.5*8 = 20
  // Universum
  case 5:
    for(double y = 20; y > -24; y-=8.0) {
      for(double x = -12; x < 16; x+=8.0) {
        double * p = new double[3];
        p[0] = x;
        p[1] = y;
        p[2] = 0.0;
        transform3(transMat, p);
        caliout << p[0] << " " << p[1] << " " << p[2] << endl;
        delete[] p;
      }
    }
    break;

  } 
  caliout.close();
  caliout.clear();

  cout << "Calipoints End" << endl;
  delete board;
  delete plane;
  return !(nr_matches < nr_points);
}

int parseArgs(int argc, char **argv, string &dir, double &red, int &start, int
&end, int &pattern, int &maxDist, int &minDist, double &top, double &bottom, int
&octree, IOType &type, bool &quiet) {

  bool reduced = false;
  int  c;
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
    { "end",             required_argument,   0,  'e' },
    { "top",             required_argument,   0,  't' },
    { "bottom",          required_argument,   0,  'b' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "f:r:s:e:m:M:O:qp:e:t:b:", longopts, NULL)) != -1) 
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
	   if (end < 0) { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
	   if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
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
      pattern = atoi(optarg);
      if(pattern < 0 || pattern > 5) { cerr << "Error: choose pattern between 0 and 3!\n"; exit(1); }
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
	 case 't':
     top = atof(optarg);
	   break;
	 case 'b':
	   bottom = atof(optarg);
     break;
	 case 'M':
	   minDist = atoi(optarg);
	   break;
   case '?':
	   usage(argv[0]);
	   return 1;
   default:
	    abort ();
      break;
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
  int    end = -1;
  int    maxDist    = -1;
  int    minDist    = -1;
  int    octree     = 0;
  bool   quiet = false;
  int    pattern = 0;
  double bottom = -5;
  double top = 170;
  IOType type    = UOS;

  cout << "Parse args" << endl;
  parseArgs(argc, argv, dir, red, start, end, pattern, maxDist, minDist, top, bottom, octree, type, quiet);
  int fileNr = start;
  string calidir = dir + "/cali";
  
#ifdef WITH_SCANSERVER
  try {
    ClientInterface::create();
  } catch(std::runtime_error& e) {
    cerr << "ClientInterface could not be created: " << e.what() << endl;
    cerr << "Start the scanserver first." << endl;
    exit(-1);
  }
#endif //WITH_SCANSERVER

#ifdef _MSC_VER
  int success = mkdir(calidir.c_str());
#else
  int success = mkdir(calidir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif
  if(success == 0) {
    if(!quiet) {
      cout << "Writing calibration results to " << calidir << endl;
    }
  } else if(errno == EEXIST) {
    cout << "Directory " << calidir << " exists already.  CONTINUE" << endl;
  } else {
    cerr << "Creating directory " << calidir << " failed" << endl;
    exit(1);
  }

  cout << start << " " << end << endl;
  int successes = 0;
  int failures = 0;
  
  long calitime = GetCurrentTimeInMilliSec();

//#ifndef WITH_SCANSERVER
  while (fileNr <= end) {
    Scan::openDirectory(false, dir, type, fileNr, fileNr);
    Scan::allScans[0]->setRangeFilter(maxDist, minDist);
    Scan::allScans[0]->setHeightFilter(top, bottom);
    Scan::allScans[0]->setSearchTreeParameter(simpleKD);

    string output = calidir + "/scan" + to_string(fileNr,3) + ".3d";
    cout << "Top: " << top << " Bottom: " << bottom << endl;

    Scan::allScans[0]->toGlobal();
    double id[16];
    M4identity(id);
    for(int i = 0; i < 10; i++) {
      Scan::allScans[0]->transform(id, Scan::ICP, 0);  // write end pose
    }
/*
#else //WITH_SCANSERVER

  Scan::readScansRedSearch(type, start, end, dir, filter, red, octree);
  for(std::vector<Scan*>::iterator it = Scan::allScans.begin(); it != Scan::allScans.end(); ++it)
  {
    Scan* scan = *it;
    string output = calidir + "/scan" + scan->getIdentifier() + ".3d";
    cout << "Top: " << top << " Bottom: " << bottom << endl;
    // set trimming, don't want to put it into readScansRedSearch too
    scan->trim(top, bottom);

    double id[16];
    M4identity(id);
    for(int i = 0; i < 10; i++) {
      scan->transform(id, Scan::ICP, 0);  // write end pose
    }
#endif //WITH_SCANSERVER
*/
    cout << "start plane detection" << endl;
    long starttime = GetCurrentTimeInMilliSec();
    vector<double *> points;
    CollisionPlane<double> * plane;
    plane = new LightBulbPlane<double>(50,120);
//#ifndef WITH_SCANSERVER
    Ransac(*plane, Scan::allScans[0], &points);
/*
#else //WITH_SCANSERVER

    cout << "S" << endl;
    Ransac(*plane, scan, &points);
    cout << "T" << endl;
//#endif //WITH_SCANSERVER
*/
    starttime = (GetCurrentTimeInMilliSec() - starttime);

    cout << "nr points " << points.size() << endl;
    double nx,ny,nz,d;
    plane->getPlane(nx,ny,nz,d);
    cout << "DONE " << endl;

    cout << nx << " " << ny << " " << nz << " " << d << endl;
    
#ifdef _MSC_VER
	if(_isnan(d)) {
#else
    if(std::isnan(d)) {
#endif
      writeFalse(output);
      failures++;
    } else {

      if(d < 0) {
        nx = -nx;
        ny = -ny;
        nz = -nz;
        d = -d;
      }

      double tx, ty, tz;
      tz = 0;
      ty = asin(nx);
      tx = asin(-ny/cos(ty));

      double rPos[3];
      double rPosTheta[3];

      for(int i = 0; i < 3; i++) {
        rPosTheta[i] = 0.0;
      }

      rPosTheta[0] = tx;
      rPosTheta[1] = ty;
      rPosTheta[2] = tz;

      //rPosTheta[1] = acos(nz);
      // rotate plane model to make parallel with detected plane

      // transform plane model to center of detected plane
      ((LightBulbPlane<double> *)plane)->getCenter(rPos[0], rPos[1], rPos[2]);
      cout << "Angle: " << deg(acos(nz)) << endl;
      for(int i = 0; i < 3; i++) {
        cout << rPos[i] << " ";
      }
      for(int i = 0; i < 3; i++) {
        cout << deg(rPosTheta[i]) << " ";
      }
      cout << endl;

      double alignxf[16];
      EulerToMatrix4(rPos, rPosTheta, alignxf);
      cout << "A" << endl;
      if(matchPlaneToBoard(points, alignxf, pattern, output)) {
        successes++;
      } else {
        failures++;
      }
      cout << "blub" << endl;

    }
   
    
    for(int i = points.size() - 1; i > -1; i--) {
      delete[] points[i];
    }
    
    
    delete plane;

    cout << "Time for Plane Detection " << starttime << endl;
//#ifndef WITH_SCANSERVER
    delete Scan::allScans[0];
    Scan::allScans.clear();
    fileNr++;
//#endif //WITH_SCANSERVER
  }
  calitime = (GetCurrentTimeInMilliSec() - calitime);

  cout << "Calibration done with " << successes << " successes and " << failures
  << " failures!" << endl;
  cout << "Time for Calibration " << calitime << endl;

/*
#ifdef WITH_SCANSERVER
  Scan::clearScans();
  ClientInterface::destroy();
#endif //WITH_SCANSERVER
*/
}

