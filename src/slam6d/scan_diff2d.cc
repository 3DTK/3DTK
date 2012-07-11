/*
 * scan_diff2d implementation
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
 * Program for calculating the 2d-difference between scans after matching with slam6d 
 * Usage: bin/scan_diff2d -s <START> -e <END> -d <NR> 'dir',
 * Use -s for the first scan, -e for the second scan
 *  'dir' the directory of a set of scans
 * The result is an image showing the color coded difference of the two scans
 * ATTENTION: All scans between START and END will be loaded!
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
using std::ifstream;
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
#include "show/colormanager.h"

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

#include "slam6d/kd.h"
#include "slam6d/kdc.h"
#include <limits.h>
#include <algorithm>
using namespace std;



int WIDTH=0;
int HEIGHT=0;
double MINTHETA;
double MAXTHETA;
double MINPHI;
double MAXPHI;

double minphi,maxphi;
double mintheta,maxtheta;
bool **stencilImage1;
bool **stencilImage;
double **rangeImage;
double **rangeImage2;
Point **pointImage;

void writeRI(const char *filename, double max, double shift)
{
  SHSVMap cmap;
  float color[3];
  unsigned int buckets = 4096;

  ofstream fp;                  // The PPM File
  unsigned char *ibuffer;       // The PPM Output Buffer
  // Allocate memory for the the frame buffer and output buffer
  ibuffer = new unsigned char[WIDTH * HEIGHT * 3];

  // Open the output file
  fp.open(filename, ios::out);

  // Write a proper P6 PPM header
  fp << "P6" << endl << "# CREATOR: 3D_Viewer by Andreas Nuechter, University of Osnabrueck"
	<< endl << WIDTH  << " " << HEIGHT << " " << UCHAR_MAX << endl;

  // Loop through the frame buffer data, writing to the PPM file.  Be careful
  //   to account for the frame buffer having 4 bytes per pixel while the
  //   output file has 3 bytes per pixel
  int l = 0;
  for (int i = 0; i < HEIGHT; i++) {     // For each row
    for (int jj = 0; jj < WIDTH; jj++) {    // For each column
      int j = (jj + (int)(WIDTH*shift))%WIDTH;
      if(!stencilImage[j][i]) { 
        ibuffer[l++] = UCHAR_MAX; 
        ibuffer[l++] = UCHAR_MAX;
        ibuffer[l++] = UCHAR_MAX;
      } else {
        cmap.calcColor(color, (max + rangeImage[j][i])/(max*2.0) * buckets, buckets);
        ibuffer[l++] = (unsigned char)min( (int)((color[0])*UCHAR_MAX), UCHAR_MAX);
        ibuffer[l++] = (unsigned char)min( (int)((color[1])*UCHAR_MAX), UCHAR_MAX);
        ibuffer[l++] = (unsigned char)min( (int)((color[2])*UCHAR_MAX), UCHAR_MAX);
      }
    }                                    // end column
  }                                      // end row

  // Write output buffer to the file */
  fp.write((const char*)ibuffer, sizeof(unsigned char) * (3 * WIDTH * HEIGHT));
  fp.close();
  fp.clear();
  delete [] ibuffer;
}


void toRI(double _x, double _y, double _z,  int &X, int &Y, double &range) {
  double x = _z/100.0;
  double y = _x/(-100.0);
  double z = _y/100.0;

  double phi = 0.0;
  double theta = 0.0;
  range = std::sqrt(x*x + y*y + z*z);
  
  if (range > 0.00000001) {
    phi = atan2(y,x);
    phi = ((phi<0.0)?(phi+2.0*M_PI):phi);
    theta = std::acos(z/range);

    if (phi > maxphi) maxphi = phi;
    if (phi < minphi) minphi = phi;
    if (theta > maxtheta) maxtheta = theta;
    if (theta < mintheta) mintheta = theta;
  }


  X = WIDTH * (phi/( MAXPHI - MINPHI ));
  Y = HEIGHT * ((theta - MINTHETA)/( MAXTHETA - MINTHETA ));
}

void toPoint(int X, int Y, double range, double &x, double &y, double &z) {
  x = y = z = 0.0;
  
  if (range > 0.000000001) {
    double _z = cos(((double)Y/(double)HEIGHT * (double)(MAXTHETA - MINTHETA))   + MINTHETA) * range;
    double _y = sin((double)X/(double)WIDTH * (double)(MAXPHI - MINPHI)) * range;
    double _x = cos((double)X/(double)WIDTH * (double)(MAXPHI - MINPHI)) * range;
    x = _y*-100.0;
    y = _z*100.0;
    z = _x*100.0;
  }
}

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
		    IOType &type, bool &desc, double &res, double &shift, int &mode)
{
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
    { "dist",            required_argument,   0,  'd' },
    { "resolution",      required_argument,   0,  'r' },
    { "shift",           required_argument,   0,  'h' },
    { "mode",            required_argument,   0,  '0' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "h:r:f:d:s:e:m:M:", longopts, NULL)) != -1)
    switch (c)
	 {
	 case '0':
	   mode = atoi(optarg);
	   break;
	 case 'h':
	   shift = atof(optarg);
	   break;
	 case 'r':
	   res = atof(optarg);
	   break;
	 case 'd':
	   dist = atof(optarg);
	   break;
	 case 's':
	   start = atoi(optarg);
	   if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
	   break;
	 case 'e':
	   end = atoi(optarg);
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
      default:
	   abort ();
      }

  if(start < 0 || end < 0 ) {
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

  cout << "(c) Jacobs University Bremen, gGmbH, 2010" << endl << endl;
  
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
  double resolution = 0.25;
  double shift = 0.5;
  int mode = 0;

  parseArgs(argc, argv, dir, start, end, maxDist, minDist, dist, type, desc, resolution, shift, mode);

  resolution = rad(resolution);

  // Get Scans (all scans between start and end)
  Scan::dir = dir;
 
  string scanFileName;
  string framesFileName;
  ifstream frames_in;
  double inMatrix0[17];
  double inMatrix1[17];
  
  cout << "Reading Scan No. " << start;
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
  cout << "Reading Scan No. " << end;
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
  
  Scan::readScans(type, start, end, dir, maxDist, minDist, 0);
  int endIndex = Scan::allScans.size() - 1;

  Scan *rangescan = Scan::allScans[0];
  Scan *targetscan = Scan::allScans[endIndex];
  Scan::allScans[endIndex]->calcReducedPoints(-1, 0);

  double tinv[16]; M4inv(inMatrix0, tinv);
  double tmat[16];
  MMult(tinv, inMatrix1, tmat);

  Scan::allScans[endIndex]->transform(tmat, Scan::INVALID);   
 
  string name;
  int thread_num = 0;
  vector<double*> diff;
  double transMat[16];
 

  cout << "set up range image" << endl;
  double x,y,z,r;
  int X,Y;
  // first subsample to RI
  const vector<Point> *points = rangescan->get_points();
  maxphi = maxtheta = -DBL_MAX;
  mintheta = minphi = DBL_MAX;
  cout << "establish max and mins..." << endl;
  double MINDIST = 40.0;

  for (unsigned int i = 0; i < points->size(); i++) {
    Point p = (*points)[i];
    ////////////////////////////
    if ( sqrt(p.x*p.x + p.z*p.z ) < MINDIST ) continue;
    ///////////////////////////
    toRI(p.x, p.y, p.z, X, Y, r);
  }
  MAXTHETA = maxtheta;
  MINTHETA = mintheta;
  MAXPHI = maxphi;
  MINPHI = minphi;
  HEIGHT = (MAXTHETA - MINTHETA)/resolution;
  WIDTH = (MAXPHI - MINPHI)/resolution;

  switch (mode) {
    case 0: 
      {
        cout << "creating data structures " << WIDTH << " " << HEIGHT << endl;
        stencilImage = new bool*[WIDTH+1];
        rangeImage = new double*[WIDTH+1];
        pointImage = new Point*[WIDTH+1];
        for (int i = 0; i <= WIDTH; i++) {
          stencilImage[i] = new bool[HEIGHT+1];
          rangeImage[i] = new double[HEIGHT+1];
          pointImage[i] = new Point[HEIGHT+1];
          for (int j = 0; j <= HEIGHT; j++) {
            rangeImage[i][j] = DBL_MAX;
            stencilImage[i][j] = false;
          }
        }
        cout << "thin out points " << points->size() << endl;
        for (unsigned int i = 0; i < points->size(); i++) {
          Point p = (*points)[i];
          ////////////////////////////
          if ( sqrt(p.x*p.x + p.z*p.z ) < MINDIST ) continue;
          ///////////////////////////
          toRI(p.x, p.y, p.z, X, Y, r);
          if (X >= 0 && X < WIDTH && Y >= 0 && Y < HEIGHT ) {
            if (rangeImage[X][Y] > r) {
              stencilImage[X][Y] = true;
              rangeImage[X][Y] = r;
              pointImage[X][Y] = p;
            }
          } 
        }


        cout << "done setting up range image" << endl;

        targetscan->createTree(false, false);
        const SearchTree *kd = targetscan->get_tree();

        cout << " looking for closest points... " << endl;
        double mdm2 = dist*dist;
        for (int X = 0; X < WIDTH; X++) {
          for (int Y = 0; Y < HEIGHT; Y++) {
            if (!stencilImage[X][Y]) continue;
            double p[3];
            p[0] = pointImage[X][Y].x;
            p[1] = pointImage[X][Y].y;
            p[2] = pointImage[X][Y].z;

            double *closest = kd->FindClosest(p, mdm2, 0);

            if (closest) {
              rangeImage[X][Y] = sqrt(Dist2(closest, p));
              double v[3];
              for (int i = 0; i < 3; i++)
                v[i] = closest[i] - p[i];

              double d = v[0]*p[0] + v[1]*p[1] + v[2]*p[2];
              if (d <= 0) {
                rangeImage[X][Y] *= -1.0;
              }

            } else {
              rangeImage[X][Y] = dist;
            }
          }
        }
      }
      break;
    case 1:
      {
        cout << "creating data structures " << WIDTH << " " << HEIGHT << endl;
        stencilImage = new bool*[WIDTH+1];
        stencilImage1 = new bool*[WIDTH+1];
        rangeImage = new double*[WIDTH+1];
        rangeImage2 = new double*[WIDTH+1];
        for (int i = 0; i <= WIDTH; i++) {
          stencilImage[i] = new bool[HEIGHT+1];
          stencilImage1[i] = new bool[HEIGHT+1];
          rangeImage[i] = new double[HEIGHT+1];
          rangeImage2[i] = new double[HEIGHT+1];
          for (int j = 0; j <= HEIGHT; j++) {
            rangeImage[i][j] = DBL_MAX;
            rangeImage2[i][j] = DBL_MAX;
            stencilImage[i][j] = false;
            stencilImage1[i][j] = false;
          }
        }
        cout << "thin out points " << points->size() << endl;
        for (unsigned int i = 0; i < points->size(); i++) {
          Point p = (*points)[i];
          ////////////////////////////
          if ( sqrt(p.x*p.x + p.z*p.z ) < MINDIST ) continue;
          ///////////////////////////
          toRI(p.x, p.y, p.z, X, Y, r);
          if (X >= 0 && X < WIDTH && Y >= 0 && Y < HEIGHT ) {
            if (rangeImage[X][Y] > r) {
              stencilImage1[X][Y] = true;
              rangeImage[X][Y] = fabs(r);
            }
          } 
        }

        for (unsigned int i = 0; i < Scan::allScans[endIndex]->get_points_red_size(); i++) {
          Point p;
          p.x = Scan::allScans[endIndex]->get_points_red()[i][0];
          p.y = Scan::allScans[endIndex]->get_points_red()[i][1];
          p.z = Scan::allScans[endIndex]->get_points_red()[i][2];
          ////////////////////////////
          if ( sqrt(p.x*p.x + p.z*p.z ) < MINDIST ) continue;
          ///////////////////////////
          toRI(p.x, p.y, p.z, X, Y, r);
          if (X >= 0 && X < WIDTH && Y >= 0 && Y < HEIGHT ) {
            if (rangeImage2[X][Y] > r && stencilImage1[X][Y]) {
              stencilImage[X][Y] = true;
              rangeImage2[X][Y] = fabs(r);
            }
          } 
        }

        cout << "done setting up range image" << endl;
        for (int X = 0; X < WIDTH; X++) {
          for (int Y = 0; Y < HEIGHT; Y++) {
            if (!stencilImage[X][Y]) continue;
            if (rangeImage[X][Y] == DBL_MAX) continue;
            rangeImage[X][Y] = 100.0 * (rangeImage[X][Y] - rangeImage2[X][Y]) ; // to meters
          }
        }

      }
    break;
    default:
    break;
  }

  
  name = dir + "diff_" + to_string(start, 3) + "_" + to_string(end,3) +  ".ppm";
  writeRI(name.c_str(), dist, shift);
      
  delete Scan::allScans[0];
  Scan::allScans.clear();

  cout << endl << endl;
  cout << "Normal program end." << endl << endl;
}
