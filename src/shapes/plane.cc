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

#ifdef _WIN32
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

#include "scanio/framesreader.h"

#include <boost/filesystem.hpp>
#include <boost/program_options.hpp>
namespace po = boost::program_options;

enum plane_alg {
  RHT, SHT, PHT, PPHT, APHT, RANSAC
};

int parse_options(int argc, char **argv, string &dir, double &red, int &start, int
        &maxDist, int&minDist, int &octree, IOType &type, plane_alg &alg, bool
        &quiet, bool& scanserver, float &cube_size)
{

  po::options_description generic("Generic options");
    generic.add_options()
      ("help,h", "output this help message");
    po::options_description input("Input options");
    input.add_options()
      ("format,f", po::value<IOType>(&type)->default_value(UOS,"uos"),
        "using shared library <arg> for input. (chose F from {uos, uos_map, "
        "uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, "
        "riegl_txt, riegl_rgb, riegl_bin, zahn, ply, las})")
      ("start,s", po::value<int>(&start)->default_value(0),
        "start at scan <arg> (i.e., neglects the first <arg> scans)"
        "[ATTENTION: counting naturally starts with 0]")
      ("max,m", po::value<int>(&maxDist)->default_value(-1),
        "neglegt all data points with a distance larger than <arg> 'units'")
      ("min,M", po::value<int>(&minDist)->default_value(-1),
        "neglegt all data points with a distance smaller than <arg> 'units'")
      ("reduce,r", po::value<double>(&red)->default_value(-1.0),
        "turns on octree based point reduction (voxel size= <arg>)")
      ("plane,p", po::value<plane_alg>(&alg)->default_value(RHT,"rht"),
        "Plane Detection Algorithm. Choose p from {rht, sht, pht, ppht, apht, ran}")
      ("cubesize,c", po::value<float>(&cube_size)->default_value(50.0),
        "Use cubesize to change the size of the smallest cube in the octtree <arg>")
      ("quiet,q", po::value<bool>(&quiet)->default_value(false),
        "Quiet: <arg>")
      ("octree,O", po::value<int>(&octree)->default_value(0),
        "Use randomized octree based point reduction (pts per voxel=<arg>)")
      ("scanserver,S", po::value<bool>(&scanserver)->default_value(false),
        "Use the scanserver as an input method and handling of scan data");

    po::options_description hidden("Hidden options");
    hidden.add_options()
      ("input-dir", po::value<std::string>(&dir), "input dir");

    // all options
    po::options_description all;
    all.add(generic).add(input).add(hidden);

    // options visible with --help
    po::options_description cmdline_options;
    cmdline_options.add(generic).add(input);

    // positional argument
    po::positional_options_description pd;
    pd.add("input-dir", 1);

    // process options
    po::variables_map vm;
    po::store(po::command_line_parser(argc, argv).
              options(all).positional(pd).run(), vm);

    // display help
    if (vm.count("help")) {
      std::cout << cmdline_options;
      std::cout << std::endl
           << "Example usage:" << std::endl
           << "\t .bin/planes -f uosr -m 1500 -p rht -s 0 dat/wue_city/" << std::endl;
      exit(0);
    }
    po::notify(vm);

  #ifndef _MSC_VER
    if (dir[dir.length()-1] != '/') dir = dir + "/";
  #else
    if (dir[dir.length()-1] != '\\') dir = dir + "\\";
  #endif

  return 0;
}

void validate(boost::any& v, const std::vector<std::string>& values,
              IOType*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  std::string arg = values.at(0);
  try {
    v = formatname_to_io_type(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}

plane_alg formatname_to_plane_alg(const char* string){
  if (strcasecmp(string, "rht") == 0) return RHT;
  else if (strcasecmp(string, "sht") == 0) return SHT;
  else if (strcasecmp(string, "pht") == 0) return PHT;
  else if (strcasecmp(string, "ppht") == 0) return PPHT;
  else if (strcasecmp(string, "apht") == 0) return APHT;
  else if (strcasecmp(string, "ran") == 0) return RANSAC;
  else throw std::runtime_error(std::string("Plane Detection Algorithm ") + string + std::string(" is unknown"));
}

void validate(boost::any& v, const std::vector<std::string>& values,
              plane_alg*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  std::string arg = values.at(0);
  try {
    v = formatname_to_plane_alg(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}

/**
 * Main function. The Hough Transform or RANSAC are called for the scan indicated as
 * argument.
 *
 */
int main(int argc, char **argv)
{

  cout << "(c) Jacobs University Bremen, gGmbH, 2010" << endl << endl;

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
  float cube_size = 50.0;

  cout << "Parse args" << endl;
  //parseArgs(argc, argv, dir, red, start, maxDist, minDist, octree, type, alg, quiet, scanserver, cube_size);
  parse_options(argc, argv, dir, red, start, maxDist, minDist, octree, type, alg, quiet, scanserver, cube_size);
  int fileNr = start;
  string planedir = dir + "planes";

#ifdef _WIN32
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
   //   scan->toGlobal();

  bool usePose = false;
  // Check if .frames or .pose should be used
  {
    string framesfilepath = dir + "scan" + to_string(start,3) + ".frames";
    ifstream f( framesfilepath.c_str() );
    if ( !f.good() && !usePose ) usePose = true;
    f.close();
  }
  readFramesAndTransform(dir, start, start, -1, usePose, (red > -1) );

  // why the hell should you do that? why 10 times? i will just leave it there for now...
//   double id[16];
//   M4identity(id);
//   for(int i = 0; i < 10; i++) {
//     scan->transform(id, Scan::ICP, 0);  // write end pose
//   }

  if (!quiet) cout << "start plane detection" << endl;
  long starttime = GetCurrentTimeInMilliSec();
  // why would you only consider the very first scan?
  if(alg >= RANSAC) {
      Hough hough(Scan::allScans[0], quiet);

      if(cube_size != 50.0) {
          cout << "alternate cube size of octtree: " << cube_size << endl;
      }

      DataXYZ xyz(Scan::allScans[0]->get("xyz reduced"));
      RansacOctTree<double>* oct = new RansacOctTree<double>(PointerArray<double>(xyz).get(), xyz.size(), cube_size );

      unsigned int stop = (unsigned int)(hough.allPoints->size()/100.0)*hough.myConfigFileHough.Get_MinSizeAllPoints();
      int counter = 0;
      while(hough.allPoints->size() > stop &&
            hough.planes.size() < (unsigned int)hough.myConfigFileHough.Get_MaxPlanes() &&
            counter < (int)hough.myConfigFileHough.Get_TrashMax()) {

          vector<double *> points;
          CollisionPlane<double> * plane;
          plane = new CollisionPlane<double>(1.0); // 1.0 cm maxdist
          Ransac(*plane, oct, &points);

          cout << "nr points " << points.size() << endl;
          double nx,ny,nz,d;
          plane->getPlane(nx,ny,nz,d);
          if(!quiet) cout << "DONE " << endl;

          if(!quiet) cout << nx << " " << ny << " " << nz << " " << d << endl;

          double * normal = new double[4];
          normal[0] = nx;
          normal[1] = ny;
          normal[2] = nz;

          for(int i = points.size() - 1; i > -1; i--) {
              delete[] points[i];
          }

          hough.deletePoints(normal, -d);
          delete plane;
          delete normal;
      }

      hough.writePlanes(0);
      delete oct;
      starttime = (GetCurrentTimeInMilliSec() - starttime);

  // for hough transform, consider all scans, not just the first
  } else {
    Hough hough(Scan::allScans, quiet);
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
      default:  exit(1);
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

