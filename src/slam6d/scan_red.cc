/*
 * scan_red implementation
 *
 * Copyright (C) Dorit Borrmann, Razvan-George Mihalyi, Remus Dumitru 
 *
 * Released under the GPL version 3.
 *
 */


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
#include <errno.h>

#include "slam6d/metaScan.h"
#include "slam6d/io_utils.h"
#include "slam6d/scan.h"
#include "slam6d/fbr/fbr_global.h"
#include "slam6d/fbr/panorama.h"
#include "slam6d/fbr/scan_cv.h"

#include "scanserver/clientInterface.h"

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

//Vertical angle of view of scanner
#define MAX_ANGLE 60.0
#define MIN_ANGLE -40.0

#define IMAGE_HEIGHT 1000
#define IMAGE_WIDTH 3600

using namespace fbr;

projection_method strToPMethod(string method){
  if(strcasecmp(method.c_str(), "EQUIRECTANGULAR") == 0) return EQUIRECTANGULAR;
  else if(strcasecmp(method.c_str(), "CYLINDRICAL") == 0) return CYLINDRICAL;
  else if(strcasecmp(method.c_str(), "MERCATOR") == 0) return MERCATOR;
  else if(strcasecmp(method.c_str(), "CONIC") == 0) return CONIC;
  else throw std::runtime_error(std::string("projection method ") + method + std::string(" is unknown"));
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
	  << "   " << prog << " [options] -r <NR> directory" << endl << endl;
  cout << bold << "OPTIONS" << normal << endl
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << endl
	  << "         [ATTENTION: counting naturally starts with 0]" << endl
	  << endl
	  << bold << "  -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << endl
	  << "         end after scan NR" << endl
	  << endl
	  << bold << "  -f" << normal << " F, " << bold << "--format=" << normal << "F" << endl
	  << "         using shared library F for input" << endl
	  << "         (choose F from {uos, uos_map, uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply})" << endl
	  << endl
	  << bold << "  -m" << normal << " NR, " << bold << "--max=" << normal << "NR" << endl
	  << "         neglegt all data points with a distance larger than NR 'units'" << endl
	  << endl
	  << bold << "  -M" << normal << " NR, " << bold << "--min=" << normal << "NR" << endl
	  << "         neglegt all data points with a distance smaller than NR 'units'" << endl
	  << endl
	  << bold << "  -r" << normal << " NR, " << bold << "--reduce=" << normal << "NR" << endl
	  << "         if NR >= 0, turns on octree based point reduction (voxel size=<NR>)" << endl
	  << "         if NR < 0, turns on rescaling based reduction" << endl
	  << endl
       << bold << "  -I" << normal << " NR," << bold << "--rangeimage=" << normal << "MET" << endl
	  << "         use rescaling of the range image as reduction method" << endl
	  << "         if NR = 1 recovers ranges from range image" << endl
	  << "         if NR = 2 interpolates 3D points in the image map" << endl
	  << endl
	  << bold << "  -p" << normal << " MET," << bold << "--projection=" << normal << "MET" << endl
	  << "         create range image using the MET projection method" << endl
	  << "         (choose MET from [EQUIRECTANGULAR|CYLINDRICAL|MERCATOR|CONIC])" << endl
    	  << bold << "  -S, --scanserver" << normal << endl
	  << "         Use the scanserver as an input method and handling of scan data" << endl
	  << endl << endl;
  
  cout << bold << "EXAMPLES " << normal << endl
	  << "   " << prog << " -m 500 -r 5 dat" << endl
	  << "   " << prog << " --max=5000 -r 10.2 dat" << endl
	  << "   " << prog << " -s 2 -e 10 -r dat" << endl
	  << "   " << prog << " -s 0 -e 1 -r 10 -I=1  dat " << endl << endl;
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
 * @param projection - projection method for building range image
 * @param quiet switches on/off the quiet mode
 * @param veryQuiet switches on/off the 'very quiet' mode
 * @return 0, if the parsing was successful. 1 otherwise
 */
int parseArgs(int argc, char **argv, string &dir, double &red, 
		    int &start, int &end, int &maxDist, int &minDist,
		    string &projection, int &octree, IOType &type,
		    int &rangeimage, bool &scanserver)
{
  bool reduced = false;
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
    { "reduce",          required_argument,   0,  'r' },
    { "octree",          optional_argument,   0,  'O' },
    { "rangeimage",      optional_argument,   0,  'I' },
    { "projection",      required_argument,   0,  'p' },
    { "scanserver",      no_argument,         0,  'S' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "f:r:s:e:m:M:O:p:", longopts, NULL)) != -1)
    switch (c)
	 {
	 case 'r':
	   red = atof(optarg);
     reduced = true;
	   break;
	 case 's':
	   w_start = atoi(optarg);
	   if (w_start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
	   break;
	 case 'e':
	   w_end = atoi(optarg);
	   if (w_end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
	   if (w_end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
	   break;
	 case 'f': 
	   try {
		w_type = formatname_to_io_type(optarg);
	   } catch (...) { // runtime_error
		cerr << "Format " << optarg << " unknown." << endl;
		abort();
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
	 case 'I':
	   if (optarg) {
		rangeimage = atoi(optarg);
	   } else {
		rangeimage = 1;
	   }
	   break;
	 case 'p':
	   projection = optarg;
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

  parseFormatFile(dir, w_type, w_start, w_end);

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

  cout << "(c) Jacobs University Bremen, gGmbH, 2012" << endl << endl;
  
  if (argc <= 1) {
    usage(argv[0]);
  }

  // parsing the command line parameters
  // init, default values if not specified
  string dir;
  double red            = -1.0;
  int    start = 0, end = -1;
  int    maxDist        = -1;
  int    minDist        = -1;
  string projection     = "EQUIRECTANGULAR";
  int    octree         = 0;
  IOType type           = RIEGL_TXT;
  int    rangeimage     = 0;
  bool   scanserver     = false;
  
  parseArgs(argc, argv, dir, red, start, end, maxDist, minDist, projection,
		  octree, type, rangeimage, scanserver);

  if (scanserver) {
    try {
	 ClientInterface::create();
    } catch(std::runtime_error& e) {
	 cerr << "ClientInterface could not be created: " << e.what() << endl;
	 cerr << "Start the scanserver first." << endl;
	 exit(-1);
    }
  }
  
  // Get Scans
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

  Scan::openDirectory(scanserver, dir, type, start, end);
  if(Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);
  }

  /// Use the OCTREE based reduction
  if (rangeimage == 0) {
    cout << endl << "Reducing point cloud using octrees" << endl;
    int scan_number = start;
    for(std::vector<Scan*>::iterator it = Scan::allScans.begin();
	   it != Scan::allScans.end(); ++it,
		++scan_number) {
      Scan* scan = *it;
      const double* rPos = scan->get_rPos();
      const double* rPosTheta = scan->get_rPosTheta();
      
      scan->setRangeFilter(maxDist, minDist);
      scan->setReductionParameter(red, octree);
      // get reduced points
      DataXYZ xyz_r(scan->get("xyz reduced"));
      unsigned int nPoints = xyz_r.size();
      // convert scan to matrix
      cv::Mat scan_cv;
      scan_cv.create(nPoints, 1, CV_32FC(3));
      scan_cv = cv::Scalar::all(0); 
      cv::MatIterator_<cv::Vec3f> it = scan_cv.begin<cv::Vec3f>();
      double zMax = numeric_limits<double>::min(); 
      double zMin = numeric_limits<double>::max();

      const char* id = scan->getIdentifier();
      cout << "Writing Scan No. " << id;
      cout << " with " << xyz_r.size() << " points" << endl; 
      string scanFileName = reddir + "/scan" + id + ".3d";
      string poseFileName = reddir  + "/scan" + id + ".pose";

      ofstream redptsout(scanFileName.c_str());
      for(unsigned int j = 0; j < nPoints; j++) {
        redptsout << xyz_r[j][0] << " " << xyz_r[j][1] << " " << xyz_r[j][2] << endl;
        (*it)[0] = xyz_r[j][0];
        (*it)[1] = xyz_r[j][1];
        (*it)[2] = xyz_r[j][2];
        //finding min and max of z                                      
        if (xyz_r[j][2]  > zMax) zMax = xyz_r[j][2];
        if (xyz_r[j][2]  < zMin) zMin = xyz_r[j][2];
        ++it;
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

      if (scanserver) {
        scan->clear("xyz reduced");
      }
    }
  } else { /// use the RESIZE based reduction
    cout << endl << "Reducing point cloud by rescaling the range image" << endl;
    int scan_number = start;
    for(std::vector<Scan*>::iterator it = Scan::allScans.begin();
	   it != Scan::allScans.end(); ++it,
		++scan_number) {
      Scan* scan = *it;
      scan->setRangeFilter(maxDist, minDist);
      // get points
      DataXYZ xyz(scan->get("xyz"));
      unsigned int nPoints = xyz.size();
      // convert scan to matrix
      cv::Mat scan_cv;
      scan_cv.create(nPoints, 1, CV_32FC(3));
      scan_cv = cv::Scalar::all(0); 
      cv::MatIterator_<cv::Vec3f> it = scan_cv.begin<cv::Vec3f>();
      double zMax = numeric_limits<double>::min(); 
      double zMin = numeric_limits<double>::max();

      const char* id = scan->getIdentifier();
      cout << "Writing Scan No. " << id;
      cout << " with " << xyz.size() << " points" << endl; 

      for(unsigned int j = 0; j < nPoints; j++) {
        //Points in global coordinate system
        (*it)[0] = xyz[j][0];
        (*it)[1] = xyz[j][1];
        (*it)[2] = xyz[j][2];
        //finding min and max of z                                      
        if (xyz[j][2]  > zMax) zMax = xyz[j][2];
        if (xyz[j][2]  < zMin) zMin = xyz[j][2];
        ++it;
      }

	 /// Project point cloud using the selected projection method
	 panorama image (IMAGE_WIDTH, IMAGE_HEIGHT, strToPMethod(projection));
	 image.createPanorama(scan_cv);
	 image.getDescription();
	 
	 /// Resize the range image, specify desired interpolation method
	 double scale = 1.0/red;
	 cv::Mat range_image_resized; // reflectance_image_resized;
	 string ofilename;
	 stringstream ss;
	 ss << setw(3) << setfill('0') << (scan_number);
	 ofilename = reddir + "/scan" + ss.str() + ".3d";
	 if (rangeimage == 1) {
	   resize(image.getRangeImage(), range_image_resized, cv::Size(), scale, scale, cv::INTER_NEAREST);
	   // Recover point cloud from image and write scan to file
	   stringstream ss;
	   ss << setw(3) << setfill('0') << (scan_number); 
	   image.recoverPointCloud(range_image_resized, ofilename);
	 } else {
	   resize(image.getMap(), range_image_resized, cv::Size(), scale, scale, cv::INTER_NEAREST);
	   ofstream redptsout(ofilename.c_str());
	   // Convert back to 3D.
	   for(int i = 0; i < range_image_resized.rows; i++) {
		for(int j = 0; j < range_image_resized.cols; j++) {
		  cv::Vec3f vec = range_image_resized.at<cv::Vec3f>(i, j);
		  double x = vec[0];
		  double y = vec[1];
		  double z = vec[2];
		  redptsout << x << " " << y << " " << z << endl;
		}
	   }
	 }
    }
  }

  cout << endl << endl;
  cout << "Normal program end." << endl << endl;

  if (scanserver) {  
    Scan::closeDirectory();
    ClientInterface::destroy();
  }
}

