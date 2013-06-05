/*
 * slam6D implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief efficient normal computation
 *
 * @author Vaibhav Kumar Mehta. Jacobs University Bremen gGmbH, Germany
 */

#include <iostream>
#include <string>
#include <fstream>
#include <errno.h>

#include <boost/program_options.hpp>

#include <slam6d/io_types.h>
#include <slam6d/globals.icc>
#include <slam6d/scan.h>
#include <scanserver/clientInterface.h>

#include <slam6d/normals.h>
#include <normals/normals_panorama.h>

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#include <direct.h>
#define mkdir(path,mode) _mkdir (path)
#else
#include <strings.h>
#endif

namespace po = boost::program_options;
using namespace std;

enum normal_method {KNN, ADAPTIVE_KNN,
				AKNN, ADAPTIVE_AKNN,
				PANORAMA, PANORAMA_FAST};

/*
 * validates normal calculation method specification
 */
void validate(boost::any& v, const std::vector<std::string>& values,
              normal_method*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  string arg = values.at(0);
  if (strcasecmp(arg.c_str(), "KNN") == 0) v = KNN;
  else if (strcasecmp(arg.c_str(), "ADAPTIVE_KNN") == 0) v = ADAPTIVE_KNN;
  else if (strcasecmp(arg.c_str(), "AKNN") == 0) v = AKNN;
  else if (strcasecmp(arg.c_str(), "ADAPTIVE_AKNN") == 0) v = ADAPTIVE_AKNN;
  else if (strcasecmp(arg.c_str(), "PANORAMA") == 0) v = PANORAMA;
  else if (strcasecmp(arg.c_str(), "PANORAMA_FAST") == 0) v = PANORAMA_FAST;
  else throw std::runtime_error(std::string("normal calculation method ")
                                + arg + std::string(" is unknown"));
}

/// validate IO types
void validate(boost::any& v, const std::vector<std::string>& values,
              IOType*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  string arg = values.at(0);
  try {
    v = formatname_to_io_type(arg.c_str());
  } catch (...) { // runtime_error
    throw std::runtime_error("Format " + arg + " unknown.");
  }
}

/// Parse commandline options
void parse_options(int argc, char **argv, int &start, int &end,
			    bool &scanserver, int &max_dist, int &min_dist, string &dir,
                   IOType &iotype, int &k1, int &k2,
			    normal_method &ntype, int &width, int &height)
{
  /// ----------------------------------
  /// set up program commandline options
  /// ----------------------------------
  po::options_description cmd_options("Usage: calculateNormals <options> "
                                      "where options are (default values "
                                      "in brackets)");
  cmd_options.add_options()
      ("help,?", "Display this help message")
      ("start,s",
       po::value<int>(&start)->default_value(0),
       "Start at scan number <arg>")
      ("end,e",
       po::value<int>(&end)->default_value(-1),
       "Stop at scan number <arg>")
      ("scanserver,S",
       po::value<bool>(&scanserver)->default_value(false),
       "Use the scanserver as an input method")
      ("format,f",
       po::value<IOType>(&iotype)->default_value(UOS),
       "using shared library <arg> for input. (chose format from "
       "[uos|uosr|uos_map|uos_rgb|uos_frames|uos_map_frames|old|rts|rts_map"
       "|ifp|riegl_txt|riegl_rgb|riegl_bin|zahn|ply])")
      ("max,M",
       po::value<int>(&max_dist)->default_value(-1),
       "neglegt all data points with a distance larger than <arg> 'units")
      ("min,m",
       po::value<int>(&min_dist)->default_value(-1),
       "neglegt all data points with a distance smaller than <arg> 'units")
      ("normal,g",
       po::value<normal_method>(&ntype)->default_value(AKNN),
       "normal calculation method "
       "(KNN, ADAPTIVE_KNN, AKNN, ADAPTIVE_AKNN, PANORAMA, PANORAMA_FAST)")
      ("K1,k",
       po::value<int>(&k1)->default_value(20),
       "<arg> value of K value used in the nearest neighbor search of ANN or"
       "kmin for k-adaptation")
      ("K2,K",
       po::value<int>(&k2)->default_value(20),
       "<arg> value of Kmax for k-adaptation")
      ("width,w",
       po::value<int>(&width)->default_value(3600),
       "width of panorama image")
      ("height,h",
       po::value<int>(&height)->default_value(1000),
       "height of panorama image")
      ;

  po::options_description hidden("Hidden options");
  hidden.add_options()
      ("input-dir", po::value<string>(&dir), "input dir");

  po::positional_options_description pd;
  pd.add("input-dir", 1);

  po::options_description all;
  all.add(cmd_options).add(hidden);

  po::variables_map vmap;
  po::store(po::command_line_parser(argc, argv).
            options(all).positional(pd).run(), vmap);
  po::notify(vmap);

  if (vmap.count("help")) {
    cout << cmd_options << endl << endl;
    cout << "SAMPLE COMMAND FOR CALCULATING NORMALS" << endl;
    cout << " bin/normals -s 0 -e 0 -f UOS -g AKNN -k 20 dat/" << endl;
    cout << endl << endl;
    cout << "SAMPLE COMMAND FOR VIEWING CALCULATING NORMALS IN RGB SPACE"
         << endl;
    cout << " bin/show -c -f UOS_RGB dat/normals/" << endl;
    exit(-1);
  }

  // read scan path
  if (dir[dir.length()-1] != '/') dir = dir + "/";

}

/// Write a pose file with the specofied name
void writePoseFiles(string dir,
                    const double* rPos, const double* rPosTheta,
                    int scanNumber)
{
  string poseFileName = dir + "/scan" + to_string(scanNumber, 3) + ".pose";
  ofstream posout(poseFileName.c_str());

  posout << rPos[0] << " "
         << rPos[1] << " "
         << rPos[2] << endl
         << deg(rPosTheta[0]) << " "
         << deg(rPosTheta[1]) << " "
         << deg(rPosTheta[2]) << endl;
  posout.clear();
  posout.close();
}

/// write scan files for all segments
void writeScanFiles(string dir,
                    vector<Point> &points, vector<Point> &normals,
                    int scanNumber)
{
  string ofilename = dir + "/scan" + to_string(scanNumber, 3) + ".3d";
  ofstream normptsout(ofilename.c_str());

  for (size_t i=0; i<points.size(); ++i) {
    int r,g,b;
    r = (int)(normals[i].x * (127.5) + 127.5);
    g = (int)(normals[i].y * (127.5) + 127.5);
    b = (int)(fabs(normals[i].z) * (255.0));
    normptsout << points[i].x << " "
               << points[i].y << " "
               << points[i].z << " "
               << r << " " << g << " " << b << " " << endl;
  }
  normptsout.clear();
  normptsout.close();
}

/// =============================================
/// Main
/// =============================================
int main(int argc, char** argv)
{
  int start, end;
  bool scanserver;
  int max_dist, min_dist;
  string dir;
  IOType iotype;
  int k1, k2;
  normal_method ntype;
  int width, height;

  parse_options(argc, argv, start, end, scanserver, max_dist, min_dist,
                dir, iotype, k1, k2, ntype, width, height);

  /// ----------------------------------
  /// Prepare and read scans
  /// ----------------------------------
  if (scanserver) {
    try {
      ClientInterface::create();
    } catch(std::runtime_error& e) {
      cerr << "ClientInterface could not be created: " << e.what() << endl;
      cerr << "Start the scanserver first." << endl;
      exit(-1);
    }
  }

  /// Make directory for saving the scan segments
  string normdir = dir + "normals";

  int success = mkdir(normdir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);

  if(success == 0) {
    cout << "Writing normals to " << normdir << endl;
  } else if(errno == EEXIST) {
    cout << "WARN: Directory " << normdir
         << " exists already. Contents will be overwriten" << endl;
  } else {
    cerr << "Creating directory " << normdir << " failed" << endl;
    exit(1);
  }

  /// Read the scans
  Scan::openDirectory(scanserver, dir, iotype, start, end);
  if(Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);
  }

  cv::Mat img;

  /// -----------------------------------------
  /// Initialize and perform normal calculation
  /// -----------------------------------------
  std::vector<Scan*>::iterator it = Scan::allScans.begin();
  int scanNumber = 0;

  for( ; it != Scan::allScans.end(); ++it) {
    Scan* scan = *it;

    // apply optional filtering
    scan->setRangeFilter(max_dist, min_dist);

    const double* rPos = scan->get_rPos();
    const double* rPosTheta = scan->get_rPosTheta();

    /// read scan into points
    DataXYZ xyz(scan->get("xyz"));
    vector<Point> points;
    points.reserve(xyz.size());
    vector<Point> normals;
    normals.reserve(xyz.size());

    for(unsigned int j = 0; j < xyz.size(); j++) {
      points.push_back(Point(xyz[j][0], xyz[j][1], xyz[j][2]));
    }

    if (ntype == KNN)
      calculateNormalsKNN(normals, points, k1, rPos);
    else if (ntype == ADAPTIVE_KNN)
      calculateNormalsAdaptiveKNN(normals, points, k1, k2, rPos);
    else if (ntype == AKNN)
      calculateNormalsApxKNN(normals, points, k1, rPos);
    else if (ntype == ADAPTIVE_AKNN)
      calculateNormalsAdaptiveApxKNN(normals, points, k1, k2, rPos);
    else
    {
      // create panorama
      fbr::panorama fPanorama(width, height, fbr::EQUIRECTANGULAR,
                              1, 0, fbr::EXTENDED);
      fPanorama.createPanorama(scan2mat(scan));

      if(ntype == PANORAMA)
        calculateNormalsPANORAMA(normals,
                                 points,
                                 fPanorama.getExtendedMap(),
                                 rPos);
      else if(ntype == PANORAMA_FAST)
        calculateNormalsFAST(normals,
                             points,
                             fPanorama.getRangeImage(),
                             fPanorama.getMaxRange(),
                             rPos,
                             fPanorama.getExtendedMap());
    }

    // pose file (repeated for the number of segments
    writePoseFiles(normdir, rPos, rPosTheta, scanNumber);
    // scan files for all segments
    writeScanFiles(normdir, points,normals,scanNumber);

    scanNumber++;
  }

  cout << "Normal program end" << endl;

  return 0;  

  // shutdown everything
  if (scanserver)
    ClientInterface::destroy();

  Scan::closeDirectory();
}
