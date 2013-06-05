/**
  * Point Cloud Segmentation using Felzenszwalb-Huttenlocher Algorithm
  *
  * Copyright (C) Jacobs University Bremen
  *
  * Released under the GPL version 3.
  *
  * @author Billy Okal <b.okal@jacobs-university.de>
  * @author Andreas Nuechter <a.nuechter@jacobs-university.de>
  * @author Mihai-Cotizo Sima
  * @file fhsegmentation.cc
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

#include <segmentation/FHGraph.h>

#include <slam6d/normals.h>
#include <math.h>

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
                   IOType &iotype, float &sigma, int &k, int &neighbors,
                   float &eps, float &radius, int &min_size)
{
  /// ----------------------------------
  /// set up program commandline options
  /// ----------------------------------
  po::options_description cmd_options("Usage: fhsegmentation <options> where options are (default values in brackets)");
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
     "using shared library <arg> for input. "
     "(chose format from [uos|uosr|uos_map|"
     "uos_rgb|uos_frames|uos_map_frames|old|rts|rts_map|ifp|"
     "riegl_txt|riegl_rgb|riegl_bin|zahn|ply])")
    ("max,M",
     po::value<int>(&max_dist)->default_value(-1),
     "neglegt all data points with a distance larger than <arg> 'units")
    ("min,m",
     po::value<int>(&min_dist)->default_value(-1),
     "neglegt all data points with a distance smaller than <arg> 'units")
    ("K,k",
     po::value<int>(&k)->default_value(1),
     "<arg> value of K value used in the FH segmentation")
    ("neighbors,N",
     po::value<int>(&neighbors)->default_value(1),
     "use approximate <arg>-nearest neighbors search or "
     "limit the number of points")
    ("sigma,v",
     po::value<float>(&sigma)->default_value(1.0),
     "Set the Gaussian variance for smoothing to <arg>")
    ("radius,r",
     po::value<float>(&radius)->default_value(-1.0),
     "Set the range of radius search to <arg>")
    ("eps,E",
     po::value<float>(&eps)->default_value(1.0),
     "Set error threshold used by the AKNN algorithm to <arg>")
    ("minsize,z",
     po::value<int>(&min_size)->default_value(0),
     "Keep segments of size at least <arg>")
    ;

  po::options_description hidden("Hidden options");
  hidden.add_options()
    ("input-dir", po::value<string>(&dir), "input dir");

  po::positional_options_description pd;
  pd.add("input-dir", 1);

  po::options_description all;
  all.add(cmd_options).add(hidden);

  po::variables_map vmap;
  po::store(po::command_line_parser(argc, argv).options(all).positional(pd).run(), vmap);
  po::notify(vmap);

  if (vmap.count("help")) {
    cout << cmd_options << endl;
    exit(-1);
  }

  // read scan path
  if (dir[dir.length()-1] != '/') dir = dir + "/";

}

/// distance measures
double weight1(Point a, Point b)
{
  return a.distance(b);
}

double weight2(Point a, Point b)
{
  //return 0;
  return (1 -(a.nx * b.nx + a.ny * b.ny + a.nz * b.nz));
 
  return a.distance(b) * .5 + fabs(a.reflectance-b.reflectance) * .5;
}

/// Write a pose file with the specofied name
void writePoseFile(string dir,
			    const double* rPos, const double* rPosTheta,
			    int num)
{
  string poseFileName = dir + "segments/scan" + to_string(num, 3) + ".pose";
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
int writeScanFiles(string dir, int outnum, int min_size,
			    const double* rPos, const double* rPosTheta,
			    const vector<vector<Point>* > cloud)
{
  int written = 0;
  for (int i = outnum, j = 0;
	  i < (int)cloud.size() && j < (int)cloud.size();
	  j++) {
    if ((min_size <= 0) || ((int)cloud[j]->size() > min_size)) {
	   vector<Point>* segment = cloud[j];
	   string scanFileName = dir + "segments/scan" + to_string(i,3) + ".3d";
	   ofstream scanout(scanFileName.c_str());
	   
	   for (int k = 0; k < (int)segment->size(); k++) {
		Point p = segment->at(k);
		scanout << p.x << " " << p.y << " " << p.z << endl;
	   }
	   scanout.close();
	   writePoseFile(dir, rPos, rPosTheta, i);
	   i++;
	   written += 1;
    }
  }
  return written;
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
  float sigma;
  int k, neighbors;
  float eps;
  float radius;
  int min_size;

  parse_options(argc, argv, start, end, scanserver, max_dist, min_dist,
                dir, iotype, sigma, k, neighbors, eps, radius, min_size);

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
  string segdir = dir + "segments";

  int success = mkdir(segdir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
  if(success == 0) {
    cout << "Writing segments to " << segdir << endl;
  } else if(errno == EEXIST) {
    cout << "WARN: Directory " << segdir << " exists already. Contents will be overwriten" << endl;
  } else {
    cerr << "Creating directory " << segdir << " failed" << endl;
    exit(1);
  }

  /// Read the scans
  Scan::openDirectory(scanserver, dir, iotype, start, end);
  if(Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);
  }

  /// --------------------------------------------
  /// Initialize and perform segmentation
  /// --------------------------------------------
  std::vector<Scan*>::iterator it = Scan::allScans.begin();
  int outscan = start;

  for( ; it != Scan::allScans.end(); ++it) {
    Scan* scan = *it;
    const double* rPos = scan->get_rPos();
    const double* rPosTheta = scan->get_rPosTheta();

    /// read scan into points
    DataXYZ xyz(scan->get("xyz"));
    DataNormal norm(scan->get("normal"));
   
    vector<Point> points;
    points.reserve(xyz.size());

    for(unsigned int j = 0; j < xyz.size(); j++) {
	 Point p(xyz[j][0], xyz[j][1], xyz[j][2],
		    norm[j][0], norm[j][1], norm[j][2]);
	 points.push_back(p);
    }

    /// create the graph and get the segments
    cout << "creating graph" << endl;
    FHGraph sgraph(&points, weight2, sigma, eps, neighbors, radius);

    cout << "segmenting graph" << endl;
    edge* sedges = sgraph.getGraph();
    universe* segmented = segment_graph(sgraph.getNumPoints(), 
                                        sgraph.getNumEdges(), 
                                        sedges,
                                        k);

    delete [] sedges;
    int nr = segmented->num_sets();
    cout << "Obtained " << nr << " segment(s)" << endl;

    /// write point clouds with segments
    vector< vector<Point>* > clouds;
    clouds.reserve(nr);
    for (int i = 0; i < nr; i++)
      clouds.push_back( new vector<Point> );

    map<int, int> components2cloud;
    int kk = 0;

    for (int i = 0; i < sgraph.getNumPoints(); ++i) {
      int component = segmented->find(i);
      if ( components2cloud.find(component)==components2cloud.end() )
        {
          components2cloud[component] = kk++;
          clouds[components2cloud[component]]
            ->reserve(segmented->size(component));
        }
      clouds[components2cloud[component]]->push_back(sgraph[i]);
    }
    
    // scan files for all segments
    int written = writeScanFiles(dir, outscan, min_size,
						   rPos, rPosTheta,
						   clouds);

    cout << "Written " << written << " segment(s) with at least size of "
	    << min_size << endl;
    outscan += written;

    /// clean up
    sgraph.dispose();
  }

  // shutdown everything
  if (scanserver)
    ClientInterface::destroy();
  else
    Scan::closeDirectory();

  cout << "Normal program end" << endl;

  return 0;
}

