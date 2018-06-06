#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <boost/program_options.hpp>

#include <slam6d/io_types.h>
#include <slam6d/globals.icc>
#include <slam6d/scan.h>
#include <scanserver/clientInterface.h>

#include "poisson/Geometry.h"
#include "poisson/PoissonParam.h"
#include "wrap/callback.h"

#include "mesh/poisson.h"

using namespace std;
using namespace vcg;

namespace po = boost::program_options;

enum normal_method {KNN, ADAPTIVE_KNN,
				AKNN, ADAPTIVE_AKNN,
#ifdef WITH_OPENCV
				PANORAMA, PANORAMA_FAST
#endif
};

// Call to excute poisson
int Execute2(PoissonParam &Par, vector<Point3D<float> > Pts, vector<Point3D<float> > Nor, 	CoredVectorMeshData &mesh, Point3D<float> &newCenter, float &newScale, vcg::CallBackPos *cb );
// Parse commandline options
void parse_options(int argc, char **argv, int &start, int &end, bool &scanserver, int &max_dist, int &min_dist, string &dir, string &odir, IOType &iotype, int &k1, int &k2, normal_method &ntype, int &width, int &height);
// validate normmal_method type (important for boost program_option)
void validate(boost::any& v, const std::vector<std::string>& values, normal_method*, int);
// validate IO types (important for boost program_option)
void validate(boost::any& v, const std::vector<std::string>& values, IOType*, int);

int main(int argc, char **argv) {
  int start, end;
  bool scanserver;
  int max_dist, min_dist;
  string dir, odir; // directory of input scan and output model
  IOType iotype;
  int k1, k2;
  normal_method ntype;
  int width, height;

  vector<Point> points;
  vector<Point> normals;
  
  // parse input arguments
  parse_options(argc, argv, start, end, scanserver, max_dist, min_dist,
                dir, odir, iotype, k1, k2, ntype, width, height);

  if (scanserver) {
    try {
      ClientInterface::create();
    } catch(std::runtime_error& e) {
      cerr << "ClientInterface could not be created: " << e.what() << endl;
      cerr << "Start the scanserver first." << endl;
      exit(-1);
    }
  }

  // Read the scans
  Scan::openDirectory(scanserver, dir, iotype, start, end);
  if(Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);
  }

  std::vector<Scan*>::iterator it = Scan::allScans.begin();
  int scanNumber = start;

  for( ; it != Scan::allScans.end(); ++it) {
    Scan* scan = *it;

    // apply optional filtering
    scan->setRangeFilter(max_dist, min_dist);

    const double* rPos = scan->get_rPos();
    const double* rPosTheta = scan->get_rPosTheta();

    // read scan into points
    DataXYZ xyz(scan->get("xyz"));
    points.reserve(xyz.size());
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
#ifdef WITH_OPENCV
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
#endif
    }
    scanNumber++;
  }

  cout << "Normal calculation end" << endl;
  cout << "Poisson reconstruction started" << endl;

  // shutdown everything
  if (scanserver)
    ClientInterface::destroy();

  Scan::closeDirectory();

  Poisson poisson;
  poisson.setVertices(points);
  poisson.setNormals(normals);
  poisson.apply();
  // CoredVectorMeshData m;
  // poisson.getMesh(&m);
  poisson.exportMesh(odir.c_str());
  cout << "Poisson reconstruction end" << endl;

  return 0;
}

// validate normmal_method type (important for boost program_option)
void validate(boost::any& v, const std::vector<std::string>& values,
              normal_method*, int) {
  if (values.size() == 0)
    throw std::runtime_error("Invalid model specification");
  string arg = values.at(0);
  if (strcasecmp(arg.c_str(), "KNN") == 0) v = KNN;
  else if (strcasecmp(arg.c_str(), "ADAPTIVE_KNN") == 0) v = ADAPTIVE_KNN;
  else if (strcasecmp(arg.c_str(), "AKNN") == 0) v = AKNN;
  else if (strcasecmp(arg.c_str(), "ADAPTIVE_AKNN") == 0) v = ADAPTIVE_AKNN;
#ifdef WITH_OPENCV
  else if (strcasecmp(arg.c_str(), "PANORAMA") == 0) v = PANORAMA;
  else if (strcasecmp(arg.c_str(), "PANORAMA_FAST") == 0) v = PANORAMA_FAST;
#endif
  else throw std::runtime_error(std::string("normal calculation method ")
                                + arg + std::string(" is unknown"));
}

// validate IO types (important for boost program_option)
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

// Parse commandline options
void parse_options(int argc, char **argv, int &start, int &end,
			    bool &scanserver, int &max_dist, int &min_dist, string &dir, string &odir,
                   IOType &iotype, int &k1, int &k2,
			    normal_method &ntype, int &width, int &height)
{
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
       "(KNN, ADAPTIVE_KNN, AKNN, ADAPTIVE_AKNN"
#ifdef WITH_OPENCV
       ", PANORAMA, PANORAMA_FAST"
#endif
       ")")
      ("K1,k",
       po::value<int>(&k1)->default_value(20),
       "<arg> value of K value used in the nearest neighbor search of ANN or"
       "kmin for k-adaptation")
      ("K2,K",
       po::value<int>(&k2)->default_value(20),
       "<arg> value of Kmax for k-adaptation")
#ifdef WITH_OPENCV
      ("width,w",
       po::value<int>(&width)->default_value(3600),
       "width of panorama image")
      ("height,h",
       po::value<int>(&height)->default_value(1000),
       "height of panorama image")
#endif
      ;

  // input scan dir and output obj dir are hidden and mandatory
  po::options_description hidden("Hidden options");
  hidden.add_options()
      ("input-dir", po::value<string>(&dir), "input dir")
      ("output-dir", po::value<string>(&odir), "output dir");

  po::positional_options_description pd;
  pd.add("input-dir", 1);
  pd.add("output-dir", 2);

  po::options_description all;
  all.add(cmd_options).add(hidden);

  po::variables_map vmap;
  po::store(po::command_line_parser(argc, argv).
            options(all).positional(pd).run(), vmap);
  po::notify(vmap);

  if (vmap.count("help")) {
    cout << cmd_options << endl << endl;
    cout << "SAMPLE COMMAND FOR CALCULATING NORMALS" << endl;
    cout << " bin/normals -s 0 -e 0 -f UOS -g AKNN -k 20 dat/ dat/test/output.obj" << endl;
    cout << endl;
    exit(-1);
  }

  // read scan path
  if (dir[dir.length()-1] != '/') dir = dir + "/";

}


Poisson::Poisson() {
  initialize();
}

Poisson::~Poisson() {
  initialize();
}

void Poisson::initialize() {
  reconstructed = 0;
  vertices.clear();
  normals.clear();
  return;
}

int Poisson::setVertices(vector<Point> v) {
  vertices.clear();
  reconstructed = 0;
  vertices.resize(v.size());
  for (int i = 0; i < vertices.size(); ++i) {
    vertices[i].coords[0] = v[i].x; 
    vertices[i].coords[1] = v[i].y; 
    vertices[i].coords[2] = v[i].z;
  }
  return 1;
}

int Poisson::setNormals(vector<Point> n) {
  normals.clear();
  reconstructed = 0;
  normals.resize(n.size());
  for (int i = 0; i < normals.size(); ++i) {
    normals[i].coords[0] = n[i].x; 
    normals[i].coords[1] = n[i].y; 
    normals[i].coords[2] = n[i].z;
  }
  return 1;
}

int Poisson::getMesh(CoredVectorMeshData *m) {
  if (!ready()) {
    return 0;
  }
  *m = mesh;
  return 1;
}

int Poisson::exportMesh(const char* modelPath) {
  fstream fs(modelPath, fstream::out);
  // // write unscaled out of core vertices
  // for (int i = 0; i < mesh.oocPoints.size(); ++i) {
  //   fs << "v " << mesh.oocPoints[i].coords[0] << " " << mesh.oocPoints[i].coords[1] << " " << mesh.oocPoints[i].coords[2] << endl;
  // }

  Point3D<float> vertex;
  TriangleIndex tIndex;
  int inCoreFlag;
  int numVerticesInCore = mesh.inCorePoints.size(),
    numVerticesOutCore = mesh.oocPoints.size(),
    numFaces=mesh.triangleCount();

  // get and write correct scaled vertices coordinates
  for (int i = 0; i < numVerticesInCore; ++i) {
    vertex = mesh.inCorePoints[i];
    fs << "v " << vertex.coords[0] * scale + center.coords[0] << " " 
      << vertex.coords[1] * scale + center.coords[1] << " " 
      << vertex.coords[2] * scale + center.coords[2] << " " << endl;
  }
  for (int i = 0; i < numVerticesOutCore; ++i) {
    vertex = mesh.oocPoints[i];
    fs << "v " << vertex.coords[0] * scale + center.coords[0] << " " 
      << vertex.coords[1] * scale + center.coords[1] << " " 
      << vertex.coords[2] * scale + center.coords[2] << " " << endl;
  }

  // get and write correct ordered faces indexes
  for (int i = 0; i < numFaces; ++i) {
    mesh.nextTriangle(tIndex, inCoreFlag);
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[0])) {
      tIndex.idx[0]+=int(mesh.inCorePoints.size());
    }
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[1])) {
      tIndex.idx[1]+=int(mesh.inCorePoints.size());
    }
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[2])) {
      tIndex.idx[2]+=int(mesh.inCorePoints.size());
    }
    fs << "f " << tIndex.idx[0] + 1 << " " 
      << tIndex.idx[1] + 1 << " " 
      << tIndex.idx[2] + 1 << " " << endl;
  }

  fs.close();
  return 0;
}

int Poisson::ready() {
  return vertices.size() > 0 && normals.size() > 0 && vertices.size() == normals.size() && reconstructed;
}

int Poisson::apply() {
  reconstructed = Execute2(params, vertices, normals, mesh, center, scale, cb);
  return reconstructed;
}
