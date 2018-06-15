#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include <boost/program_options.hpp>

#include <slam6d/io_types.h>
#include <slam6d/globals.icc>
#include <slam6d/scan.h>
#include <scanserver/clientInterface.h>
#include <slam6d/point.h>
#include <slam6d/globals.icc>

#include "poisson/Geometry.h"
#include "poisson/PoissonParam.h"
#include "wrap/callback.h"

#include "mesh/poisson.h"

using namespace std;
using namespace vcg;

// test mesh filtering with vcglib ---
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/smooth.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>
class MyFace;
class MyVertex;
struct MyUsedTypes : public UsedTypes<	Use<MyVertex>::AsVertexType, Use<MyFace>::AsFaceType>{};
class MyVertex : public Vertex< MyUsedTypes, vertex::VFAdj, vertex::Coord3f, vertex::Normal3f, vertex::BitFlags  >{};
class MyFace : public Face  < MyUsedTypes, face::VFAdj, face::Normal3f, face::VertexRef, face::BitFlags > {};
class MyMesh : public vcg::tri::TriMesh<vector<MyVertex>, vector<MyFace> > {};
// test mesh filtering with vcglib ---

namespace po = boost::program_options;

enum normal_method {KNN, ADAPTIVE_KNN,
				AKNN, ADAPTIVE_AKNN,
#ifdef WITH_OPENCV
				PANORAMA, PANORAMA_FAST
#endif
};

enum mesh_filter {
  SMOOTH
};

// Call to excute poisson
int Execute2(PoissonParam &Par, vector<Point3D<float>> Pts, vector<Point3D<float>> Nor, CoredVectorMeshData &mesh, Point3D<float> &newCenter, float &newScale, vcg::CallBackPos *cb );
// Parse commandline options and assign to parameters
void parse_options(int argc, char **argv, int &start, int &end, bool &scanserver, int &max_dist, int &min_dist, string &dir, string &odir, IOType &iotype, 
  bool &join, double &red, int &rand, bool &uP, bool &use_xyz, bool &use_color, bool &use_reflectance, int &octree, bool &rangeFilterActive, bool &customFilterActive, string &customFilter, double &scaleFac, bool &hexfloat, bool &high_precision, int &frame,
  int &k1, int &k2, normal_method &ntype, int &width, int &height, 
  int &depth, int &solverDivide, float &samplesPerNode, float &offset);
// validate normmal_method type (important for boost program_option)
void validate(boost::any& v, const std::vector<std::string>& values, normal_method*, int);
// validate IO types (important for boost program_option)
void validate(boost::any& v, const std::vector<std::string>& values, IOType*, int);
// read frams used by join scans
void readFrames(std::string dir, int start, int end, int frame, bool use_pose=false);
// calculate normals
void calcNormals(vector<Point> &points, vector<Point> &normals, normal_method ntype, int k1, int k2, int width, int height, const double* rPos, const double* rPosTheta);

int main(int argc, char **argv) {
  // parameters for io
  int start, end;
  bool scanserver;
  int max_dist, min_dist;
  string dir, odir; // directory of input scan and output model
  IOType iotype;

  // parameters for join scans
  bool join;
  double red;
  int rand;
  bool uP;  // should we use the pose information instead of the frames?? 
  bool use_xyz;
  bool use_color;
  bool use_reflectance;
  int octree;  // employ randomized octree reduction?
  bool rangeFilterActive;
  bool customFilterActive = false;
  std::string customFilter;
  double scaleFac;
  bool hexfloat;
  bool high_precision;
  int frame;

  // parameters for normal calculation
  int k1, k2;
  normal_method ntype;
  int width, height;

  // parameters for poisson
  int depth;
	int solverDivide;
	float samplesPerNode;
  float offset;
  Poisson poisson;
  PoissonParam pp;

  vector<Point> points;
  vector<Point> normals;
  
  // parse input arguments
  parse_options(argc, argv, start, end, scanserver, max_dist, min_dist,dir, odir, iotype, 
    join, red, rand, uP, use_xyz, use_color, use_reflectance, octree, rangeFilterActive, customFilterActive, customFilter, scaleFac, hexfloat, high_precision, frame,
    k1, k2, ntype, width, height,
    depth, solverDivide, samplesPerNode, offset);

  if (scanserver) {
    try {
      ClientInterface::create();
    } catch(std::runtime_error& e) {
      cerr << "ClientInterface could not be created: " << e.what() << endl;
      cerr << "Start the scanserver first." << endl;
      exit(-1);
    }
  }

  rangeFilterActive = min_dist > 0 || max_dist > 0;

  // custom filter set? quick check, needs to contain at least one ';' 
  // (proper chsecking will be done case specific in pointfilter.cc)
  size_t pos = customFilter.find_first_of(";");
  if (pos != std::string::npos){
      customFilterActive = true;

      // check if customFilter is specified in file
      if (customFilter.find("FILE;") == 0){
          std::string selection_file_name = customFilter.substr(5, customFilter.length());
          std::ifstream selectionfile;
          // open the input file
          selectionfile.open(selection_file_name, std::ios::in);

          if (!selectionfile.good()){
              std::cerr << "Error loading custom filter file " << selection_file_name << "!" << std::endl;
              std::cerr << "Data will NOT be filtered!" << std::endl;
              customFilterActive = false;
          }
          else {
              std::string line;
              std::string custFilt;
              while (std::getline(selectionfile, line)){
                  if (line.find("#") == 0) continue;
                  custFilt = custFilt.append(line);
                  custFilt = custFilt.append("/");
              }
              if (custFilt.length() > 0) {
                  // last '/'
                  customFilter = custFilt.substr(0, custFilt.length() - 1);
              }
          }
          selectionfile.close();
      }
  }
  else {
      // give a warning if custom filter has been inproperly specified
      if (customFilter.length() > 0){
          std::cerr << "Custom filter: specifying string has not been set properly, data will NOT be filtered." << std::endl;
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
  
  // join all scans then call surface reconstrucion
  // ---
  if (join) {
    unsigned int types = PointType::USE_NONE;
    if(supportsReflectance(iotype)) types |= PointType::USE_REFLECTANCE;
    if(supportsColor(iotype)) types |= PointType::USE_COLOR;

    // if specified, filter scans
    for (size_t i = 0; i < Scan::allScans.size(); i++)  {
      if(rangeFilterActive) Scan::allScans[i]->setRangeFilter(max_dist, min_dist);
      if(customFilterActive) Scan::allScans[i]->setCustomFilter(customFilter);
    }
    
    int end_reduction = (int)Scan::allScans.size();
    #ifdef _OPENMP
    #pragma omp parallel for schedule(dynamic)
    #endif
    for (int iterator = 0; iterator < end_reduction; iterator++) {
      if (red > 0) {
        PointType pointtype(types);
        std::cout << "Reducing Scan No. " << iterator << std::endl;
        Scan::allScans[iterator]->setReductionParameter(red, octree, pointtype);
        Scan::allScans[iterator]->calcReducedPoints();
      } else {
        std::cout << "Copying Scan No. " << iterator << std::endl;
      }
      // reduction filter for current scan!
    }

    readFrames(dir, start, end, frame, uP);

    vector<Point> pts;
    vector<Point> norms;
    for(unsigned int i = 0; i < Scan::allScans.size(); i++) {
      Scan *source = Scan::allScans[i];
      std::string red_string = red > 0 ? " reduced" : "";

      // apply optional filtering
      source->setRangeFilter(max_dist, min_dist);

      const double* rPos = source->get_rPos();
      const double* rPosTheta = source->get_rPosTheta();

      DataXYZ xyz  = source->get("xyz");
      // record UOS format data
      scaleFac = 1.0;
      for(unsigned int j = 0; j < xyz.size(); j++) {
        pts.push_back(Point(scaleFac * xyz[j][0], scaleFac * xyz[j][1], scaleFac * xyz[j][2]));
      }
      
      // calculate normals for current scan, then merge them
      calcNormals(pts, norms, ntype, k1, k2, width, height, rPos, rPosTheta);
      points.insert(points.end(), pts.begin(), pts.end());
      normals.insert(normals.end(), norms.begin(), norms.end());
      pts.clear();
      norms.clear();
    }
    cout << "Poisson reconstruction started" << endl;

    // reconstruction for current scan
    pp.Depth = depth;
    pp.SolverDivide = solverDivide;
    pp.Depth = depth;
    pp.Offset = offset;
    poisson.setVertices(points);
    poisson.setNormals(normals);
    poisson.setParams(pp);
    poisson.apply();
    // CoredVectorMeshData m;
    // poisson.getMesh(&m);
    poisson.exportMesh((odir + "all.obj").c_str());
    cout << "Poisson reconstruction end, model generated at: " +  odir + "all.obj" << endl;
  }
  // apply surface reconstruction to each scan individually
  // ---
  else {
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

      // calculate normals
      calcNormals(points, normals, ntype, k1, k2, width, height, rPos, rPosTheta);

      cout << "Normal calculation end" << endl;
      cout << "Poisson reconstruction started" << endl;

      // reconstruction for current scan
      pp.Depth = depth;
      pp.SolverDivide = solverDivide;
      pp.Depth = depth;
      pp.Offset = offset;
      poisson.setVertices(points);
      poisson.setNormals(normals);
      poisson.setParams(pp);
      poisson.apply();
      // CoredVectorMeshData m;
      // poisson.getMesh(&m);
      poisson.exportMesh((odir + to_string(scanNumber) + ".obj").c_str());
      cout << "Poisson reconstruction end, model generated at: " +  odir + to_string(scanNumber) + ".obj" << endl;

      // clear points and normal of previous scan
      points.clear();
      normals.clear();
      scanNumber++;
    }
  }
  
  // shutdown everything
  if (scanserver)
    ClientInterface::destroy();

  Scan::closeDirectory();

  // post-processing of reconstructed surface
  poisson.testVcgFilter();

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
void parse_options(int argc, char **argv, int &start, int &end, bool &scanserver, int &max_dist, int &min_dist, string &dir, string &odir, IOType &iotype, 
  bool &join, double &red, int &rand, bool &use_pose, bool &use_xyz, bool &use_color, bool &use_reflectance, int &octree, bool &rangeFilterActive, bool &customFilterActive, string &customFilter, double &scaleFac, bool &hexfloat, bool &high_precision, int &frame,
  int &k1, int &k2, normal_method &ntype, int &width, int &height,
  int &depth, int &solverDivide, float &samplesPerNode, float &offset)
{
  po::options_description cmd_options("Poisson Surface Reconstruction <options> \n"
                                      "Options list:");
  cmd_options.add_options()
      // IO parameters
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
      // Join scans parameters
      ("join,j", po::value<bool>(&join)->default_value(false),
      "whether to join scans together for surface reconstruction")
      ("customFilter,u", po::value<string>(&customFilter),
      "Apply a custom filter. Filter mode and data are specified as a "
      "semicolon-seperated string:"
      "{filterMode};{nrOfParams}[;param1][;param2][...]"
      "Multiple filters can be specified in a file (syntax in file is same as"
      "direct specification"
      "FILE;{fileName}"
      "See filter implementation in src/slam6d/pointfilter.cc for more detail.")
      ("reduce,r", po::value<double>(&red)->default_value(-1.0),
      "turns on octree based point reduction (voxel size=<NR>)")
      ("octree,O", po::value<int>(&octree)->default_value(1),
      "use randomized octree based point reduction (pts per voxel=<NR>)")
      ("scale,y", po::value<double>(&scaleFac)->default_value(0.01),
      "scale factor for point cloud in m (be aware of the different units for uos (cm) and xyz (m), default: 0.01 means that input and output remain the same)'")
      // FIXME: for bool values, I cannot use po::bool_switch here, use po::value<bool> as a workaround
      // xiasun 
      ("color,c", po::value<bool>(&use_color)->default_value(false),
      "export in color as RGB")
      ("reflectance,R", po::value<bool>(&use_reflectance)->default_value(false),
      "end after scan <arg>")
      ("trustpose,t", po::value<bool>(&use_pose)->default_value(false),
      "Trust the pose file, do not extrapolate the last transformation."
      "(just for testing purposes, or gps input.)")
      ("xyz,x", po::value<bool>(&use_xyz)->default_value(false),
      "export in xyz format (right handed coordinate system in m)")
      ("hexfloat,0", po::value<bool>(&hexfloat)->default_value(false),
      "export points with hexadecimal digits")
      ("highprecision,H", po::value<bool>(&high_precision)->default_value(false),
      "export points with full double precision")
      ("frame,n", po::value<int>(&frame)->default_value(-1),
      "uses frame NR for export")
      // Normal parameters 
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
      // Poisson parameters
      ("depth,d",
      po::value<int>(&depth)->default_value(8),
      "value of poisson octree depth")
      ("divide,D",
      po::value<int>(&solverDivide)->default_value(8),
      "value of poisson solver divider")
      ("samples,P",
      po::value<float>(&samplesPerNode)->default_value(1.0f),
      "value of poisson samples per node")
      ("offset,o",
      po::value<float>(&offset)->default_value(1.0f),
      "value of poisson offset");

  // input scan dir and output obj dir are hidden and mandatory
  po::options_description hidden("Hidden options");
  hidden.add_options()
      ("input-dir", po::value<string>(&dir), "input dir")
      ("output-dir", po::value<string>(&odir), "output dir");

  po::positional_options_description pd;
  pd.add("input-dir", 1).add("output-dir", 1);

  po::options_description all;
  all.add(cmd_options).add(hidden);

  po::variables_map vmap;
  po::store(po::command_line_parser(argc, argv).
            options(all).positional(pd).run(), vmap);
  po::notify(vmap);

  if (vmap.count("help")) {
    cout << cmd_options << endl << endl;
    cout << "Sample command for reconstructing surface:" << endl;
    cout << "- e.g. Join scans and trust pose with octree depth 12:" << endl;
    cout << "  bin/poisson dat dat/test/join_output -j true -t true -d 12" << endl;
    cout << endl;
    exit(-1);
  }

  // read scan path
  if (dir[dir.length()-1] != '/') dir = dir + "/";

}

void readFrames(std::string dir, int start, int end, int frame, bool use_pose=false)
{
  std::ifstream frame_in;
  int  fileCounter = start;
  std::string frameFileName;
  if((int)(start + Scan::allScans.size() - 1) > end) end = start + Scan::allScans.size() - 1;
  for (;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read
    
    frameFileName = dir + "scan" + to_string(fileCounter++,3) + ".frames";
    if(!use_pose) {

      frame_in.open(frameFileName.c_str());

      // read 3D scan
      if (!frame_in.good()) break; // no more files in the directory

      std::cout << "Reading Frames for 3D Scan " << frameFileName << "..." << std::endl;

      double transMat[16];
      int algoTypeInt;

      int frameCounter = 0;
      while (frame_in.good()) {
        if (frame != -1 && frameCounter > frame) break;
        frameCounter++;
        try {
          frame_in >> transMat >> algoTypeInt;
        }
        catch (const std::exception &e) {   
          break;
        }
      }

      // calculate RELATIVE transformation
      const double * transMatOrig = Scan::allScans[fileCounter - start - 1]->get_transMatOrg();
      double tinv[16];
      M4inv(transMatOrig, tinv);

      double tfin[16];
      MMult(transMat, tinv, tfin);
      //Scan::allScans[fileCounter - start - 1]->transformMatrix(tfin);
      //Scan::allScans[fileCounter - start - 1]->transformMatrix(tinv);
      // save final pose in scan
      Scan::allScans[fileCounter - start - 1]->transformMatrix(tfin);
      
      Scan::allScans[fileCounter - start - 1]->transformAll(transMat);
    
    } else {
      const double * transMatOrig = Scan::allScans[fileCounter - start - 1]->get_transMatOrg();
      Scan::allScans[fileCounter - start - 1]->transformAll(transMatOrig);
    }
    frame_in.close();
    frame_in.clear();
  }
}

void calcNormals(vector<Point> &points, vector<Point> &normals, normal_method ntype, int k1, int k2, int width, int height, const double* rPos, const double* rPosTheta) {
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

int Poisson::setParams(PoissonParam &p) {
  params = p;
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

  mesh.resetIterator(); // reset iterator
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

int Poisson::testVcgFilter() {
  MyMesh m;
  
  mesh.resetIterator(); // reset iterator
  Point3D<float> vertex;
  TriangleIndex tIndex;
  int inCoreFlag;
  int numVerticesInCore = mesh.inCorePoints.size(),
    numVerticesOutCore = mesh.oocPoints.size(),
    numFaces=mesh.triangleCount();

  // get and write correct scaled vertices coordinates
  for (int i = 0; i < numVerticesInCore; ++i) {
    vertex = mesh.inCorePoints[i];
    vcg::tri::Allocator<MyMesh>::AddVertex(m,MyMesh::CoordType(
        vertex.coords[0] * scale + center.coords[0], 
        vertex.coords[1] * scale + center.coords[1], 
        vertex.coords[2] * scale + center.coords[2]));
  }
  for (int i = 0; i < numVerticesOutCore; ++i) {
    vertex = mesh.oocPoints[i];
    vcg::tri::Allocator<MyMesh>::AddVertex(m,MyMesh::CoordType(
        vertex.coords[0] * scale + center.coords[0], 
        vertex.coords[1] * scale + center.coords[1], 
        vertex.coords[2] * scale + center.coords[2]));
  }

  // get and write correct ordered faces indexes
  for (int i = 0; i < numFaces; ++i) {
    mesh.nextTriangle(tIndex, inCoreFlag);
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[0])) {
      tIndex.idx[0] += int(mesh.inCorePoints.size());
    }
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[1])) {
      tIndex.idx[1] += int(mesh.inCorePoints.size());
    }
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[2])) {
      tIndex.idx[2] += int(mesh.inCorePoints.size());
    }
    vcg::tri::Allocator<MyMesh>::AddFace(m, tIndex.idx[0], tIndex.idx[1], tIndex.idx[2]); // vertice index start from 0 here
  }
  
  tri::UpdateTopology<MyMesh>::VertexFace(m);
  // test: laplacian smoothing for three iterations
  for(int i=0; i < 3; ++i)
  {
    tri::UpdateNormal<MyMesh>::PerFaceNormalized(m);
    tri::Smooth<MyMesh>::VertexCoordPasoDoble(m, 1, 0, 50, false);
  }
  tri::io::ExporterPLY<MyMesh>::Save(m, "out_s.ply");

  return 1;
}

int Poisson::ready() {
  return vertices.size() > 0 && normals.size() > 0 && vertices.size() == normals.size() && reconstructed;
}

int Poisson::apply() {
  reconstructed = Execute2(params, vertices, normals, mesh, center, scale, cb);
  return reconstructed;
}
