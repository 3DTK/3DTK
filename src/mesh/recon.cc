#include "mesh/recon_commons.h"

using namespace std;

// comment `__attribute__((optimize(0)))` to allow compiling optimization
// e.g. int /*__attribute__((optimize(0)))*/ main(int argc, char **argv) 
int __attribute__((optimize(0))) main(int argc, char **argv) 
{
  // parameters for io
  int start, end;
  bool scanserver;
  int max_dist, min_dist;
  string dir, odir; // directory of input scan and output model
  IOType iotype;
  bool in_color; // input points with color
  bool out_normal; // output points with normal

  // parameters for transfromation and filtering
  bool join;
  int rand;
  bool use_pose;  // should we use the pose information instead of the frames?? 
  bool use_xyz;
  bool use_color;
  bool use_reflectance;
  bool rangeFilterActive;
  bool customFilterActive = false;
  std::string customFilter;
  double scaleFac;
  bool hexfloat;
  bool high_precision;
  int frame;

  // parameters for reduction
  bool autoRed;
  double red;
  int octree;  // employ randomized octree reduction?

  // parameters for normal calculation
  int k1, k2;
  normal_method ntype;
  int width, height;
  bool inward;

  // parameters for poisson
  int depth;
	int solverDivide;
	float samplesPerNode;
  float offset;
  Poisson poisson;
  PoissonParam pp;
  float trimVal = 0.0;

  vector<Point> points;
  vector<Point> normals;
  vector<vector<float>> colors;

  std::string red_string = red > 0 ? " reduced" : "";
  
  // parse input arguments
  parse_options(argc, argv, start, end, 
    scanserver, max_dist, min_dist,
    dir, odir, iotype, 
    in_color, out_normal, join, 
    red, rand, use_pose, use_xyz, use_color, use_reflectance, 
    octree, rangeFilterActive, customFilterActive, customFilter, 
    scaleFac, hexfloat, high_precision, frame, autoRed,
    k1, k2, ntype, width, height, inward, 
    depth, solverDivide, samplesPerNode, offset, trimVal);

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
  if (pos != std::string::npos) {
      customFilterActive = true;

      // check if customFilter is specified in file
      if (customFilter.find("FILE;") == 0) {
          std::string selection_file_name = customFilter.substr(5, customFilter.length());
          std::ifstream selectionfile;
          // open the input file
          selectionfile.open(selection_file_name, std::ios::in);

          if (!selectionfile.good()) {
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
      if (customFilter.length() > 0) {
          std::cerr << "Custom filter: specifying string has not been set properly, data will NOT be filtered." << std::endl;
      }
  }

  // Read the scans
  Scan::openDirectory(scanserver, dir, iotype, start, end);
  if(Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);
  }
  // Apply transformation of scans, with frames or with poses
  readFrames(dir, start, end, frame, use_pose);

  std::vector<Scan*>::iterator it = Scan::allScans.begin();
  int scanNumber = start;
  
  // join all scans then call surface reconstrucion
  // ---
  if (join) {
    // calculate appropriate reduction parameters
    if (red < 0 && autoRed) {
      RedParam rp; 
      getRedParam(rp);
      red = rp.voxelSize;
      octree = rp.ptsPerVerxel;
    }
    
    // octree based reduction
    unsigned int types = PointType::USE_NONE;
    if(supportsReflectance(iotype)) types |= PointType::USE_REFLECTANCE;
    if(supportsColor(iotype)) types |= PointType::USE_COLOR;

    // if specified, filter scans
    for (size_t i = 0; i < Scan::allScans.size(); i++) {
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

    vector<Point> pts;
    vector<Point> norms;
    for(unsigned int i = 0; i < Scan::allScans.size(); i++) {
      Scan *scan = Scan::allScans[i];

      // apply optional filtering
      if(rangeFilterActive) scan->setRangeFilter(max_dist, min_dist);
      if(customFilterActive) scan->setCustomFilter(customFilter);

      const double* rPos = scan->get_rPos();
      const double* rPosTheta = scan->get_rPosTheta();

      DataXYZ xyz = scan->get("xyz" + red_string);
      DataRGB rgb = scan->get(red_string.size() == 0 ? "rgb" : "color" + red_string);
      // record UOS format data
      scaleFac = 1.0;
      for(unsigned int j = 0; j < xyz.size(); j++) {
        pts.push_back(Point(scaleFac * xyz[j][0], scaleFac * xyz[j][1], scaleFac * xyz[j][2]));
        vector<float> c;
        if (in_color) {
          c = {(float)rgb[j][0], (float)rgb[j][1], (float)rgb[j][2]};
        }
        else {
          c = {255.0, 255.0, 255.0};
        }
        colors.push_back(c);
      }
      
      // calculate normals for current scan, then merge them
      calcNormals(pts, norms, ntype, k1, k2, width, height, rPos, rPosTheta, scan);
      if (inward) {
        flipNormals(norms);
      }
      points.insert(points.end(), pts.begin(), pts.end());
      normals.insert(normals.end(), norms.begin(), norms.end());
      pts.clear();
      norms.clear();
    }

    // data conversion
    vector<vector<float>> vPoints;
    vector<vector<float>> vNormals;
    convert(points, vPoints);   vector<Point>().swap(points);
    convert(normals, vNormals); vector<Point>().swap(normals);

    cout << "Poisson reconstruction started" << endl;
    // reconstruction for joined scan
    pp.Depth = depth;
    pp.Trim = trimVal;
    pp.UseColor = in_color;
    pp.ExportNormal = out_normal;
    poisson.setPoints(vPoints);   vector<vector<float>>().swap(vPoints); 
    poisson.setNormals(vNormals); vector<vector<float>>().swap(vNormals); 
    poisson.setColors(colors);
    poisson.setParams(pp);
    poisson.apply();
    poisson.exportMesh((odir + "_all.obj").c_str());
    poisson.exportTrimmedMesh((odir + "_all_trimmed.obj").c_str());
    cout << "Poisson reconstruction end, model generated at: " +  odir + "_all & all_trimmed.obj" << endl;
  }
  // apply surface reconstruction to each scan individually
  // ---
  else {
    for(unsigned int i = 0; i < Scan::allScans.size(); ++i) {
      Scan* scan = Scan::allScans[i];

      // get reduction parameters
      if (red && autoRed) {
        RedParam rp;
        getRedParam(rp, scan);
        red = rp.voxelSize;
        octree = rp.ptsPerVerxel;
      }

      // octree based reduction
      unsigned int types = PointType::USE_NONE;
      if(supportsReflectance(iotype)) types |= PointType::USE_REFLECTANCE;
      if(supportsColor(iotype)) types |= PointType::USE_COLOR;
      if (red > 0) {
        PointType pointtype(types);
        std::cout << "Reducing Scan No. " << i << std::endl;
        scan->setReductionParameter(red, octree, pointtype);
        scan->calcReducedPoints();
      } else {
        std::cout << "Copying Scan No. " << i << std::endl;
      }

      // apply optional filtering
      if(rangeFilterActive) scan->setRangeFilter(max_dist, min_dist);
      if(customFilterActive) scan->setCustomFilter(customFilter);

      const double* rPos = scan->get_rPos();
      const double* rPosTheta = scan->get_rPosTheta();

      // read scan into points
      DataXYZ xyz(scan->get("xyz" + red_string));
      DataRGB rgb = scan->get(red_string.size() == 0 ? "rgb" : "color" + red_string);
      points.reserve(xyz.size());
      normals.reserve(xyz.size());
      colors.reserve(xyz.size());

      for(unsigned int j = 0; j < xyz.size(); j++) {
        points.push_back(Point(xyz[j][0], xyz[j][1], xyz[j][2]));
        vector<float> c;
        if (in_color) {
          c = {(float)rgb[j][0], (float)rgb[j][1], (float)rgb[j][2]};
        }
        else {
          c = {255.0, 255.0, 255.0};
        }
        colors.push_back(c);
      }

      // calculate normals
      calcNormals(points, normals, ntype, k1, k2, width, height, rPos, rPosTheta, scan);
      if (inward) {
        flipNormals(normals);
      }
      cout << "Normal calculation end" << endl;

      // data conversion
      vector<vector<float>> vPoints;
      vector<vector<float>> vNormals;
      convert(points, vPoints);   vector<Point>().swap(points);
      convert(normals, vNormals); vector<Point>().swap(normals);

      cout << "Poisson reconstruction started" << endl;
      // reconstruction for current scan
      pp.Depth = depth;
      pp.Trim = trimVal;
      pp.UseColor = in_color;
      pp.ExportNormal = out_normal;
      poisson.setPoints(vPoints);   vector<vector<float>>().swap(vPoints); 
      poisson.setNormals(vNormals); vector<vector<float>>().swap(vNormals); 
      poisson.setColors(colors);
      poisson.setParams(pp);
      poisson.apply();
      poisson.exportMesh((odir + to_string(scanNumber) + ".obj").c_str());
      poisson.exportTrimmedMesh((odir + to_string(scanNumber) + "_trimmed.obj").c_str());
      cout << "Poisson reconstruction end, model generated at: " +  odir + to_string(scanNumber) + " & _trimmed.obj" << endl;

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

  return 0;
}
