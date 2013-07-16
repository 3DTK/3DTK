/*
 * basicScan implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/basicScan.h"

#include "scanio/scan_io.h"
#include "slam6d/kd.h"
#include "slam6d/Boctree.h"
#include "slam6d/ann_kd.h"

#ifdef WITH_METRICS
#include "slam6d/metrics.h"
#endif //WITH_METRICS

#include <list>
#include <utility>
#include <fstream>
using std::ifstream;
using std::ofstream;
using std::flush;
using std::string;
using std::map;
using std::pair;
using std::vector;

#include <boost/filesystem/operations.hpp>
using namespace boost::filesystem;



void BasicScan::openDirectory(const std::string& path,
                              IOType type,
                              int start,
                              int end)
{
#ifdef WITH_METRICS
  Timer t = ClientMetric::read_scan_time.start();
#endif //WITH_METRICS

  // create an instance of ScanIO
  ScanIO* sio = ScanIO::getScanIO(type);

  // query available scans in the directory from the ScanIO
  std::list<std::string> identifiers(sio->readDirectory(path.c_str(),
                                                        start,
                                                        end));

  Scan::allScans.reserve(identifiers.size());

  // for each identifier, create a scan
  for(std::list<std::string>::iterator it = identifiers.begin();
      it != identifiers.end();
      ++it) {
    Scan::allScans.push_back(new BasicScan(path, *it, type));
  }

#ifdef WITH_METRICS
  ClientMetric::read_scan_time.end(t);
#endif //WITH_METRICS
}

void BasicScan::closeDirectory()
{
  // clean up the scan vector
  for(ScanVector::iterator it = Scan::allScans.begin(); 
      it != Scan::allScans.end(); 
      ++it) {
    delete *it; 
    *it = 0;
  }
  Scan::allScans.clear();
  ScanIO::clearScanIOs();
}

BasicScan::BasicScan(double *_rPos,
                     double *_rPosTheta,
                     vector<double*> points)
{
  init();
  for(int i = 0; i < 3; i++) {
    rPos[i] = _rPos[i];
    rPosTheta[i] = _rPosTheta[i];
  }
  // write original pose matrix
  EulerToMatrix4(rPos, rPosTheta, transMatOrg);

  // initialize transform matrices from the original one, 
  // could just copy transMatOrg to transMat instead
  transformMatrix(transMatOrg);

  // reset the delta align matrix to represent only the transformations 
  // after local-to-global (transMatOrg) one
  M4identity(dalignxf);
  PointFilter filter;
  if(m_filter_range_set)
    filter.setRange(m_filter_max, m_filter_min);
  if(m_filter_height_set)
    filter.setHeight(m_filter_top, m_filter_bottom);
  if(m_range_mutation_set)
    filter.setRangeMutator(m_range_mutation);
    
  double* data = reinterpret_cast<double*>(create("xyz", 
                     sizeof(double) * 3 * points.size()).get_raw_pointer());
  int tmp = 0;
  for(unsigned int i = 0; i < points.size(); ++i) {
    for(unsigned int j = 0; j < 3; j++) {
      data[tmp++] = points[i][j];
    }
  }
}

BasicScan::BasicScan(const std::string& path, 
                     const std::string& identifier, 
                     IOType type) :
  m_path(path), m_identifier(identifier), m_type(type)
{
  init();

  // request pose from file
  double euler[6];
  ScanIO* sio = ScanIO::getScanIO(m_type);
  sio->readPose(m_path.c_str(), m_identifier.c_str(), euler);
  rPos[0] = euler[0];
  rPos[1] = euler[1];
  rPos[2] = euler[2];
  rPosTheta[0] = euler[3];
  rPosTheta[1] = euler[4];
  rPosTheta[2] = euler[5];

  // write original pose matrix
  EulerToMatrix4(euler, &euler[3], transMatOrg);

  // initialize transform matrices from the original one, 
  // could just copy transMatOrg to transMat instead
  transformMatrix(transMatOrg);

  // reset the delta align matrix to represent only the transformations
  // after local-to-global (transMatOrg) one
  M4identity(dalignxf);
}

BasicScan::~BasicScan()
{
  for (map<string, pair<unsigned char*, 
         unsigned int>>::iterator it = m_data.begin(); 
       it != m_data.end(); 
       it++) {
    delete [] it->second.first;
  }
}

void BasicScan::init()
{
  m_filter_max = 0.0;
  m_filter_min = 0.0;
  m_filter_top = 0.0;
  m_filter_bottom = 0.0;
  m_range_mutation = 0.0;
  m_filter_range_set = false;
  m_filter_height_set = false;
  m_range_mutation_set = false;
}


void BasicScan::setRangeFilter(double max, double min)
{
  m_filter_max = max;
  m_filter_min = min;
  m_filter_range_set = true;
}

void BasicScan::setHeightFilter(double top, double bottom)
{
  m_filter_top = top;
  m_filter_bottom = bottom;
  m_filter_height_set = true;
}

void BasicScan::setRangeMutation(double range)
{
  m_range_mutation_set = true;
  m_range_mutation = range;
}

void BasicScan::get(unsigned int types)
{
  ScanIO* sio = ScanIO::getScanIO(m_type);

  vector<double> xyz;
  vector<unsigned char> rgb;
  vector<float> reflectance;
  vector<float> temperature;
  vector<float> amplitude;
  vector<int> type;
  vector<float> deviation;

  PointFilter filter;
  if(m_filter_range_set)
    filter.setRange(m_filter_max, m_filter_min);
  if(m_filter_height_set)
    filter.setHeight(m_filter_top, m_filter_bottom);
  if(m_range_mutation_set)
    filter.setRangeMutator(m_range_mutation);

  sio->readScan(m_path.c_str(),
                m_identifier.c_str(),
                filter,
                &xyz,
                &rgb,
                &reflectance,
                &temperature,
                &amplitude,
                &type,
                &deviation);

  // for each requested and filled data vector,
  // allocate and write contents to their new data fields
  if(types & DATA_XYZ && !xyz.empty()) {
    double* data = reinterpret_cast<double*>(create("xyz",
                      sizeof(double) * xyz.size()).get_raw_pointer());
    for(unsigned int i = 0; i < xyz.size(); ++i) data[i] = xyz[i];
  }
  if(types & DATA_RGB && !rgb.empty()) {
    unsigned char* data = reinterpret_cast<unsigned char*>(create("rgb",
                      sizeof(unsigned char) * rgb.size()).get_raw_pointer());
    for(unsigned int i = 0; i < rgb.size(); ++i)
      data[i] = rgb[i];
  }
  if(types & DATA_REFLECTANCE && !reflectance.empty()) {
    float* data = reinterpret_cast<float*>(create("reflectance",
                      sizeof(float) * reflectance.size()).get_raw_pointer());
    for(unsigned int i = 0; i < reflectance.size(); ++i)
      data[i] = reflectance[i];
  }
  if(types & DATA_TEMPERATURE && !temperature.empty()) {
    float* data = reinterpret_cast<float*>(create("temperature",
                      sizeof(float) * temperature.size()).get_raw_pointer());
    for(unsigned int i = 0; i < temperature.size(); ++i)
      data[i] = temperature[i];
  }
  if(types & DATA_AMPLITUDE && !amplitude.empty()) {
    float* data = reinterpret_cast<float*>(create("amplitude",
                      sizeof(float) * amplitude.size()).get_raw_pointer());
    for(unsigned int i = 0; i < amplitude.size(); ++i) data[i] = amplitude[i];
  }
  if(types & DATA_TYPE && !type.empty()) {
    int* data = reinterpret_cast<int*>(create("type",
                      sizeof(int) * type.size()).get_raw_pointer());
    for(unsigned int i = 0; i < type.size(); ++i) data[i] = type[i];
  }
  if(types & DATA_DEVIATION && !deviation.empty()) {
    float* data = reinterpret_cast<float*>(create("deviation",
                      sizeof(float) * deviation.size()).get_raw_pointer());
    for(unsigned int i = 0; i < deviation.size(); ++i) data[i] = deviation[i];
  }
}

DataPointer BasicScan::get(const std::string& identifier)
{
  // try to get data
  map<string, pair<unsigned char*, unsigned int>>::iterator
    it = m_data.find(identifier);

  // create data fields
  if (it == m_data.end()) {
    // load from file
    if (identifier == "xyz") get(DATA_XYZ);
    else
      if (identifier == "rgb")
        get(DATA_RGB);
      else
      if (identifier == "reflectance")
        get(DATA_REFLECTANCE);
      else
        if (identifier == "temperature")
          get(DATA_TEMPERATURE);
        else
          if (identifier == "amplitude")
            get(DATA_AMPLITUDE);
          else
            if (identifier == "type")
              get(DATA_TYPE);
            else
              if (identifier == "deviation")
                get(DATA_DEVIATION);
              else
                // normals on demand
                if (identifier == "normal")
                  calcNormalsOnDemand();
                else
                  // reduce on demand
                  if (identifier == "xyz reduced")
                    calcReducedOnDemand();
                  else
                    if (identifier == "xyz reduced original")
                      calcReducedOnDemand();
                    else
                      // show requests reduced points
                      // manipulate in showing the same entry
                      if (identifier == "xyz reduced show") {
                        calcReducedOnDemand();
                        m_data["xyz reduced show"] = m_data["xyz reduced"];
                      } else
                        if(identifier == "octtree") {
                          createOcttree();
                        }
    it = m_data.find(identifier);
  }

  // if nothing can be loaded, return an empty pointer
  if(it == m_data.end())
    return DataPointer(0, 0);
  else
    return DataPointer(it->second.first, it->second.second);
}

DataPointer BasicScan::create(const std::string& identifier,
                              unsigned int size)
{
  map<string, pair<unsigned char*, unsigned int>>::iterator
    it = m_data.find(identifier);
  if(it != m_data.end()) {
    // try to reuse, otherwise reallocate
    if(it->second.second != size) {
      delete[] it->second.first;
      it->second.first = new unsigned char[size];
      it->second.second = size;
    }
  } else {
    // create a new block of data
    it = m_data.insert(std::make_pair(identifier,
                                      std::make_pair(new unsigned char[size],
                                                     size))).first;
  }
  return DataPointer(it->second.first, it->second.second);
}

void BasicScan::clear(const std::string& identifier)
{
  map<string, pair<unsigned char*, unsigned int>>::iterator
    it = m_data.find(identifier);
  if(it != m_data.end()) {
    delete[] it->second.first;
    m_data.erase(it);
  }
}


void BasicScan::createSearchTreePrivate()
{
  DataXYZ xyz_orig(get("xyz reduced original"));
  PointerArray<double> ar(xyz_orig);
  switch(searchtree_nnstype)
    {
    case simpleKD:
      kd = new KDtree(ar.get(), xyz_orig.size());
      break;
    case ANNTree:
      kd = new ANNtree(ar, xyz_orig.size());
      break;
    case BOCTree:
      kd = new BOctTree<double>(ar.get(),
                                xyz_orig.size(),
                                10.0,
                                PointType(), true);
      break;
    case -1:
      throw runtime_error("Cannot create a SearchTree without setting a type.");
    default:
      throw runtime_error("SearchTree type not implemented");
    }
}

void BasicScan::calcReducedOnDemandPrivate()
{
  // create reduced points and transform to initial position,
  // save a copy of this for SearchTree
  calcReducedPoints();
  transformReduced(transMatOrg);
  copyReducedToOriginal();
}

void BasicScan::calcNormalsOnDemandPrivate()
{
  // create normals
  calcNormals();
}


void BasicScan::createOcttree()
{
  string scanFileName = m_path + "scan" + m_identifier + ".oct";
  BOctTree<float>* btree = 0;

  // try to load from file, if successful return
  if (octtree_loadOct && exists(scanFileName)) {
    btree = new BOctTree<float>(scanFileName);
    m_data.insert(std::make_pair("octtree",
             std::make_pair(reinterpret_cast<unsigned char*>(btree),
             0 // or memorySize()?
             )));
    return;
  }

  // create octtree from scan
  if (octtree_reduction_voxelSize > 0) { // with reduction, only xyz points
    DataXYZ xyz_r(get("xyz reduced show"));
    btree = new BOctTree<float>(PointerArray<double>(xyz_r).get(),
                                xyz_r.size(),
                                octtree_voxelSize,
                                octtree_pointtype,
                                true);
  } else { // without reduction, xyz + attribute points
    float** pts = octtree_pointtype.createPointArray<float>(this);
    unsigned int nrpts = size<DataXYZ>("xyz");
    btree = new BOctTree<float>(pts,
                                nrpts,
                                octtree_voxelSize,
                                octtree_pointtype,
                                true);
    for(unsigned int i = 0; i < nrpts; ++i) delete[] pts[i]; delete[] pts;
  }

  // save created octtree            
  if(octtree_saveOct) {
    cout << "Saving octree " << scanFileName << endl;
    btree->serialize(scanFileName);
  }

  m_data.insert(std::make_pair("octtree",
           std::make_pair(reinterpret_cast<unsigned char*>(btree),
                          0 // or memorySize()?
                          )));
}

BOctTree<float>* BasicScan::convertScanToShowOcttree()
{
  string scanFileName = m_path + "scan" + m_identifier + ".oct";
  BOctTree<float>* btree = 0;

  // try to load from file, if successful return
  if (octtree_loadOct && exists(scanFileName)) {
    btree = new BOctTree<float>(scanFileName);
    return btree;
  }

  // create octtree from scan
  if (octtree_reduction_voxelSize > 0) { // with reduction, only xyz points
    DataXYZ xyz_r(get("xyz reduced show"));
    btree = new BOctTree<float>(PointerArray<double>(xyz_r).get(),
                                xyz_r.size(),
                                octtree_voxelSize,
                                octtree_pointtype,
                                true);
  } else { // without reduction, xyz + attribute points
    float** pts = octtree_pointtype.createPointArray<float>(this);
    unsigned int nrpts = size<DataXYZ>("xyz");
    btree = new BOctTree<float>(pts,
                                nrpts,
                                octtree_voxelSize,
                                octtree_pointtype,
                                true);
    for(unsigned int i = 0; i < nrpts; ++i) delete[] pts[i]; delete[] pts;
  }

  // save created octtree            
  if(octtree_saveOct) {
    cout << "Saving octree " << scanFileName << endl;
    btree->serialize(scanFileName);
  }
  return btree;
}

unsigned int BasicScan::readFrames()
{
  string filename = m_path + "scan" + m_identifier + ".frames";
  ifstream file(filename.c_str());
  file.exceptions(ifstream::eofbit|ifstream::failbit|ifstream::badbit);
  try {
    double transformation[16];
    unsigned int type;
    do {
      file >> transformation >> type;
      m_frames.push_back(Frame(transformation, type));
    } while(file.good());
  } catch(...) {}

  return m_frames.size();
}

void BasicScan::saveFrames()
{
  string filename = m_path + "scan" + m_identifier + ".frames";
  ofstream file(filename.c_str());
  for(vector<Frame>::iterator it = m_frames.begin();
      it != m_frames.end();
      ++it) {
    file << it->transformation << it->type << '\n';
  }
  file << flush;
  file.close();
}

unsigned int BasicScan::getFrameCount()
{
  return m_frames.size();
}

void BasicScan::getFrame(unsigned int i,
                         const double*& pose_matrix,
                         AlgoType& type)
{
  const Frame& frame(m_frames.at(i));
  pose_matrix = frame.transformation;
  type = static_cast<AlgoType>(frame.type);
}

void BasicScan::addFrame(AlgoType type)
{
  m_frames.push_back(Frame(transMat, type));
}
