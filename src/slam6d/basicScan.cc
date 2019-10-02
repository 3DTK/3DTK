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

#ifdef WITH_MMAP_SCAN
#include <sys/mman.h>
#include <fcntl.h>
#include <cerrno>
#include <cstring>
#endif

#include <boost/filesystem/operations.hpp>
using namespace boost::filesystem;



void BasicScan::openDirectory(const std::string& path,
                              IOType type,
                              int start,
                              int end
#ifdef WITH_MMAP_SCAN
                              , boost::filesystem::path cache
#endif
                              )
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
    Scan::allScans.push_back(new BasicScan(path, *it, type
#ifdef WITH_MMAP_SCAN
                                           , cache
#endif
                                          ));
  }

#ifdef WITH_METRICS
  ClientMetric::read_scan_time.end(t);
#endif //WITH_METRICS
}

void BasicScan::openDirectory(dataset_settings& dss
#ifdef WITH_MMAP_SCAN
  , boost::filesystem::path cache
#endif
)
{
#ifdef WITH_METRICS
  Timer t = ClientMetric::read_scan_time.start();
#endif //WITH_METRICS

  if (dss.data_type == dataset_settings::MULTI_SRCS) {
    std::cout << "Getting data from multiple sources..." << std::endl;
    //int start = dss.scan_numbers.min;
    //int end = dss.scan_numbers.max;
    //if (end == -1) end = dss.subsets.size() - 1;
    //for (int i = start; i <= end; ++i) {
    for (int i = 0; i < dss.subsets.size(); ++i) {
        BasicScan::openDirectory(*(dss.subsets[i]));
    }
  }
  else {
    // create an instance of ScanIO
    ScanIO* sio = ScanIO::getScanIO(dss.format);

    // query available scans in the directory from the ScanIO
    std::list<std::string> identifiers(sio->readDirectory(dss));

    Scan::allScans.reserve(Scan::allScans.size() + identifiers.size());

    // for each identifier, create a scan
    for (std::list<std::string>::iterator it = identifiers.begin();
      it != identifiers.end();
      ++it)
    {
      Scan::allScans.push_back(new BasicScan(dss, *it
#ifdef WITH_MMAP_SCAN
        , cache
#endif
      ));
    }
  }
#ifdef WITH_METRICS
  ClientMetric::read_scan_time.end(t);
#endif //WITH_METRICS
}

void BasicScan::closeDirectory()
{
  // clean up the scan vector

  // avoiding "Expression: vector iterators incompatible" on empty allScans-vector
  if (Scan::allScans.size()){
    for(ScanVector::iterator it = Scan::allScans.begin();
      it != Scan::allScans.end();
      ++it) {
        delete *it;
        *it = 0;
    }
    Scan::allScans.clear();
  }
  ScanIO::clearScanIOs();
}

// TODO check init state
void BasicScan::updateTransform(double *_rPos, double *_rPosTheta)
{
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
  if (m_filter_custom_set)
    filter.setCustom(customFilterStr);
  if(m_range_mutation_set)
    filter.setRangeMutator(m_range_mutation);
  if(m_filter_scale_set)
    filter.setScale(m_filter_scale);

}


// TODO DOKU. Insert points with a DataPointer
// create the DataPointer with the DataPointer
BasicScan::BasicScan(double *_rPos,
                     double *_rPosTheta)
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
  if (m_filter_custom_set)
    filter.setCustom(customFilterStr);
  if(m_range_mutation_set)
    filter.setRangeMutator(m_range_mutation);
  if(m_filter_scale_set)
    filter.setScale(m_filter_scale);
}


BasicScan::BasicScan(double *_rPos,
                     double *_rPosTheta,
                     std::vector<double*> points)
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
  if (m_filter_custom_set)
    filter.setCustom(customFilterStr);
  if(m_range_mutation_set)
    filter.setRangeMutator(m_range_mutation);
  if(m_filter_scale_set)
    filter.setScale(m_filter_scale);

  // check if we can create a large enough array. The maximum size_t on 32 bit
  // is around 4.2 billion which is too little for scans with more than 179
  // million points
  if (sizeof(size_t) == 4 && points.size() > ((size_t)(-1))/sizeof(double)/3) {
      throw std::runtime_error("Insufficient size of size_t datatype");
  }
  double* data = reinterpret_cast<double*>(create("xyz",
                     sizeof(double) * 3 * points.size()).get_raw_pointer());
  int tmp = 0;
  for(size_t i = 0; i < points.size(); ++i) {
    for(size_t j = 0; j < 3; j++) {
      data[tmp++] = points[i][j];
    }
  }
}

BasicScan::BasicScan(const std::string& path,
                     const std::string& identifier,
                     IOType type
#ifdef WITH_MMAP_SCAN
                     , boost::filesystem::path cache
#endif
                     ) :
  m_path(path), m_identifier(identifier), m_type(type)
{
  init();

  int pos = identifier.find_first_of(':');
  std::string first_line_identifier = identifier.substr(0, pos);

  // request pose from file
  double euler[6];
  ScanIO* sio = ScanIO::getScanIO(m_type);
  if (Scan::continue_processing) {
    sio->readPoseFromFrames(m_path.c_str(), first_line_identifier.c_str(), euler);
  }
  else {
    sio->readPose(m_path.c_str(), first_line_identifier.c_str(), euler, &m_timestamp);
  }
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

#ifdef WITH_MMAP_SCAN
  m_mmap_cache = cache;
#endif
}

BasicScan::~BasicScan()
{
  for (std::map<std::string, std::pair<unsigned char*,
         size_t>>::iterator it = m_data.begin();
       it != m_data.end();
       it++) {
#ifdef WITH_MMAP_SCAN
    // depending on whether the data was backed by an mmap-ed file or by
    // memory on the heap, delete the right thing
    std::map<std::string, int>::iterator it2 = m_mmap_fds.find(it->first);
    if (it2 != m_mmap_fds.end()) {
      int ret;
      ret = munmap(it->second.first, it->second.second);
      if (ret != 0) {
        throw std::runtime_error("cannot munmap");
      }
      // since we called unlink() before, this also deletes the file for good
      ret = close(it2->second);
      if (ret != 0) {
        throw std::runtime_error("cannot close");
      }
      m_mmap_fds.erase(it2);
    } else {
#endif
      // otherwise delete allocated memory
      delete[] it->second.first;
#ifdef WITH_MMAP_SCAN
    }
#endif
  }
}

void BasicScan::init()
{
  m_filter_max = 0.0;
  m_filter_min = 0.0;
  m_filter_top = 0.0;
  m_filter_bottom = 0.0;
  m_range_mutation = 0.0;
  m_filter_scale = 0.0;
  m_filter_range_set = false;
  m_filter_height_set = false;
  m_filter_custom_set = false;
  m_range_mutation_set = false;
  m_filter_scale_set = false;
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

void BasicScan::setCustomFilter(std::string& cFiltStr)
{
  customFilterStr = cFiltStr;
  m_filter_custom_set = true;
}

void BasicScan::setRangeMutation(double range)
{
  m_range_mutation_set = true;
  m_range_mutation = range;
}

void BasicScan::setScaleFilter(double scale)
{
  m_filter_scale = scale;
  m_filter_scale_set = true;
}

time_t BasicScan::getLastModified()
{
  ScanIO* sio = ScanIO::getScanIO(m_type);
  return sio->lastModified(m_path.c_str(), m_identifier.c_str());
}

void BasicScan::get(IODataType types)
{
  ScanIO* sio = ScanIO::getScanIO(m_type);

  if (!sio->supports(types)) {
	  return;
  }

  std::vector<double> xyz;
  std::vector<unsigned char> rgb;
  std::vector<float> reflectance;
  std::vector<float> temperature;
  std::vector<float> amplitude;
  std::vector<int> type;
  std::vector<float> deviation;
  std::vector<double> normal;

  PointFilter filter;
  if(m_filter_range_set)
    filter.setRange(m_filter_max, m_filter_min);
  if(m_filter_height_set)
    filter.setHeight(m_filter_top, m_filter_bottom);
  if (m_filter_custom_set)
    filter.setCustom(customFilterStr);
  if(m_range_mutation_set)
    filter.setRangeMutator(m_range_mutation);
  if(m_filter_scale_set)
    filter.setScale(m_filter_scale);

  std::string identifiers = m_identifier;
  std::string current_identifier;
  size_t pos = identifiers.find_first_of(';');
  do {
    current_identifier = identifiers.substr(0, pos);
    if (pos != std::string::npos) identifiers = identifiers.substr(pos + 1);
    else identifiers = "";
  sio->readScan(m_path.c_str(),
      current_identifier.c_str(),
                filter,
                &xyz,
                &rgb,
                &reflectance,
                &temperature,
                &amplitude,
                &type,
                &deviation,
                &normal);
  } while ((pos = identifiers.find_first_of(';')) != std::string::npos || !identifiers.empty() );

  // for each requested and filled data vector,
  // allocate and write contents to their new data fields
  if(types & DATA_XYZ && !xyz.empty()) {
    // check if we can create a large enough array. The maximum size_t on 32 bit
    // is around 4.2 billion which is too little for scans with more than 537
    // million points
    if (sizeof(size_t) == 4 && xyz.size() > ((size_t)(-1))/sizeof(double)) {
            throw std::runtime_error("Insufficient size of size_t datatype");
    }
    double* data = reinterpret_cast<double*>(create("xyz",
                      sizeof(double) * xyz.size()).get_raw_pointer());
    for(size_t i = 0; i < xyz.size(); ++i) data[i] = xyz[i];
  }
  if(types & DATA_RGB && !rgb.empty()) {
    // check if we can create a large enough array. The maximum size_t on 32 bit
    // is around 4.2 billion which is too little for scans with more than 4.2
    // billion points
    if (sizeof(size_t) == 4 && rgb.size() > ((size_t)(-1))/sizeof(unsigned char)) {
            throw std::runtime_error("Insufficient size of size_t datatype");
    }
    unsigned char* data = reinterpret_cast<unsigned char*>(create("rgb",
                      sizeof(unsigned char) * rgb.size()).get_raw_pointer());
    for(size_t i = 0; i < rgb.size(); ++i)
      data[i] = rgb[i];
  }
  if(types & DATA_REFLECTANCE && !reflectance.empty()) {
    // check if we can create a large enough array. The maximum size_t on 32 bit
    // is around 4.2 billion which is too little for scans with more than 1.07
    // billion points
    if (sizeof(size_t) == 4 && reflectance.size() > ((size_t)(-1))/sizeof(float)) {
            throw std::runtime_error("Insufficient size of size_t datatype");
    }
    float* data = reinterpret_cast<float*>(create("reflectance",
                      sizeof(float) * reflectance.size()).get_raw_pointer());
    for(size_t i = 0; i < reflectance.size(); ++i)
      data[i] = reflectance[i];
  }
  if(types & DATA_TEMPERATURE && !temperature.empty()) {
    // check if we can create a large enough array. The maximum size_t on 32 bit
    // is around 4.2 billion which is too little for scans with more than 1.07
    // billion points
    if (sizeof(size_t) == 4 && temperature.size() > ((size_t)(-1))/sizeof(float)) {
            throw std::runtime_error("Insufficient size of size_t datatype");
    }
    float* data = reinterpret_cast<float*>(create("temperature",
                      sizeof(float) * temperature.size()).get_raw_pointer());
    for(size_t i = 0; i < temperature.size(); ++i)
      data[i] = temperature[i];
  }
  if(types & DATA_AMPLITUDE && !amplitude.empty()) {
    // check if we can create a large enough array. The maximum size_t on 32 bit
    // is around 4.2 billion which is too little for scans with more than 1.07
    // billion points
    if (sizeof(size_t) == 4 && amplitude.size() > ((size_t)(-1))/sizeof(float)) {
            throw std::runtime_error("Insufficient size of size_t datatype");
    }
    float* data = reinterpret_cast<float*>(create("amplitude",
                      sizeof(float) * amplitude.size()).get_raw_pointer());
    for(size_t i = 0; i < amplitude.size(); ++i) data[i] = amplitude[i];
  }
  if(types & DATA_TYPE && !type.empty()) {
    // check if we can create a large enough array. The maximum size_t on 32 bit
    // is around 4.2 billion which is too little for scans with more than 1.07
    // billion points
    if (sizeof(size_t) == 4 && type.size() > ((size_t)(-1))/sizeof(float)) {
            throw std::runtime_error("Insufficient size of size_t datatype");
    }
    int* data = reinterpret_cast<int*>(create("type",
                      sizeof(int) * type.size()).get_raw_pointer());
    for(size_t i = 0; i < type.size(); ++i) data[i] = type[i];
  }
  if(types & DATA_DEVIATION && !deviation.empty()) {
    // check if we can create a large enough array. The maximum size_t on 32 bit
    // is around 4.2 billion which is too little for scans with more than 1.07
    // billion points
    if (sizeof(size_t) == 4 && deviation.size() > ((size_t)(-1))/sizeof(float)) {
            throw std::runtime_error("Insufficient size of size_t datatype");
    }
    float* data = reinterpret_cast<float*>(create("deviation",
                      sizeof(float) * deviation.size()).get_raw_pointer());
    for(size_t i = 0; i < deviation.size(); ++i) data[i] = deviation[i];
  }
  if(types & DATA_NORMAL && !normal.empty()) {
    // check if we can create a large enough array. The maximum size_t on 32 bit
    // is around 4.2 billion which is too little for scans with more than 537
    // million points
    if (sizeof(size_t) == 4 && normal.size() > ((size_t)(-1))/sizeof(double)) {
            throw std::runtime_error("Insufficient size of size_t datatype");
    }
    double* data = reinterpret_cast<double*>(create("normal",
                      sizeof(double) * normal.size()).get_raw_pointer());
    for(size_t i = 0; i < normal.size(); ++i) data[i] = normal[i];
  }


}

DataPointer BasicScan::get(const std::string& identifier)
{
  // try to get data
  std::map<std::string, std::pair<unsigned char*, size_t>>::iterator
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
                if (identifier == "normal") {
                  if(supportsNormals(m_type)) {
                    get(DATA_NORMAL);
                  } else {
                    calcNormalsOnDemand();
                  }
                } else
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
                              size_t size)
{
  std::map<std::string, std::pair<unsigned char*, size_t>>::iterator
    it = m_data.find(identifier);

  if(it != m_data.end() && it->second.second != size) {
#ifdef WITH_MMAP_SCAN
    // depending on whether the data was backed by an mmap-ed file or by
    // memory on the heap, delete the right thing
    std::map<std::string, int>::iterator it2 = m_mmap_fds.find(identifier);
    if (it2 != m_mmap_fds.end()) {
      int ret;
      ret = munmap(it->second.first, it->second.second);
      if (ret != 0) {
        throw std::runtime_error("cannot munmap");
      }
      // since we called unlink() before, this also deletes the file for good
      ret = close(it2->second);
      if (ret != 0) {
        throw std::runtime_error("cannot close");
      }
      m_mmap_fds.erase(it2);
    } else {
#endif
      // otherwise delete allocated memory
      delete[] it->second.first;
#ifdef WITH_MMAP_SCAN
    }
#endif
  }

  unsigned char *data;
  if(it == m_data.end() || it->second.second != size) {
    // depending on whether the cache path is empty or not, allocate memory
    // as an mmap-ed file or on the heap
#ifdef WITH_MMAP_SCAN
    if (!m_mmap_cache.empty()) {
      char filename[] = "ppl_XXXXXX";
      int fd = mkstemp(filename);
      if (fd == -1) {
        throw std::runtime_error("cannot create temporary file");
      }
      // by unlinking the file now, we make sure that there are no leftover
      // files even if the process is killed
      unlink(filename);
      int ret = fallocate(fd, 0, 0, size);
      if (ret == -1) {
        throw std::runtime_error("cannot fallocate");
      }
      data = (unsigned char *)mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
      if (data == MAP_FAILED) {
        throw std::runtime_error(std::string("cannot mmap: ")+std::string(std::strerror(errno))+std::string(" ")+std::to_string(errno));
      }
      m_mmap_fds.insert(std::make_pair(identifier, fd));
    } else {
#endif
      data = new unsigned char[size];
#ifdef WITH_MMAP_SCAN
    }
#endif
  }

  if(it == m_data.end()) {
    it = m_data.insert(std::make_pair(identifier,
                                      std::make_pair(data, size))).first;
  } else if (it->second.second != size) {
    it->second.first = data;
    it->second.second = size;
  }
  return DataPointer(it->second.first, it->second.second);
}

void BasicScan::clear(const std::string& identifier)
{
  std::map<std::string, std::pair<unsigned char*, size_t>>::iterator
    it = m_data.find(identifier);
  if(it != m_data.end()) {
#ifdef WITH_MMAP_SCAN
    std::map<std::string, int>::iterator it2 = m_mmap_fds.find(identifier);
    if (it2 != m_mmap_fds.end()) {
      // if the data was backed by an mmap-ed file, close
      int ret;
      ret = munmap(it->second.first, it->second.second);
      if (ret != 0) {
        throw std::runtime_error("cannot munmap");
      }
      // since we called unlink() before, this also deletes the file for good
      ret = close(it2->second);
      if (ret != 0) {
        throw std::runtime_error("cannot close");
      }
      m_mmap_fds.erase(it2);
    } else {
#endif
      // otherwise delete allocated memory
      delete[] it->second.first;
#ifdef WITH_MMAP_SCAN
    }
#endif
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
      kd = new KDtree(ar.get(), xyz_orig.size(), searchtree_bucketsize);
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
      throw std::runtime_error("Cannot create a SearchTree without setting a type.");
    default:
      throw std::runtime_error("SearchTree type not implemented");
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

void BasicScan::saveBOctTree(std::string & filename)
{

  BOctTree<float>* btree = 0;

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
    size_t nrpts = size<DataXYZ>("xyz");
    btree = new BOctTree<float>(pts,
                                nrpts,
                                octtree_voxelSize,
                                octtree_pointtype,
                                true);
    for(size_t i = 0; i < nrpts; ++i) delete[] pts[i]; delete[] pts;
  }

  btree->serialize(filename);

}

void BasicScan::createOcttree()
{
  std::string scanFileName = m_path + "scan" + m_identifier + ".oct";
  BOctTree<float>* btree = 0;
  boost::filesystem::path octpath(scanFileName);

  // try to load from file, if successful return
  // If autoOct is true, then only load the octtree if it is newer than
  // the underlying data.
  if (octtree_loadOct && exists(scanFileName) &&
     (!octtree_autoOct || getLastModified() < boost::filesystem::last_write_time(octpath))) {
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
    size_t nrpts = size<DataXYZ>("xyz");
    btree = new BOctTree<float>(pts,
                                nrpts,
                                octtree_voxelSize,
                                octtree_pointtype,
                                true);
    for(size_t i = 0; i < nrpts; ++i) delete[] pts[i]; delete[] pts;
  }

  // save created octtree
  // If autoOct is true, then only save the octree if it is older than the
  // underlying data
  if(octtree_saveOct &&
      (!octtree_autoOct || !boost::filesystem::exists(scanFileName) ||
        getLastModified() > boost::filesystem::last_write_time(octpath))) {
    std::cout << "Saving octree " << scanFileName << std::endl;
    btree->serialize(scanFileName);
  }

  m_data.insert(std::make_pair("octtree",
           std::make_pair(reinterpret_cast<unsigned char*>(btree),
                          0 // or memorySize()?
                          )));
}

BOctTree<float>* BasicScan::convertScanToShowOcttree()
{
  std::string scanFileName = m_path + "scan" + m_identifier + ".oct";
  BOctTree<float>* btree = 0;
  boost::filesystem::path octpath(scanFileName);

  // try to load from file, if successful return
  // If autoOct is true, then only load the octtree if it is newer than
  // the underlying data.
  if (octtree_loadOct && exists(scanFileName) &&
     (!octtree_autoOct || getLastModified() < boost::filesystem::last_write_time(octpath))) {
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
    size_t nrpts = size<DataXYZ>("xyz");
    btree = new BOctTree<float>(pts,
                                nrpts,
                                octtree_voxelSize,
                                octtree_pointtype,
                                true);
    for(size_t i = 0; i < nrpts; ++i) delete[] pts[i]; delete[] pts;
  }

  // save created octtree
  // If autoOct is true, then only save the octree if it is older than the
  // underlying data
  if(octtree_saveOct &&
      (!octtree_autoOct || !boost::filesystem::exists(scanFileName) ||
        getLastModified() > boost::filesystem::last_write_time(octpath))) {
    btree->serialize(scanFileName);
  }
  return btree;
}

size_t BasicScan::readFrames()
{
  int pos = m_identifier.find_first_of(':');
  std::string first_line_identifier = m_identifier.substr(0, pos);

  std::string filename = m_path + "scan" + first_line_identifier + ".frames";
  std::string line;
  std::ifstream file(filename.c_str());
  // clear frame vector here to allow reloading without (old) duplicates
  m_frames.clear();
  while(getline(file, line)) {
          // ignore empty lines
          if(line.length() == 0) continue;
	  //ignore comment lines starting with #
	  if(line[0]=='#') continue;
	  std::istringstream line_stream(line);
	  double transformation[16];
	  unsigned int type;
	  if (line_stream >> transformation >> type) {
	    m_frames.push_back(Frame(transformation, type));
	  } else {
	    std::string msg("Malformed line in ");
		msg += filename + ": " + line;
	    throw std::runtime_error(msg);
	  }
  }

  return m_frames.size();
}

void BasicScan::saveFrames(bool append)
{
  std::string filename = m_path + "scan" + m_identifier + ".frames";
  std::ios_base::openmode open_mode;

  if(append) open_mode = std::ios_base::app;
  else open_mode = std::ios_base::out;
  std::ofstream file(filename.c_str(), open_mode);
  for(std::vector<Frame>::iterator it = m_frames.begin();
      it != m_frames.end();
      ++it) {
    file << it->transformation << it->type << '\n';
  }
  file << std::flush;
  file.close();
}

size_t BasicScan::getFrameCount()
{
  return m_frames.size();
}

void BasicScan::getFrame(size_t i,
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
