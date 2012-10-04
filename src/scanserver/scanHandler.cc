/*
 * scanHandler implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/scanHandler.h"

#include <iostream>
#include <vector>
#include <stdexcept>
using namespace std;

#include <boost/scoped_ptr.hpp>
using boost::scoped_ptr;

#include "scanserver/cache/cacheManager.h"
#include "scanio/scan_io.h"

#ifdef WITH_METRICS
#include "slam6d/metrics.h"
#endif //WITH_METRICS



bool ScanHandler::binary_caching = false;
std::map<SharedScan*, std::vector<double>* > ScanHandler::m_prefetch_xyz;
std::map<SharedScan*, std::vector<unsigned char>* > ScanHandler::m_prefetch_rgb;
std::map<SharedScan*, std::vector<float>* > ScanHandler::m_prefetch_reflectance;
std::map<SharedScan*, std::vector<float>* > ScanHandler::m_prefetch_temperature;
std::map<SharedScan*, std::vector<float>* > ScanHandler::m_prefetch_amplitude;
std::map<SharedScan*, std::vector<int>* > ScanHandler::m_prefetch_type;
std::map<SharedScan*, std::vector<float>* > ScanHandler::m_prefetch_deviation;



//! Abstract class for merging calls to the main vector
class PrefetchVectorBase {
public:
  virtual bool prefetch() = 0;
  virtual void create() = 0;
  virtual unsigned int size() const = 0;
  virtual void write(void* data_ptr) = 0;
};

/** Class for handling a vector and its special case for prefetching.
 *
 *  Outline: prefetch() main vector, otherwise create() on main+prefetched
 *  vectors and use of ScanIO, cache allocation with size(), write() for main,
 *  save() for prefetched vectors.
 */
template<typename T>
class PrefetchVector : public PrefetchVectorBase {
public:
  PrefetchVector(SharedScan* scan, map<SharedScan*, vector<T>*>& prefetches) :
    m_scan(scan), m_prefetches(&prefetches), m_vector(0)
  {
  }
  
  ~PrefetchVector()
  {
    // remove vectors that are still here (RAII/exceptions)
    if(m_vector)
      delete m_vector;
  }
  
  inline vector<T>* get() const { return m_vector; }
  
  //! If a prefetch is found, take ownership and signal true for a successful prefetch
  virtual bool prefetch()
  {
    // check if a prefetch is available
    typename map<SharedScan*, vector<T>*>::iterator it = m_prefetches->find(m_scan);
    if(it != m_prefetches->end()) {
      // take ownership of the vector and remove it from the map
      m_vector = it->second;
      m_prefetches->erase(it);
      return true;
    }
    return false;
  }
  
  //! Create an empty vector
  virtual void create() {
    if(m_vector == 0) {
      m_vector = new vector<T>;
    }
  }
  
  //! Size of vector contents in bytes
  virtual unsigned int size() const {
    return m_vector->size()*sizeof(T);
  }
  
  //! Write vector contents into the cache object via \a data_ptr and clean up the vector
  virtual void write(void* data_ptr) {
    // write vector contents
    for(unsigned int i = 0; i < m_vector->size(); ++i) {
      reinterpret_cast<T*>(data_ptr)[i] = (*m_vector)[i];
    }
    // remove so it won't get saved for prefetches
    delete m_vector;
    m_vector = 0;
  }
  
  //! Save vector for prefetching
  void save() {
    if(m_vector != 0 && m_vector->size() != 0) {
      // create map entry and assign the vector
      (*m_prefetches)[m_scan] = m_vector;
      // ownership transferred
      m_vector = 0;
    }
  }
private:
  SharedScan* m_scan;
  map<SharedScan*, vector<T>*>* m_prefetches;

  vector<T>* m_vector;
};



ScanHandler::ScanHandler(CacheObject* obj, CacheManager* cm, SharedScan* scan, IODataType data) :
  TemporaryHandler(obj, cm, scan, true),
  m_data(data)
{
}

bool ScanHandler::load()
{
  ScanIO* sio = ScanIO::getScanIO(m_scan->getIOType());
  
  // avoid loading of a non-supported type
  if(!sio->supports(m_data)) return false;
  
#ifdef WITH_METRICS
  Timer t = ServerMetric::scan_loading.start();
#endif //WITH_METRICS
  
  // INFO
  //cout << "[" << m_scan->getIdentifier() << "][" << m_data << "] ScanHandler::load" << endl;
  
  // if binary scan caching is enabled try to read it via TemporaryHandler first, if written-flag wasn't set or file didn't exist, parse scan
  if(binary_caching) {
    if(TemporaryHandler::load()) {
      // INFO
      //cout << "[" << m_scan->getIdentifier() << "][" << m_data << "] ScanHandler::load successful via TemporaryHandler" << endl << endl;
#ifdef WITH_METRICS
      ServerMetric::scan_loading.end(t);
#endif //WITH_METRICS
      return true;
    }
  }
  
  PrefetchVector<double> xyz(m_scan, m_prefetch_xyz);
  PrefetchVector<unsigned char> rgb(m_scan, m_prefetch_rgb);
  PrefetchVector<float> reflectance(m_scan, m_prefetch_reflectance);
  PrefetchVector<float> temperature(m_scan, m_prefetch_temperature);
  PrefetchVector<float> amplitude(m_scan, m_prefetch_amplitude);
  PrefetchVector<int> type(m_scan, m_prefetch_type);
  PrefetchVector<float> deviation(m_scan, m_prefetch_deviation);
  
  // assign vector for this particular ScanHandler
  PrefetchVectorBase* vec = 0;
  if(m_data == DATA_XYZ) vec = &xyz;
  else if(m_data == DATA_RGB) vec = &rgb;
  else if(m_data == DATA_REFLECTANCE) vec = &reflectance;
  else if(m_data == DATA_TEMPERATURE) vec = &temperature;
  else if(m_data == DATA_AMPLITUDE) vec = &amplitude;
  else if(m_data == DATA_TYPE) vec = &type;
  else if(m_data == DATA_DEVIATION) vec = &deviation;
  
  unsigned int prefetch = m_scan->getPrefetch();
  // try to prefetch only the requested type from its handler
  if(vec->prefetch())
  {
    // nothing to do if prefetch was successful, vector is initialized
    // reset prefetch flags, nothing needs to be saved
    prefetch = 0;
  } else {
    // create vector and exclude it from prefetch handling
    vec->create();
    prefetch &= ~m_data;
    
    // create vectors which are to be prefetched
    if(prefetch & DATA_XYZ) xyz.create();
    if(prefetch & DATA_RGB) rgb.create();
    if(prefetch & DATA_REFLECTANCE) reflectance.create();
    if(prefetch & DATA_TEMPERATURE) temperature.create();
    if(prefetch & DATA_AMPLITUDE) amplitude.create();
    if(prefetch & DATA_TYPE) type.create();
    if(prefetch & DATA_DEVIATION) deviation.create();
    
    // request data from the ScanIO
    try {
      PointFilter filter(m_scan->getPointFilter());
      sio->readScan(m_scan->getDirPath(), m_scan->getIdentifier(),
        filter,
        xyz.get(), rgb.get(), reflectance.get(), temperature.get(), amplitude.get(), type.get(), deviation.get());
    } catch(std::runtime_error& e) {
      // INFO
      // cerr << "[" << m_scan->getIdentifier() << "][" << m_data << "] ScanIO runtime_error: " << e.what() << endl;
      // rethrow
      throw e;
    } catch(std::bad_alloc& e) {
      // INFO
      // cerr << "[" << m_scan->getIdentifier() << "][" << m_data << "] ScanIO bad_alloc: " << e.what() << endl;
      // rethrow
      throw e;
    }
  }
  
  // after successful loading, allocate enough cache space
  unsigned int size = vec->size();
  void* data_ptr;
  try {
    data_ptr = m_manager->allocateCacheObject(m_object, size);
  } catch(runtime_error& e) {
    // INFO
    // cerr << "[" << m_scan->getIdentifier() << "][" << m_data << "] CacheManager error: " << e.what() << endl;
    // rethrow
    throw e;
  }
  
  // write data into the cache object and clean up the vector
  vec->write(data_ptr);
  
  // save all vectors that still hold their data for prefetching
  xyz.save();
  rgb.save();
  reflectance.save();
  temperature.save();
  amplitude.save();
  type.save();
  deviation.save();
  
  // INFO
  //cout << "[" << m_scan->getIdentifier() << "][" << m_data << "] ScanHandler::load successful" << endl << endl;
  
#ifdef WITH_METRICS
  ServerMetric::scan_loading.end(t);
#endif //WITH_METRICS
  return true;
}

void ScanHandler::save(unsigned char* data, unsigned int size)
{
  // INFO
  //cout << "[" << m_scan->getIdentifier() << "][" << m_data << "] ScanHandler::save" << endl;
  
  // if global binary scan caching is enabled, save to file for later faster reloading
  if(binary_caching) {
    // duplicate writes of static data are handled in TemporaryHandler already
    TemporaryHandler::save(data, size);
  }
}

void ScanHandler::setBinaryCaching()
{
  binary_caching = true;
}
