/*
 * serverInterface implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/serverInterface.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <stdexcept>
#include <vector>
#include <algorithm>

#include <boost/filesystem.hpp>
using namespace boost::filesystem;
using namespace boost::interprocess;

#include "scanserver/defines.h"
#include "scanserver/frame_io.h"
#include "scanserver/serverScan.h"
#include "scanio/scan_io.h"

#ifndef _MSC_VER
#include <sys/mman.h> // mlock for avoiding swaps
#endif

#ifdef WITH_METRICS
#include "slam6d/metrics.h"
#endif //WITH_METRICS



SharedScanVector* ServerInterface::readDirectory(const char * dir_path, IOType type, unsigned int start, unsigned int end)
{
  // INFO
  cout << "[Scanserver] Reading directory '" << dir_path << "' ... ";
  
  // create an instance of ScanIO
  ScanIO* sio = ScanIO::getScanIO(type);
  
  // query available scans in the directory from the ScanIO
  std::list<std::string> identifiers(sio->readDirectory(dir_path, start, end));
  
  // create a vector for returning
  SharedScanVector* scans = segment_manager->construct<SharedScanVector>(anonymous_instance)(allocator);
  scans->reserve(identifiers.size());
  
  // INFO
  cout << identifiers.size() << " scans found." << endl;
  
  // share the directory path string between all scans because it's the same
  // BUG: before boost-1.47, gcc -O2 and higher caused several boost::ipc:string instances having errors with garbage characters in between and/or more at the end, this is to avoid that problem
  SharedStringSharedPtr dir_path_ptr = make_managed_shared_ptr(
    segment_manager->construct<SharedString>(anonymous_instance)(dir_path, allocator), *segment_manager);
  
  // for each identifier, create a scan
  for(std::list<std::string>::iterator it = identifiers.begin(); it != identifiers.end(); ++it) {
    // first try to get the scan by matching in the global scan vector
    SharedScan* scan = findScan(dir_path_ptr, it->c_str(), type);
    // otherwise create a new one and insert it into the global vector
    if(!scan) {
      // TODO: catch exceptions/errors
      scan = static_cast<SharedScan*>(
        segment_manager->construct<ServerScan>(anonymous_instance)
          (allocator, dir_path_ptr, it->c_str(), type, &m_manager)
        );
      m_scans.push_back(offset_ptr<SharedScan>(scan));
    }
    // insert scan into return scan vector
    scans->push_back(offset_ptr<SharedScan>(scan));
  }
  
  return scans;
}

bool ServerInterface::loadCacheObject(CacheObject* obj)
{
  // INFO
  //cout << "ServerInterface::loadCacheObject..." << endl;
  
  // cache manager handles all the io and allocation issues by calling the cache handler
  return m_manager.loadCacheObject(obj);
}

void ServerInterface::allocateCacheObject(CacheObject* obj, unsigned int size)
{
  // INFO
  //cout << "ServerInterface::allocateCacheObject (" << size << ")" << endl;
  // allocate enough space for a cache object
  m_manager.allocateCacheObject(obj, size);
  
  // INFO
  //cout << endl;
}

void ServerInterface::invalidateCacheObject(CacheObject* obj)
{
  m_manager.invalidateCacheObject(obj);
}

void ServerInterface::getPose(SharedScan* scan)
{
  // INFO
  //cout << "[" << scan->getIdentifier() << "] getPose()";
  
  // TODO: catch exceptions
  double* pose = segment_manager->construct<double>(anonymous_instance)[6]();
  ScanIO* sio = ScanIO::getScanIO(scan->getIOType());
  // TODO: catch more exceptions
  sio->readPose(scan->getDirPath(), scan->getIdentifier(), pose);
  static_cast<ServerScan*>(scan)->setPose(pose);
  
  // INFO
  //cout << endl;
}

void ServerInterface::addFrame(SharedScan* scan)
{
  static_cast<ServerScan*>(scan)->getFrames().push_back(Frame());
}

void ServerInterface::loadFramesFile(SharedScan* scan)
{
  FrameIO::loadFile(scan->getDirPath(), scan->getIdentifier(), static_cast<ServerScan*>(scan)->getFrames());
}

void ServerInterface::saveFramesFile(SharedScan* scan)
{
  FrameIO::saveFile(scan->getDirPath(), scan->getIdentifier(), static_cast<ServerScan*>(scan)->getFrames());
}

void ServerInterface::clearFrames(SharedScan* scan)
{
  static_cast<ServerScan*>(scan)->getFrames().clear();
}

std::size_t ServerInterface::getCacheSize()
{
  return m_cache_size;
}

void ServerInterface::printMetrics()
{
#ifdef WITH_METRICS
  ServerMetric::print();
#endif //WITH_METRICS
}

SharedScan* ServerInterface::findScan(const SharedStringSharedPtr& dir_path, const char* identifier, IOType type) const
{
  // create a stack local scan to compare with all other scans
  SharedScan scan(allocator, dir_path, identifier, type);
  for(SharedScanVector::const_iterator it = m_scans.begin(); it != m_scans.end(); ++it) {
    if(scan == *(*it))
      return it->get();
  }
  return 0;
}

ServerInterface* ServerInterface::create(std::size_t data_size, std::size_t cache_size)
{
  // remove any existing shared memory that wasn't cleaned up
  shared_memory_object::remove(SHM_NAME_DATA);
  // create the shared memory
  if(m_msm != 0)
    throw std::runtime_error("Could not create ServerInterface because shared memory already exists");
  try {
    m_msm = new managed_shared_memory(create_only, SHM_NAME_DATA, data_size);
#ifndef WIN32
    cout << "  Locking data memory... " << std::flush;
    int ret = mlock(m_msm->get_address(), m_msm->get_size());
    if(ret == 0)
      cout << "success.";
    else if(ret == -EPERM)
      cout << "unsuccessful, no permissions.";
    else if(ret == -ENOMEM)
      cout << "unsuccessful, RLIMIT_MEMLOCK too small.";
    else
      cout << "unsuccessful, error=" << ret << ".";
    cout << endl;
#endif
  } catch(interprocess_exception& e) {
    throw std::runtime_error(std::string("Could not create shared memory: ") + e.what());
  }
  // create server interface and set the client pointer to it
  ServerInterface* server = m_msm->construct<ServerInterface>(unique_instance)(m_msm->get_segment_manager(), SHM_NAME_CACHE, cache_size);
  offset_ptr<ClientInterface> *ptr = m_msm->construct<offset_ptr<ClientInterface> >(unique_instance)();
  (*ptr) = reinterpret_cast<ClientInterface *>(server);
  return server;
}

void ServerInterface::destroy()
{
  // delete the server instance first
  try {
    // send message to quit
    ServerInterface* server = m_msm->find<ServerInterface>(unique_instance).first;
    server->cleanup();
/* resetting conditions and mutexes won't work, so just clean up manually
    server->m_condition_server.notify_one();
    server->m_condition_client.notify_one();
    server->m_mutex_client.unlock();
    server->m_mutex_server.unlock();
    m_msm->destroy<ServerInterface>(unique_instance);
*/
  } catch(...) {
    // whatever
  }
  // clean up the memory
  if(m_msm != 0) {
    delete m_msm;
    m_msm = 0;
    shared_memory_object::remove(SHM_NAME_DATA);
    shared_memory_object::remove(SHM_NAME_CACHE);
  }
}

ServerInterface::ServerInterface(managed_shared_memory::segment_manager * sm, const char* shm_name, std::size_t cache_size) :
  ClientInterface(sm),
  m_scans(allocator),
  m_manager(sm, shm_name, cache_size),
  m_cache_size(cache_size)
{
}

ServerInterface::~ServerInterface()
{
}

void ServerInterface::cleanup()
{
  ScanIO::clearScanIOs();
}

void ServerInterface::run()
{
 // take ownership of the server mutex as long as the server is busy
  scoped_lock<interprocess_mutex> lock(m_mutex_server);
  
  // run the shop
  bool running = true;
  while(running) {
    // wait for input notification
    m_condition_server.wait(lock);
    
    // clear the error message because the client isn't responsible for it
    m_error_message.clear();
    
    // process the input
    // TEST: simple int pod for message type
    try {
      if(m_message == MESSAGE_STOP) {
        running = false;
        cout << "Stopping execution by MESSAGE_STOP request." << endl;
      } else
      if(m_message == MESSAGE_READ_DIRECTORY) {
        m_scanvector_ptr = readDirectory(m_arg_string_1.c_str(), m_arg_io_type, m_arg_uint_1, m_arg_uint_2);
      } else
      if(m_message == MESSAGE_LOAD_CACHE_OBJECT) {
        m_arg_uint_1 = (loadCacheObject(m_cacheobject_ptr.get()) == true? 1: 0);
      } else
      if(m_message == MESSAGE_ALLOCATE_CACHE_OBJECT) {
        allocateCacheObject(m_cacheobject_ptr.get(), m_arg_uint_1);
      } else
      if(m_message == MESSAGE_INVALIDATE_CACHE_OBJECT) {
        invalidateCacheObject(m_cacheobject_ptr.get());
      } else
      if(m_message == MESSAGE_GET_POSE) {
        getPose(m_sharedscan_ptr.get());
      } else
      if(m_message == MESSAGE_ADD_FRAME) {
        addFrame(m_sharedscan_ptr.get());
      } else
      if(m_message == MESSAGE_LOAD_FRAMES_FILE) {
        loadFramesFile(m_sharedscan_ptr.get());
      } else
      if(m_message == MESSAGE_SAVE_FRAMES_FILE) {
        saveFramesFile(m_sharedscan_ptr.get());
      } else
      if(m_message == MESSAGE_CLEAR_FRAMES) {
        clearFrames(m_sharedscan_ptr.get());
      } else
      if(m_message == MESSAGE_GET_CACHE_SIZE) {
        m_arg_size_t = getCacheSize();
      } else
      if(m_message == MESSAGE_PRINT_METRICS) {
        printMetrics();
      } else
      {
        cout << "WAH! I do not know thee: " << (unsigned int)m_message << endl;
      }
    } catch(bad_alloc& e) {
      cerr << "Allocation error (you may need to increase the data_size): " << e.what() << endl;
      m_error_message = e.what();
    } catch(std::runtime_error& e) {
      // don't repeat this for the client
      // cerr << "RUNTIME ERROR: " << e.what() << endl;
      m_error_message = e.what();
    }
    // clear message
    m_message = MESSAGE_NONE;
    
    // notify client about completion
    if(running)
      m_condition_client.notify_one();
  }
}
