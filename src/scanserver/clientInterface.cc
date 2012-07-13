/*
 * clientinterface implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/clientInterface.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::cin;
using std::endl;
#include <stdexcept>

#include <boost/filesystem.hpp>
using namespace boost::filesystem;
using namespace boost::interprocess;

#include "scanserver/defines.h"

#ifdef WITH_METRICS
#include "slam6d/metrics.h"
#endif



managed_shared_memory* ClientInterface::m_msm = 0;
ClientInterface* ClientInterface::m_singleton = 0;



SharedScanVector * ClientInterface::readDirectory(const char * dir_path, IOType type, unsigned int start, unsigned int end)
{
  path to_add(dir_path);
  if(!exists(to_add) || !is_directory(to_add)) {
    // TODO: throw an exception
    cerr << "Directory " << to_add << " does not exist" << endl;
    return 0;
  }
  
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);

  // pass a system complete (absolute) path for compability with possibly
  // different working directories in different processes
  m_arg_string_1 = system_complete(to_add).string().c_str();
  m_arg_io_type = type;
  m_arg_uint_1 = start;
  m_arg_uint_2 = end;
  sendMessage(MESSAGE_READ_DIRECTORY);
  // don't catch the exception, there is nothing this function can fix
  SharedScanVector* scans = m_scanvector_ptr.get();
  m_scanvector_ptr = 0;
  return scans;
}

void ClientInterface::closeDirectory(SharedScanVector*& scans)
{
  if(scans == 0) return;
  
  // release all scans
  // TODO: either lower recursive mutexes in all scans or notice the server
  
  // delete scan vector
  segment_manager->destroy_ptr(scans);
  
  // set scans to 0 since it's a reference
  scans = 0;
}

bool ClientInterface::loadCacheObject(CacheObject* obj)
{
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);
  
#ifdef WITH_METRICS
  Timer t = ClientMetric::cache_miss_time.start();
#endif //WITH_METRICS
  
  m_cacheobject_ptr = obj;
  sendMessage(MESSAGE_LOAD_CACHE_OBJECT);
  bool success = m_arg_uint_1 == 1;
  m_arg_uint_1 = 0;
  
#ifdef WITH_METRICS
  ClientMetric::cache_miss_time.end(t);
#endif //WITH_METRICS
  
  return success;
}

void ClientInterface::allocateCacheObject(CacheObject* obj, unsigned int size)
{
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);
  
#ifdef WITH_METRICS
  Timer t = ClientMetric::allocate_time.start();
#endif //WITH_METRICS
  
  m_cacheobject_ptr = obj;
  m_arg_uint_1 = size;
  sendMessage(MESSAGE_ALLOCATE_CACHE_OBJECT);
  
#ifdef WITH_METRICS
  ClientMetric::allocate_time.end(t);
#endif //WITH_METRICS
}

void ClientInterface::invalidateCacheObject(CacheObject* obj)
{
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);
  
  m_cacheobject_ptr = obj;
  sendMessage(MESSAGE_INVALIDATE_CACHE_OBJECT);
}

void ClientInterface::getPose(SharedScan* scan)
{
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);

  m_sharedscan_ptr = scan;
  sendMessage(MESSAGE_GET_POSE);
}

void ClientInterface::addFrame(SharedScan* scan, double* transformation, unsigned int type)
{
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);
  
#ifdef WITH_METRICS
  Timer t = ClientMetric::frames_time.start();
#endif //WITH_METRICS
  
  m_sharedscan_ptr = scan;
  sendMessage(MESSAGE_ADD_FRAME);
  
  // write into the new frame
  const FrameVector& frames = scan->getFrames();
  Frame& frame = const_cast<Frame&>(frames.back());
  frame.set(transformation, type);
  
#ifdef WITH_METRICS
  ClientMetric::frames_time.end(t);
#endif //WITH_METRICS
}

void ClientInterface::loadFramesFile(SharedScan* scan)
{
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);
  
#ifdef WITH_METRICS
  Timer t = ClientMetric::frames_time.start();
#endif //WITH_METRICS

  m_sharedscan_ptr = scan;
  sendMessage(MESSAGE_LOAD_FRAMES_FILE);
  
#ifdef WITH_METRICS
  ClientMetric::frames_time.end(t);
#endif //WITH_METRICS
}

void ClientInterface::saveFramesFile(SharedScan* scan)
{
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);
  
#ifdef WITH_METRICS
  Timer t = ClientMetric::frames_time.start();
#endif //WITH_METRICS

  m_sharedscan_ptr = scan;
  sendMessage(MESSAGE_SAVE_FRAMES_FILE);
  
#ifdef WITH_METRICS
  ClientMetric::frames_time.end(t);
#endif //WITH_METRICS
}

void ClientInterface::clearFrames(SharedScan* scan)
{
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);
  
#ifdef WITH_METRICS
  Timer t = ClientMetric::frames_time.start();
#endif //WITH_METRICS

  m_sharedscan_ptr = scan;
  sendMessage(MESSAGE_CLEAR_FRAMES);
  // TODO: remove the .frames-file if appropriate, clear all records
  
#ifdef WITH_METRICS
  ClientMetric::frames_time.end(t);
#endif //WITH_METRICS
}

std::size_t ClientInterface::getCacheSize()
{
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);
  
  sendMessage(MESSAGE_GET_CACHE_SIZE);
  
  return m_arg_size_t;
}

void ClientInterface::printMetrics()
{
  // aquire client mutex for uninterrupted work
  scoped_lock<interprocess_mutex> lock(m_mutex_client);
  
  sendMessage(MESSAGE_PRINT_METRICS);
}

void ClientInterface::sendMessage(message_t message)
{
#ifdef WITH_METRICS
  Timer t = ClientMetric::clientinterface_time.start();
#endif //WITH_METRICS
  
  // try to lock the server mutex to send when it's not occupied
  scoped_lock<interprocess_mutex> lock(m_mutex_server);
  
  // set message
  m_message = message;
  
  // notify and wake up the server and wait until the message is processed
  m_condition_server.notify_one();
  m_condition_client.wait(lock);
  
#ifdef WITH_METRICS
  ClientMetric::clientinterface_time.end(t);
#endif //WITH_METRICS
  
  // process errors
  // TODO: better
  if(!m_error_message.empty()) {
      std::string msg(m_error_message.c_str());
     m_error_message.clear();
     throw std::runtime_error(msg);
  }
}

ClientInterface* ClientInterface::create()
{
  // open the shared memory in this application
  if(m_msm == 0) {
    try {
      m_msm = new managed_shared_memory(open_only, SHM_NAME_DATA);
    } catch(interprocess_exception& e) {
      throw std::runtime_error(std::string("Could not open shared memory: ") + e.what());
    }
  }
  
  // now try to open the cache shared memory
  CacheObject::openSharedMemory(SHM_NAME_CACHE);
  
  // prepare singleton pointer
  offset_ptr<ClientInterface>* ptr = m_msm->find<offset_ptr<ClientInterface> >(unique_instance).first;
  if(ptr == 0)
    throw std::runtime_error("Could not find the ClientInterface pointer in shared memory");
  m_singleton = ptr->get();
  
  // test for old crashed clients
  {
    scoped_lock<interprocess_mutex> lock(m_singleton->m_mutex_client, try_to_lock);
    if(!lock) {
      cout << "Scanserver communication is currently blocked, waiting for server... " << std::flush;
      // wait for server to finish, lock and immediately unlock the server mutex
      {scoped_lock<interprocess_mutex> lock_server(m_singleton->m_mutex_server);}
      // clean up the client mutex which was left locked by previous program calls
      m_singleton->m_mutex_client.unlock();
      cout << "done." /* Stop crashing/interrupting programs */ << endl;
    }
  }
  
  return m_singleton;
}

void ClientInterface::destroy()
{
  m_singleton = 0;
  if(m_msm != 0) {
    delete m_msm;
    m_msm = 0;
  }
}

ClientInterface* ClientInterface::getInstance()
{
  if(m_singleton == 0)
    throw std::runtime_error("ClientInterface Singleton not set");
  return m_singleton;
}
