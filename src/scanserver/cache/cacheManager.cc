/*
 * cacheManager implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/cache/cacheManager.h"

#include <stdexcept>
#include <string>

#include <boost/interprocess/exceptions.hpp>

using namespace boost::interprocess;
using std::runtime_error;
using std::vector;
using std::string;

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#ifdef _MSC_VER
#include <windows.h>
#else
#include <sys/mman.h> // mlock for avoiding swaps
#endif


CacheManager::CacheManager(SegmentManager* sm, const char* shm_name, std::size_t cache_size) :
  m_segment_manager(sm),
  m_shm_name(shm_name)
{
  // remove any existing shared memory that wasn't cleaned up
  shared_memory_object::remove(m_shm_name.c_str());
  
  try {
    m_msm = new managed_shared_memory(create_only, m_shm_name.c_str(), cache_size);
#ifndef WIN32
    cout << "  Locking cache memory... " << std::flush;
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
}

CacheManager::~CacheManager()
{
  // clean up objects
  for(vector<CacheObject*>::iterator it = m_objects.begin(); it != m_objects.end(); ++it)
    m_segment_manager->destroy_ptr(*it);
  
  // remove cache data shared memory
  delete m_msm;
  shared_memory_object::remove(m_shm_name.c_str());
}

CacheObject* CacheManager::createCacheObject()
{
  CacheObject* obj = m_segment_manager->construct<CacheObject>(anonymous_instance)();
  m_objects.push_back(obj);
  return obj;
}

bool CacheManager::loadCacheObject(CacheObject* obj)
{
  if(obj->m_handler) {
    return obj->m_handler->load();
  } else {
    throw runtime_error("No CacheHandler set for loading");
  }
}

unsigned char* CacheManager::allocateCacheObject(CacheObject* obj, unsigned int size)
{
  // remove old data if this isn't a cache miss call but a direct allocate call
  if(obj->m_handle != 0) {
    if(size == obj->m_size) {
      // INFO
      //cout << "CacheManager::allocateCacheObject reusing space" << endl;
      // space fits? leave it be
      return reinterpret_cast<unsigned char*>(m_msm->get_address_from_handle(obj->m_handle));
    } else {
      // reset CO
      m_msm->destroy_ptr(m_msm->get_address_from_handle(obj->m_handle));
      obj->m_size = 0;
      obj->m_handle = 0;
    }
  }
  
  // try to allocate it initially
  try {
    return load(obj, size);
  } catch(bad_alloc& e) {
    // flush behaviour below
  }
  
  // create a list of COs to remove from memory
  // TODO: create the list from the heuristic
  vector<CacheObject*> loaded = m_loaded;
  // try to exclusively lock COs to remove them from memory
  for(vector<CacheObject*>::iterator it = loaded.begin(); it != loaded.end(); ++it) {
    CacheObject* target = *it;
    scoped_lock<interprocess_upgradable_mutex> lock(target->m_mutex_in_use, try_to_lock);
    if(lock) {
      unload(target);
      // try to allocate it
      try {
        return load(obj, size);
      } catch(bad_alloc& e) {
        // continue flushing
      }
    }
  }
  
  // when flushing didn't work, there's nothing to do anymore
  throw runtime_error("CacheManager could not allocate enough memory for CacheObject. All available memory is locked for used CacheObjects and you need to increase the cache memory size");
}

void CacheManager::invalidateCacheObject(CacheObject* obj)
{
  // remove its data
  if(obj->m_handle != 0) {
    // reset CO
    m_msm->destroy_ptr(m_msm->get_address_from_handle(obj->m_handle));
    obj->m_size = 0;
    obj->m_handle = 0;
  }
  
  // invalidate the handler too
  obj->m_handler->invalidate();
}

unsigned char* CacheManager::load(CacheObject* obj, unsigned int size)
{
  // INFO
  //cout << " CM::load (" << size << ")" << endl;
  
  // allocate
  unsigned char* data = m_msm->construct<unsigned char>(anonymous_instance)[size]();
  obj->m_size = size;
  obj->m_handle = m_msm->get_handle_from_address(data);
  
  // mark it as loaded
  m_loaded.push_back(obj);
  
  return data;
}

void CacheManager::unload(CacheObject* obj)
{
  if(obj->m_handle == 0) return;
  
  // INFO
  //cout << " CM::unload" << endl;
  
  // save the CO by its handler
  unsigned char* data = reinterpret_cast<unsigned char*>(m_msm->get_address_from_handle(obj->m_handle));
  obj->m_handler->save(data, obj->m_size);
  // TODO: exceptions?
  
  // reset CO
  m_msm->destroy_ptr(data);
  obj->m_size = 0;
  obj->m_handle = 0;
  
  // mark it as unloaded
  for(vector<CacheObject*>::iterator it = m_loaded.begin(); it != m_loaded.end(); ++it) {
    if(obj == *it) {
      m_loaded.erase(it);
      break;
    }
  }
}
