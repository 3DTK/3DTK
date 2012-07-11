/*
 * cacheObject implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/cache/cacheObject.h"

#include <stdexcept>
#include <string>
using std::runtime_error;
using std::string;

using namespace boost::interprocess;

managed_shared_memory* CacheObject::m_msm = 0;

CacheObject::CacheObject() :
  m_size(0),
  m_handle(0),
  m_handler(0)
{
}

CacheObject::~CacheObject()
{
}

void CacheObject::setCacheHandler(CacheHandler* handler)
{
  m_handler = handler;
}

void CacheObject::openSharedMemory(const char* shm_name)
{
 // open the shared memory in this application
  if(m_msm == 0) {
    try {
      m_msm = new managed_shared_memory(open_only, shm_name);
    } catch(interprocess_exception& e) {
      throw runtime_error(string("Could not open shared memory: ") + e.what());
    }
  }
}
