#include "scanserver/cache/cacheDataAccess.h"

using namespace boost::interprocess;

CacheDataAccess::CacheDataAccess(ip::interprocess_upgradable_mutex& mutex, unsigned int& size, unsigned char* data) :
  m_lock(mutex),
  m_size(size), m_data(data)
{
}

CacheDataAccess::CacheDataAccess(unsigned int& size, unsigned char* data) :
  m_size(size), m_data(data)
{
}

CacheDataAccess::CacheDataAccess(const CacheDataAccess& other) :
  m_lock(),
  m_size(other.m_size), m_data(other.m_data)
{
  if(other.m_lock.mutex() != 0) {
    ip::sharable_lock<ip::interprocess_upgradable_mutex> lock(*other.m_lock.mutex());
    m_lock.swap(lock);
  }
}

CacheDataAccess::operator bool() const
{
  // if size is non-zero we should have a valid pointer to cache shared memory, don't know what happens to a zero-handle after you put it into a segment manager get address function
  return m_size != 0;  
}
