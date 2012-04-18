/**
 * @file
 * @brief 
 *
 * @author Thomas Escher
 */

#ifndef CACHE_DATA_ACCESS_H
#define CACHE_DATA_ACCESS_H

#include "slam6d/data_types.h"

#include <boost/interprocess/sync/interprocess_upgradable_mutex.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>

// hide the boost namespace and shorten others
namespace
{
  namespace ip = boost::interprocess;
}



/**
 * @brief Access object for data in a CacheObject.
 *
 * This object functions as a reading lock on the data in a CacheObject. While any of these objects exists with a lock on a CacheObject it is guaranteed the CacheManager won't remove it from memory when flushing the cache.
 * It is derived from the DataPointer class, which already represents application-specific access operators for this data, and builds the locking functionality on top of it.
 * A CacheDataAccess is returned by the accessing function in a CacheObject as a temporary and is constructed by these means (&&-ctor). Size and data pointer will be copied and the reading lock will also be obtained. When all CacheDataAccess instances for a given CacheObject are destroyed (fell out of scope) all read locks are gone and the CacheManager is able to remove its contained data from memory.
 */
class CacheDataAccess : public DataPointer {
  class Lock : public PrivateImplementation {
  public:
    Lock(ip::interprocess_upgradable_mutex& mutex);
    // on destruction, the lock has to be released, which comes from m_lock destruction
    virtual ~Lock() {}
  private:
    ip::sharable_lock<ip::interprocess_upgradable_mutex> m_lock;
  };
public:
  //  CacheDataAccess& operator=(const CacheDataAccess&) = delete;
  //  CacheDataAccess(const CacheDataAccess&) = delete;

  //! Aquires a lock on the mutex and takes assigned data
  CacheDataAccess(ip::interprocess_upgradable_mutex& mutex, unsigned int& size, unsigned char* data);

  CacheDataAccess(CacheDataAccess&& other) : DataPointer(other) {}

  //! Non-locking version
  //CacheDataAccess(unsigned int& size, unsigned char* data);

  //! Transfers the lock to this instance and also takes assigned data
  //CacheDataAccess(const CacheDataAccess& other);
};

#endif //CACHE_DATA_ACCESS_H
