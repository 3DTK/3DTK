/**
 * @file
 * @brief 
 *
 * @author Thomas Escher
 */

#ifndef CACHE_DATA_ACCESS_H
#define CACHE_DATA_ACCESS_H

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
 * It is intented to derive this class to provide application-specific access to the contained data by means of overloading operators.
 * A CacheDataAccess is returned by the accessing function in a CacheObject as a temporary and is constructed by these means (&&-ctor). Size and data pointer will be copied and the reading lock will also be obtained. When all CacheDataAccess instances for a given CacheObject are destroyed (fell out of scope) all read locks are gone and the CacheManager is able to remove its contained data from memory.
 */
class CacheDataAccess {
public:
  //! Aquires a lock on the mutex and takes assigned data
  CacheDataAccess(ip::interprocess_upgradable_mutex& mutex, unsigned int& size, unsigned char* data);

  //! Non-locking version
  CacheDataAccess(unsigned int& size, unsigned char* data);

  //! Transfers the lock to this instance and also takes assigned data
  CacheDataAccess(const CacheDataAccess& other);

  //! Bool operator to check whether this cache data holds valid pointer information
  operator bool() const;

protected:
  //! Size in bytes of the referenced data
  inline unsigned int getSize() const { return m_size; }

  //! Referenced data
  inline unsigned char* getData() const { return m_data; }
private:
  //! (RAII) Lock on the CacheObject to ensure the pointer is valid while this instance is up
  ip::sharable_lock<ip::interprocess_upgradable_mutex> m_lock;

  //! Size of the CacheObject
  unsigned int m_size;

  //! Data of the CacheObject
  unsigned char* m_data;
};

#endif //CACHE_DATA_ACCESS_H
