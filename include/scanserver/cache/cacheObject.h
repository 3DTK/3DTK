/**
 * @file
 *
 * @author Thomas Escher
 */

#ifndef CACHE_OBJECT_H
#define CACHE_OBJECT_H

#include <boost/interprocess/sync/interprocess_upgradable_mutex.hpp>
#include <boost/interprocess/sync/sharable_lock.hpp>
#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/managed_shared_memory.hpp>

// hide the boost namespace and shorten others
namespace
{
  namespace ip = boost::interprocess;
}

#include "scanserver/cache/cacheHandler.h"
#include "scanserver/cache/cacheDataAccess.h"



/**
 * @brief An object representing a cache entry, holding the data and managing cache access.
 *
 * This cache object holds a pointer to the cached data if it is loaded and is accessible through the aquisition of a CacheDataAccess. The CacheDataAccess will lock this CacheObject so the CacheManager can't remove it from memory while it is read (i.e. CacheDataAccess holds a lock). If the data isn't hold in memory the access will cause a cache miss to occur and consequently communications to the CacheManager are started to get this CacheObject loaded into memory.
 * Cached data is held in a shared memory exclusively for these objects. On client startup openSharedMemory has to be called once. Every access then obtains the process-local data pointer by its handle in this shared memory.
 * When the CacheObject is created it has to be assigned a CacheHandler which handles the application specific IO part. Save calls should serialize the CacheObject's contents to a safe place (e.g. harddrive), Load calls should recall these contents.
 */
class CacheObject {
  friend class CacheManager;
public:
  CacheObject();
  ~CacheObject();

  /**
   * Aquires the lock, accesses the contained data directly on hit or requests loading from the CacheManager if missed.
   * The aquisition takes place through the function represented by template F. F relays the call to the process who created the CacheHandler which is then able to do its work. This is neccessary because CacheHandler owns virtual functions and these can only be called in the creating process.
   *
   * Can throw an exception if the server is unable to load the data.
   */
  template<void(*F)(CacheObject*)>
  inline CacheDataAccess getCacheData()
  {
    // lock read mutex to prevent removal in between calls
    ip::sharable_lock<ip::interprocess_upgradable_mutex> use(m_mutex_in_use);
    // aquire data by safely requesting it once through the means of functionality given by F
    if(m_handle == 0) {
      ip::scoped_lock<ip::interprocess_mutex> request(m_cache_miss);
      if(m_handle == 0) {
        F(this);
      }
      // TODO: exceptions checking
    }
    // TODO: Access Data
    return CacheDataAccess(m_mutex_in_use, m_size, reinterpret_cast<unsigned char*>(m_msm->get_address_from_handle(m_handle)));
  }
  
  /**
   * Allocate space to write into.
   * Repeated calls will always create new space without saving the old one, no CacheHandler calls will be made for this CacheObject.
   */
  template<void(*F)(CacheObject*, unsigned int)>
  inline CacheDataAccess createCacheData(unsigned int size)
  {
    // lock read mutex to prevent removal in between calls
    ip::sharable_lock<ip::interprocess_upgradable_mutex> use(m_mutex_in_use);
    // allocate data through template function
    F(this, size);
    // TODO: Access Data
    return CacheDataAccess(m_mutex_in_use, m_size, reinterpret_cast<unsigned char*>(m_msm->get_address_from_handle(m_handle)));
  }
  
  /**
   * Let the CacheManager invalidate this CacheObject and its handler
   */
  template<void(*F)(CacheObject*)>
  inline void invalidate()
  {
    // lock read mutex to prevent removal in between calls
    ip::sharable_lock<ip::interprocess_upgradable_mutex> use(m_mutex_in_use);
    // allocate data through template function
    F(this);
    // TODO: Access Data
  }
  
  /**
   * Set a cache handler for loading and saving data on cache misses or flushes.
   * This may only be assigned once and load/save calls have to be process local.
   */
  void setCacheHandler(CacheHandler* handler);
  
  /**
   * Open the shared memory on client side so CacheObjects can access their data from there.
   * Call once on client initialization.
   */
  static void openSharedMemory(const char* shm_name);
private:
  //! Size in bytes of contained data
  unsigned int m_size;
  
  //! Handle to contained data in CacheObject exclusive shared memory, used to obtain process-local pointers
  ip::managed_shared_memory::handle_t m_handle;
  
  //! Will be share-locked by every reading entity, exclusive-locked by manipulating entity, the CacheManager
  ip::interprocess_upgradable_mutex m_mutex_in_use;
  
  //! Execute-once protection for a read request on a cache miss
  ip::interprocess_mutex m_cache_miss;
  
  //! IO handling object for load and saves, to be called within the creating process
  CacheHandler* m_handler;
  
  //! Singleton shared memory for data access
  static ip::managed_shared_memory* m_msm;
};

#endif //CACHE_OBJECT_H
