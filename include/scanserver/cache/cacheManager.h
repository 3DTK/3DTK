/**
 * @file
 * @brief 
 *
 * @author Thomas Escher
 */

#ifndef CACHE_MANAGER_H
#define CACHE_MANAGER_H

#include <vector>
#include <string>

// segment manager, allocators, pointers, ...
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/allocators/allocator.hpp>

// hide the boost namespace and shorten others
namespace
{
  namespace ip = boost::interprocess;
  // the segment manager providing the allocations
  typedef ip::managed_shared_memory::segment_manager SegmentManager;
}

#include "scanserver/cache/cacheObject.h"
#include "scanserver/cache/cacheHandler.h"


/**
 * @brief Central cache management for CacheObjects, handling cache misses and calling CacheHandlers and allocating memory for contents.
 *
 * The CacheManager creates and handles CacheObjects in the shared memory given by the segment manager in the constructor. It also opens a shared memory exclusively for CacheObjects' contents.
 * Cache misses in CacheObject should invoke loadCacheObject to have it loaded into memory. This CacheObject's CacheHandler is called, which in turn requests memory via allocateCacheObject. This function tries to allocate enough memory and flushes out other CacheObjects which are not read-locked in order to do the former.
 * The flushing behaviour determines which CacheObjects are to be removed first and can be altered. (TODO)
 */
class CacheManager {
public:
  /**
   * The SegmentManager comes from another shared memory where COs have to be created in.
   * A CacheManager owned shared memory will be opened to create the CO contents in.
   */
  CacheManager(SegmentManager* sm, const char* shm_name, std::size_t cache_size);
  
  /**
   * Deletes all CacheObjects and closes the shared memory.
   */
  ~CacheManager();

  /**
   * Allocates a CacheObject in shared memory.
   */
  CacheObject* createCacheObject();

  /**
   * Request a CacheObject to be loaded into memory, causing its CacheHandler to be called.
   * If no resource was to be found, the function returns false. If an error occured (e.g. IO/stream/conversion errors) it will throw.
   * @return if the CacheObject was loaded successfully
   * @throws when no memory could be allocated (see allocateCacheObject) or an error occured in the CacheHandler implementation
   */
  bool loadCacheObject(CacheObject* obj);

  /**
    * Allocates enough space for a cache object. This will flush other CacheObjects if the exclusive shared memory is full.
    * @return Pointer to the allocated space in the object
    * @throws when no memory could be allocated because removing all remaining (non read-locked) CacheObjects removed didn't free enough memory.
    */
  unsigned char* allocateCacheObject(CacheObject* obj, unsigned int size);
  
  /**
   * Invalidate a CacheObject and its handler.
   */
  void invalidateCacheObject(CacheObject* obj);

  /**
   * Change the flushing behaviour by setting a specific heuristic.
   */
  // TODO

private:
  SegmentManager* m_segment_manager;
  ip::managed_shared_memory* m_msm;
  std::string m_shm_name;

  std::vector<CacheObject*> m_objects, m_loaded;

  /**
   * Allocates memory for a CO. Will throw a bad_alloc if it fails so.
   * Only to be called within allocateCacheObject.
   */
  unsigned char* load(CacheObject* obj, unsigned int size);

  /**
   * Removes cached data from a CO and marks it as unloaded.
   * Only to be called when an exclusive lock has been obtained inside allocateCacheObject.
   */
  void unload(CacheObject* obj);
};

#endif //CACHE_MANAGER_H
