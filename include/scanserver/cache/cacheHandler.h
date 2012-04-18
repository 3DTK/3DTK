/**
 * @file
 * @brief 
 *
 * @author Thomas Escher
 */

#ifndef CACHE_HANDLER_H
#define CACHE_HANDLER_H

class CacheManager;
class CacheObject;

/**
 * @brief IO handling for CacheObjects to (de)serialize the cached data.
 *
 * Each CacheObject has to be assigned a CacheHandler to have its contents loaded (e.g. from harddrive) when a cache miss occurs or save it when the CacheManager flushes the cache and removes CacheObjects' contents.
 * The load and save functions are virtual and thereby can only be called within the process where they have been created.
 *
 */
class CacheHandler {
public:
  CacheHandler(CacheObject* object, CacheManager* manager);
  virtual ~CacheHandler();

  /**
   * Called by the CacheManager when a CacheObject has to be loaded after a cache miss
   * The usual course of action should be to determine the size (and load the data locally if needed), request the referenced CacheObject to be allocated by the CacheManager via allocateCacheObject and have the data written to the obtained memory.
   * @return whether the load was successful or no cached resource was assigned/found
   * @throw possibly IO/stream/conversion errors in overloaded classes
   */
  virtual bool load() = 0;
  
  /**
   * Called by the CacheManager when a CacheObject has to be saved before removal
   * The data to be saved it given in the arguments and will be removed by the CacheManager after this function returns.
   * @throw possibly IO/stream/conversion errors in overloaded classes
   */
  virtual void save(unsigned char* data, unsigned int size) = 0;

  /**
   * Called by the CacheManager when a CacheObject has been invalidated.
   * Appropriate action should be taken to invalidate the handled resource.
   */
  virtual void invalidate() = 0;
protected:
  CacheManager* m_manager;
  CacheObject* m_object;
};

#endif //CACHE_HANDLER_H
