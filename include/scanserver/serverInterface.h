/**
 * @file
 * @brief Serverside command structure for client-server interaction.
 *
 * @author Thomas Escher
 */

#ifndef SCANSERVER_SERVERINTERFACE_H
#define SCANSERVER_SERVERINTERFACE_H

#include "scanserver/clientInterface.h"
#include "scanserver/cache/cacheManager.h"

// hide the boost namespace
namespace
{
  namespace ip = boost::interprocess;
  // the segment manager providing the allocations
  typedef ip::managed_shared_memory::segment_manager SegmentManager;
}



/**
 * @brief Central instance of the server management.
 *
 * This class handles all the serverside communication and relays cache management calls to the CacheManager.
 * It derives ClientInterface and shares its mutexes and arguments, neccessary for the communication. It also holds the SharedScan and CacheManager instances.
 * create will open the shared memory and place a ServerInterface instance in it, after which the main server loop run handles all communication.
 */
class ServerInterface : public ClientInterface
{
private:
  //! Container for all shared scans the server can hold
  SharedScanVector m_scans;

  //! Cache manager for handling all cache objects inside the shared scans
  CacheManager m_manager;

  //! Saved size of the CacheObject shared memory
  std::size_t m_cache_size;

private:
  //! Read a directory of scans by letting the corresponding ScanIO reading it and creating a scan for each entry
  SharedScanVector* readDirectory(const char * dir_path, IOType type, unsigned int start, unsigned int end);

  //! Cache miss from a CacheObject, relayed to CacheManager
  bool loadCacheObject(CacheObject* obj);

  //! Allocate call from SharedScan, relayed to CacheManager
  void allocateCacheObject(CacheObject* obj, unsigned int size);

  //! Invalidate call from SharedScan, relayed to CacheManager
  void invalidateCacheObject(CacheObject* obj);

  //! Call from SharedScan, relayed to ScanIO
  void getPose(SharedScan* scan);

  //! Allocate a new Frame in its vector
  void addFrame(SharedScan* scan);

  //! Relayed to FrameIO
  void loadFramesFile(SharedScan* scan);

  //! Relayed to FrameIO
  void saveFramesFile(SharedScan* scan);
 
  //! Empties this SharedScan's FrameVector
  void clearFrames(SharedScan* scan);
  
  //! Call from client
  std::size_t getCacheSize();

  //! Prints out caching related metrics
  void printMetrics();
  
  //! Find a scan by matching its identifier, path and io type to avoid placing the same scan multiple times into the scan vector
  SharedScan* findScan(const SharedStringSharedPtr& dir_path, const char* identifier, IOType type) const;

public:
  /**
   * Constructor
   * @param sm SegmentManager for allocator objects to construct interprocess containers with
   * @param shm_name Name of the cache shared memory that will be passed to CacheManager
   * @param cache_size Size of cache shared memory
   */
  ServerInterface(SegmentManager* sm, const char* shm_name, std::size_t cache_size);
  ~ServerInterface();

  //! Create the shared memory in the system and put all neccessary structures in it
  static ServerInterface* create(std::size_t data_size, std::size_t cache_size);
  
  //! remove the shared memory from the system
  static void destroy();
  
  //! Main server loop for message handling and function dispatching
  void run();

private:
  //! Cleaning up internal data without destroying the instance
  void cleanup();
};

#endif //SCANSERVER_SERVERINTERFACE_H
