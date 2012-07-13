/**
 * @file
 * @brief Clientside command structure for client-server interaction.
 *
 * @author Thomas Escher
 */

#ifndef SCANSERVER_CLIENTINTERFACE_H
#define SCANSERVER_CLIENTINTERFACE_H

// segment manager, allocators, containers, pointers, mutexes ...
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/sync/interprocess_mutex.hpp>
#include <boost/interprocess/sync/interprocess_condition.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <boost/interprocess/offset_ptr.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/containers/string.hpp>

// hide the boost namespace and shorten others
namespace
{
  namespace ip = boost::interprocess;
  // the segment manager providing the allocations
  typedef ip::managed_shared_memory::segment_manager SegmentManager;
}

// allocator and type for a shared string
typedef ip::allocator<char, SegmentManager> CharAllocator;
typedef ip::basic_string<char, std::char_traits<char>, CharAllocator> SharedString;

#include "scanserver/sharedScan.h"
#include "scanserver/cache/cacheObject.h"



//! TEST: Pod message type
enum message_t {
  MESSAGE_NONE = 0,
  MESSAGE_STOP,
  MESSAGE_READ_DIRECTORY,
  MESSAGE_LOAD_CACHE_OBJECT,
  MESSAGE_ALLOCATE_CACHE_OBJECT,
  MESSAGE_INVALIDATE_CACHE_OBJECT,
  MESSAGE_GET_POSE,
  MESSAGE_ADD_FRAME,
  MESSAGE_LOAD_FRAMES_FILE,
  MESSAGE_SAVE_FRAMES_FILE,
  MESSAGE_CLEAR_FRAMES,
  MESSAGE_GET_CACHE_SIZE,
  MESSAGE_PRINT_METRICS
};



/**
 * @brief Clientside communication for the scanserver management.
 *
 * This is the main class visible in clients, relaying all calls to the server via shared memory. Access can be obtained via create and subsequent calls to getInstance.
 * All calls to the server are put into a message type and its arguments written to interprocess containers for transfer to the server.
 * The ServerInterface derives this class to share the mutexes and arguments and hides the server functionality from the client. This also splits up compilation between the client and server parts.
 */
class ClientInterface {
protected:
  //! The segment manager used for creating new objects with construct<>
  ip::offset_ptr<SegmentManager> segment_manager;

  //! Void allocator to use for ip-STL-containers
  ip::allocator<void, SegmentManager> allocator;
  
  //! Mutex for concurrent access to the client interface, protecting the server communication and argument manipulation
  ip::interprocess_mutex m_mutex_client;
  
  //! Mutex for client-server communication, handling the process switch and work on the server
  ip::interprocess_mutex m_mutex_server;

  //! Condition the server waits on and the client notifies to
  ip::interprocess_condition m_condition_server;
  
  //! Condition the clients will wait on while the server processes
  ip::interprocess_condition m_condition_client;
  
  //! TEST: Pod message type
  message_t m_message;
  
  //! String arguments for message passing
  SharedString m_arg_string_1, m_arg_string_2;
  
  //! Integer arguments for message passing
  unsigned int m_arg_uint_1, m_arg_uint_2;
  
  //! size_t argument for >4GB sizes
  std::size_t m_arg_size_t;
  
  //! Float arguments for message passing
  float m_arg_float_1, m_arg_float_2;
  
  //! IO type argument for message passing
  IOType m_arg_io_type;
  
  //! An error message containing detais
  SharedString m_error_message;
  
  //! Pointer for a scanvector
  ip::offset_ptr<SharedScanVector> m_scanvector_ptr;
  
  //! Pointer for a scan
  ip::offset_ptr<SharedScan> m_sharedscan_ptr;
  
  //! Pointer for a cache object
  ip::offset_ptr<CacheObject> m_cacheobject_ptr;
  
// TODO: remove this later on, this is for close for the testclient
public:
// private:
  //! internal message sending
  void sendMessage(message_t message);

public:
  //! Add and read a directory into the scan vector, ownership: client
  SharedScanVector* readDirectory(const char* dir_path, IOType type = UOS, unsigned int start = 0, unsigned int end = -1);

  //! "Close" a directory by releasing all contained scans, deleting the vector and resetting the pointer
  void closeDirectory(SharedScanVector*& scans);
  
  //! Called from SharedScan on a cache miss, this requests the serverside cache handler to load, returns false if no cached file was found
  bool loadCacheObject(CacheObject* obj);

  //! Called from SharedScan, request enough memory to hold reduced points
  void allocateCacheObject(CacheObject* obj, unsigned int size);

  //! Called from SharedScan, let the CacheManager invalide a CacheObject
  void invalidateCacheObject(CacheObject* obj);

  //! Called from SharedScan, requests the pose
  void getPose(SharedScan* scan);

  //! Called from SharedScan, requests a frame and writes into it
  void addFrame(SharedScan* scan, double* transformation, unsigned int type);

  //! Called from SharedScan, requests loading frames saved in the file
  void loadFramesFile(SharedScan* scan);

  //! Called from SharedScan, requests saving frames into a file
  void saveFramesFile(SharedScan* scan);
 
  //! Called from SharedScan, this removes all previous contained frames
  void clearFrames(SharedScan* scan);
  
  //! Returns the CacheObject shared memory size for allocation and planning purposes
  std::size_t getCacheSize();

  //! Let the server print out its metric
  void printMetrics();
  
protected:
  //! Trivial constructor: initialize all shared memory containers, only allowed to be called by the ServerInterface
  ClientInterface(SegmentManager* sm) :
    segment_manager(sm),
    allocator(sm),
    m_message(MESSAGE_NONE),
    m_arg_string_1(allocator),
    m_arg_string_2(allocator),
    m_error_message(allocator)
  {
  }
  
  // only the instance in the server will deconstruct
  virtual ~ClientInterface()
  {
  }
  
protected:
  //! The one shared memory opened in the application, a single one in each of the clients and in the server
  static ip::managed_shared_memory* m_msm;

private:
  //! Client singleton for easy global access in the whole application
  static ClientInterface* m_singleton;

public:
  //! Opens the shared memory and sets the singleton in a client process
  static ClientInterface* create();

  //! Closes the shared memory
  static void destroy();

  //! Returns the singleton instance
  static ClientInterface* getInstance();
};

#endif //SCANSERVER_CLIENTINTERFACE_H
