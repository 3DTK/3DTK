/**
 * @file
 * @brief Scan containing all interprocess shared data and management values.
 *
 * @author Thomas Escher
 */

#ifndef SERVER_SCAN__H
#define SERVER_SCAN__H

#include "sharedScan.h"

// hide the boost namespace and shorten others
namespace
{
  namespace ip = boost::interprocess;
  // the segment manager providing the allocations
  typedef ip::managed_shared_memory::segment_manager SegmentManager;
}

class CacheManager;



/**
 * @brief Privileged derivation of SharedScan for serverside construction.
 *
 * For client/server separation in both code access and compilation matters this class creates the CacheObjects and CacheHandlers for SharedScan in the server process. It further gives access to the pose and frame vector members.
 */
class ServerScan : public SharedScan {
public:
  ServerScan(const ip::allocator<void, SegmentManager> & allocator,
    const SharedStringSharedPtr& dir_path_ptr, const char* io_identifier,
    IOType iotype, CacheManager* cm);

  inline void setPose(double* pose) { m_pose = pose; }
  inline FrameVector& getFrames() { return m_frames; }
private:
};

#endif //SERVER_SCAN__H