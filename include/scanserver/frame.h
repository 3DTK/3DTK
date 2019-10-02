/**
 * @file
 * @brief
 *
 * @author Thomas Escher
 */

#ifndef SHARED_FRAME_H
#define SHARED_FRAME_H

#include "slam6d/frame.h"

#if defined(_WINDEF_) && defined(__CYGWIN__)
#error boost-interprocess is incompatible with windef.h on cygwin
#endif
#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>

// hide the boost namespace and shorten others
namespace
{
  namespace ip = boost::interprocess;
  // the segment manager providing the allocations
  typedef ip::managed_shared_memory::segment_manager SegmentManager;
}

typedef ip::allocator<Frame, SegmentManager> FrameAllocator;
typedef ip::vector<Frame, FrameAllocator> FrameVector;


#endif //SHARED_FRAME_H
