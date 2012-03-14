/**
 * @file
 * @brief 
 *
 * @author Thomas Escher
 */

#ifndef FRAME_H
#define FRAME_H

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>

// hide the boost namespace and shorten others
namespace
{
  namespace ip = boost::interprocess;
  // the segment manager providing the allocations
  typedef ip::managed_shared_memory::segment_manager SegmentManager;
}

class Frame;
typedef ip::allocator<Frame, SegmentManager> FrameAllocator;
typedef ip::vector<Frame, FrameAllocator> FrameVector;



/**
 * @brief Simple frame class containing a transformation and type
 */
class Frame {
public:
  double transformation[16];
  unsigned int type;
  
  void set(double* transformation, unsigned int type) {
    for(unsigned int i = 0; i < 16; ++i)
      this->transformation[i] = transformation[i];
    this->type = type;
  }
};

#endif
