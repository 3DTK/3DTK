/**
 * @file
 * @brief Scan containing all interprocess shared data and management values.
 *
 * @author Thomas Escher
 */

#ifndef SHARED_SCAN_H
#define SHARED_SCAN_H

#include "scanserver/frame.h"
#include "slam6d/io_types.h"
#include "slam6d/data_types.h"
#include "slam6d/pointfilter.h"

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/smart_ptr/shared_ptr.hpp>
#include <boost/interprocess/containers/string.hpp>
#include <boost/interprocess/containers/vector.hpp>

// hide the boost namespace and shorten others
namespace
{
  namespace ip = boost::interprocess;
  // the segment manager providing the allocations
  typedef ip::managed_shared_memory::segment_manager SegmentManager;
}

// allocator and type for a scan vector
class SharedScan;
typedef ip::allocator<ip::offset_ptr<SharedScan>, SegmentManager> SharedScanAllocator;
typedef ip::vector<ip::offset_ptr<SharedScan>, SharedScanAllocator> SharedScanVector;

// allocator and type for a shared string
typedef ip::allocator<char, SegmentManager> CharAllocator;
typedef ip::basic_string<char, std::char_traits<char>, CharAllocator> SharedString;

// shared pointer for shared strings
typedef ip::managed_shared_ptr<SharedString, ip::managed_shared_memory>::type SharedStringSharedPtr;

class CacheObject;



/**
 * @brief Central class representing a single scan file in shared memory.
 *
 * This class identifies a scan file by its path, identificator and io type, which are given to the ScanIO to read a scan.
 * It holds all neccessary information like pose, frames and cached data from the scan. The cached data is contained in CacheObjects, one for each data channel and two reduced (one transformed, one untransformed). The access to the CacheObjects' data is given via a derivation from CacheData via MultiArray-types, which imitates a double**-array for easy use.
 * All calls to the server are relayed to the ClientInterface and handled there. Access to the CacheObjects causing a cache miss also invoke a server call to the CacheManager.
 */
class SharedScan
{
  // private static class access for onChacheMiss/onAllocation
  friend class CacheObject;
public:
  //! Constructor by identifiers
  SharedScan(const ip::allocator<void, SegmentManager> & allocator,
    const SharedStringSharedPtr& dir_path_ptr, const char* io_identifier,
    IOType iotype);

  //! Deconstructor
  ~SharedScan();

  //! Equality operator based on identifier, directory and type
  bool operator==(const SharedScan& r) const;
  
  //! Filter parameters for range checks when loading from file, invalidate cache for scan CacheObject if it differs
  void setRangeParameters(double max_dist, double min_dist);

  //! Filter parameters for height checks when loading from file, invalidate cache for scan CacheObject if it differs
  void setHeightParameters(double top, double bottom);

  //! Filter parameters for the range mutator for showing the spheres, invalidate cache for scan CacheObject if it differs
  void setRangeMutationParameters(double range);
    
  //! Set parameters and invalidate cache for reduced CacheObjects if it differs
  void setReductionParameters(const char* params);

  //! Set parameters and invalidate cache for reduced CacheObjects if it differs
  void setShowReductionParameters(const char* params);

  //! Set parameters and invalidate cache for reduced CacheObjects if it differs
  void setOcttreeParameters(const char* params);

  //! Add a new frame with the current transformation and given type
  void addFrame(double* transformation, unsigned int type);
  
  //! Save frames into a file for later use
  void saveFrames();

  //! Clear existing frames
  void clearFrames();
  
  //! Get contained frames
  const FrameVector& getFrames();
  
  //! Get pose from pose file
  double* getPose();
  
  //! Get specific cached data
  DataXYZ getXYZ();
  DataRGB getRGB();
  DataReflectance getReflectance();
  DataTemperature getTemperature();
  DataAmplitude getAmplitude();
  DataType getType();
  DataDeviation getDeviation();
  
  //! Reduced transformed points
  DataXYZ getXYZReduced();
  
  //! Create a new set of reduced points
  DataXYZ createXYZReduced(unsigned int size);


  //! Create a new set of reflectance
  DataReflectance createReflectance(unsigned int size);
  

  //! Reduced untransformed points
  DataXYZ getXYZReducedOriginal();
  
  //! Create a new set of reduced points originals
  DataXYZ createXYZReducedOriginal(unsigned int size);
  
  //! Individual reduced points to use in show if requested
  TripleArray<float> getXYZReducedShow();
  
  //! Create a new set of reduced points for use in show
  TripleArray<float> createXYZReducedShow(unsigned int size);
  
  //! Cached tree structure for show
  DataPointer getOcttree();
  
  //! Create a cached tree structure for show
  DataPointer createOcttree(unsigned int size);
  
  //! ScanHandler related prefetching values to combine loading of separate cache objects
  void prefetch(unsigned int type) { m_prefetch |= type; }
  
  //! Return prefetch values
  unsigned int getPrefetch() const { return m_prefetch; }
  
  //! Clear prefetch values
  void clearPrefetch() { m_prefetch = 0; }
  
  // IO-specific getters
  inline const char* getDirPath() const { return m_dir_path_ptr->c_str(); }
  inline const char* getIdentifier() const { return m_io_identifier.c_str(); }
  inline IOType getIOType() const { return m_iotype; }
  inline float getMaxDist() const { return m_max_dist; }
  inline float getMinDist() const { return m_min_dist; }
  inline double getRangeMutator() const { return m_range_mutator_param; }
  inline double getHeightTop() const { return m_height_top; }
  inline double getHeightBottom() const { return m_height_bottom; }
  
  //! Assembles an PointFilter with range/height parameters (if set) to use process-locally
  PointFilter getPointFilter() const;

private:
  SharedStringSharedPtr m_dir_path_ptr;
  SharedString m_io_identifier;
  IOType m_iotype;
  unsigned int m_prefetch;
  double m_max_dist, m_min_dist;
  double m_height_top, m_height_bottom;
  double m_range_mutator_param;
  bool m_range_mutator_param_set, m_range_param_set, m_height_param_set;
  SharedString m_reduction_parameters;
  SharedString m_show_parameters;
  SharedString m_octtree_parameters;
  bool m_load_frames_file;

protected:
  ip::offset_ptr<double> m_pose;
  ip::offset_ptr<CacheObject> m_xyz, m_rgb, m_reflectance, m_temperature, m_amplitude, m_type, m_deviation,
    m_xyz_reduced, m_xyz_reduced_original,
    m_show_reduced, m_octtree;
  FrameVector m_frames;

  //! invalidate full cache objects
  void invalidateFull();

  //! invalidate reduced cache objects
  void invalidateReduced();

  //! invalidate show related cache objects
  void invalidateShow();

private:
  //! Static callback for cache misses to send a request to the server interface
  static void onCacheMiss(CacheObject* obj);

  //! Static callback for cache object creation calls
  static void onAllocation(CacheObject* obj, unsigned int size);

  //! Static callback for cache object invalidation
  static void onInvalidation(CacheObject* obj);
};

#endif //SHARED_SCAN_H
