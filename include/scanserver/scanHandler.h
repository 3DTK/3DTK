/**
 * @file
 * @brief 
 *
 * @author Thomas Escher
 * @author Dorit Borrmann
 */

#ifndef SCAN_HANDLER_H
#define SCAN_HANDLER_H

#include "scanserver/temporaryHandler.h"
#include <map>
#include <vector>


/**
 * @brief CacheHandler for scan files.
 *
 * This class handles scan files. On a cache miss it reads from the original scan file by calling ScanIO to load the proper library for input.
 * If binary scan caching is enabled the TemporaryHandler functionality is invoked in saves and, if binary scan caching is enabled, also on loads. In the latter case the binary cache file has priority over parsing the scan anew. Invalidation is taken into consideration, reloading the scan with new range parameters.
 */
class ScanHandler : public TemporaryHandler
{
public:
  ScanHandler(CacheObject* obj, CacheManager* cm, SharedScan* scan, IODataType data);
  
  /**
   * Reads specific scan data from a file, located by SharedScan's identifiers via ScanIO.
   * If binary caching is enabled it will try to read from this resource first to speed up the process.
   * @return true, will throw otherwise because we are desperate for scans
   * @throw if both the binary cache and the scan resource were unavailable
   */
  virtual bool load();
  
  /**
   * Does nothing unless binary caching is enabled, which will save the contents via CacheIO.
   */
  virtual void save(unsigned char* data, unsigned int size);
  
  //! Enable binary caching of scan data
  static void setBinaryCaching();
private:
  IODataType m_data;
  
  static bool binary_caching;

  //! Cached vectors for prefetching
  static std::map<SharedScan*, std::vector<double>* > m_prefetch_xyz;
  static std::map<SharedScan*, std::vector<unsigned char>* > m_prefetch_rgb;
  static std::map<SharedScan*, std::vector<float>* > m_prefetch_reflectance;
  static std::map<SharedScan*, std::vector<float>* > m_prefetch_temperature;
  static std::map<SharedScan*, std::vector<float>* > m_prefetch_amplitude;
  static std::map<SharedScan*, std::vector<int>* > m_prefetch_type;
  static std::map<SharedScan*, std::vector<float>* > m_prefetch_deviation;
};

#endif //SCAN_HANDLER_H
