/**
 * @file
 * @brief 
 *
 * @author Thomas Escher
 */

#ifndef TEMPORARY_HANDLER_H
#define TEMPORARY_HANDLER_H

#include "scanserver/cache/cacheHandler.h"
#include "scanserver/sharedScan.h"
#include "scanserver/cacheIO.h"



/**
 * @brief CacheHandler for artificially created CacheObjects by reduction in SharedScan.
 *
 * This handler saves and loads the contents of a CacheObject with arbitrary contents via CacheIO.
 */
class TemporaryHandler : public CacheHandler
{
public:
  /**
   * Constructor
   * @param static_data determines overwriting policy. Set false for changing data, true for static write-only-once data.
   */
  TemporaryHandler(CacheObject* obj, CacheManager* cm, SharedScan* scan, bool static_data = false);

  /**
   * Deserialize data from a file if it exists and written flag is set, otherwise does nothing
   * @return whether written flag was set and file was found and loaded
   */
  virtual bool load();

  /**
   * Serialize all data into a file
   * It will do so if either the written flag isn't set, or static data flag isn't set regardless of the written flag.
   */
  virtual void save(unsigned char* data, unsigned int size);

  //! Reset flag for having a cached file, causing reads to fail and saves to overwrite older files.
  virtual void invalidate() { m_written = false; }
protected:
  SharedScan* m_scan;
private:
  CacheIO::IDType m_id;
  bool m_written, m_static_data;
};

#endif //TEMPORARY_HANDLER_H
