/*
 * serverScan implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/serverScan.h"

#include "scanserver/cache/cacheManager.h"
#include "scanserver/scanHandler.h"
#include "scanserver/temporaryHandler.h"

ServerScan::ServerScan(const ip::allocator<void, SegmentManager> & allocator,
    const SharedStringSharedPtr& dir_path_ptr, const char* io_identifier,
    IOType iotype, CacheManager* cm) :
  SharedScan(allocator, dir_path_ptr, io_identifier, iotype)
{
  // let the manager allocate cache objects in shared memory and assign them process local handlers, called through the interface
  m_xyz = cm->createCacheObject();
  m_xyz->setCacheHandler(new ScanHandler(m_xyz.get(), cm, this, DATA_XYZ));
  m_rgb = cm->createCacheObject();
  m_rgb->setCacheHandler(new ScanHandler(m_rgb.get(), cm, this, DATA_RGB));
  m_reflectance = cm->createCacheObject();
  m_reflectance->setCacheHandler(new ScanHandler(m_reflectance.get(), cm, this, DATA_REFLECTANCE));
  m_temperature = cm->createCacheObject();
  m_temperature->setCacheHandler(new ScanHandler(m_temperature.get(), cm, this, DATA_TEMPERATURE));
  m_amplitude = cm->createCacheObject();
  m_amplitude->setCacheHandler(new ScanHandler(m_amplitude.get(), cm, this, DATA_AMPLITUDE));
  m_type = cm->createCacheObject();
  m_type->setCacheHandler(new ScanHandler(m_type.get(), cm, this, DATA_TYPE));
  m_deviation = cm->createCacheObject();
  m_deviation->setCacheHandler(new ScanHandler(m_deviation.get(), cm, this, DATA_DEVIATION));
  
  m_xyz_reduced = cm->createCacheObject();
  m_xyz_reduced->setCacheHandler(new TemporaryHandler(m_xyz_reduced.get(), cm, this));
  m_xyz_reduced_original = cm->createCacheObject();
  m_xyz_reduced_original->setCacheHandler(new TemporaryHandler(m_xyz_reduced_original.get(), cm, this, true));
  
  m_show_reduced = cm->createCacheObject();
  m_show_reduced->setCacheHandler(new TemporaryHandler(m_show_reduced.get(), cm, this, true));
  m_octtree = cm->createCacheObject();
  m_octtree->setCacheHandler(new TemporaryHandler(m_octtree.get(), cm, this, true));
}
