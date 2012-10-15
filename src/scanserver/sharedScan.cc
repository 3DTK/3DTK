/*
 * sharedScan implementation
 *
 * Copyright (C) Thomas Escher, Kai Lingemann
 *
 * Released under the GPL version 3.
 *
 */

#include "scanserver/sharedScan.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <string>

#include "scanserver/clientInterface.h"



SharedScan::SharedScan(const ip::allocator<void, SegmentManager> & allocator,
  const SharedStringSharedPtr& dir_path_ptr, const char *io_identifier,
  IOType iotype) :
  m_dir_path_ptr(dir_path_ptr),
  m_io_identifier(io_identifier, allocator),
  m_iotype(iotype),
  m_prefetch(0),
  m_max_dist(0.0), m_min_dist(0.0),
  m_height_top(0.0), m_height_bottom(0.0),
  m_range_mutator_param(0.0),
  m_range_mutator_param_set(false),
  m_range_param_set(false), m_height_param_set(false),
  m_reduction_parameters(allocator),
  m_show_parameters(allocator),
  m_octtree_parameters(allocator),
  m_load_frames_file(true),
  m_frames(allocator)
{
  // until boost-1.47 ipc-strings can do garbage with g++-4.4 -O2 and higher (optimizations and versions)
  std::string bugfix(io_identifier);
  m_io_identifier[bugfix.length()] = '\0';

  // COs are allocated in the ServerScan-ctor
}

SharedScan::~SharedScan()
{
  // TODO
  // get segment manager and deconstruct all available data fields
  // TODO: Delete COs if notzero and notify CM?
}

bool SharedScan::operator==(const SharedScan& r) const
{
  return (m_io_identifier == r.m_io_identifier) && (*m_dir_path_ptr == *r.m_dir_path_ptr) && m_iotype == r.m_iotype;
}

void SharedScan::setRangeParameters(double max_dist, double min_dist)
{
  // if a non-first set differs from the previous ones, invalidate all COs
  if(m_range_param_set) {
    if(max_dist != m_max_dist || min_dist != m_min_dist) {
      invalidateFull();
      invalidateReduced();
      invalidateShow();
    }
  }
  m_max_dist = max_dist;
  m_min_dist = min_dist;
  m_range_param_set = true;
}

void SharedScan::setHeightParameters(double top, double bottom)
{
  // if a non-first set differs from the previous ones, invalidate all COs
  if(m_height_param_set) {
    if(top != m_height_top || bottom != m_height_bottom) {
      invalidateFull();
      invalidateReduced();
      invalidateShow();
    }
  }
  m_height_top = top;
  m_height_bottom = bottom;
  m_height_param_set = true;
}

void SharedScan::setRangeMutationParameters(double range)
{
  // if a non-first set differs from the previous ones, invalidate all COs
  if(m_range_mutator_param_set) {
    if(range != m_range_mutator_param) {
      invalidateFull();
      invalidateReduced();
      invalidateShow();
    }
  }
  m_range_mutator_param = range;
  m_range_mutator_param_set = true;
}

void SharedScan::setReductionParameters(const char* params)
{
  // if a non-first set differs from the previous ones, invalidate reduced COs
  if(!m_reduction_parameters.empty() && m_reduction_parameters != params) {
    invalidateReduced();
  }
  m_reduction_parameters = params;
}

void SharedScan::setShowReductionParameters(const char* params)
{
  // if a non-first set differs from the previous ones, invalidate reduced COs
  if(!m_show_parameters.empty() && m_show_parameters != params) {
    invalidateShow();
  }
  m_show_parameters = params;
}

void SharedScan::setOcttreeParameters(const char* params)
{
  // if a non-first set differs from the previous ones, invalidate reduced COs
  if(!m_octtree_parameters.empty() && m_octtree_parameters != params) {
    m_octtree->invalidate<SharedScan::onInvalidation>();
  }
  m_octtree_parameters = params;
}

PointFilter SharedScan::getPointFilter() const
{
  PointFilter r;
  if(m_range_param_set)
    r.setRange(m_max_dist, m_min_dist);
  if(m_height_param_set)
    r.setHeight(m_height_top, m_height_bottom);
  if(m_range_mutator_param_set)
    r.setRangeMutator(m_range_mutator_param);
  
  return r;
}

void SharedScan::invalidateFull()
{
  m_xyz->invalidate<SharedScan::onInvalidation>();
  m_rgb->invalidate<SharedScan::onInvalidation>();
}

void SharedScan::invalidateReduced()
{
  m_xyz_reduced->invalidate<SharedScan::onInvalidation>();
  m_xyz_reduced_original->invalidate<SharedScan::onInvalidation>();
}

void SharedScan::invalidateShow()
{
  m_show_reduced->invalidate<SharedScan::onInvalidation>();
  m_octtree->invalidate<SharedScan::onInvalidation>();
}

void SharedScan::clearFrames()
{
  ClientInterface* client = ClientInterface::getInstance();
  client->clearFrames(this);
  // don't try to load again from the still existing files
  m_load_frames_file = false;
}

void SharedScan::addFrame(double* transformation, unsigned int type)
{
  ClientInterface* client = ClientInterface::getInstance();
  client->addFrame(this, transformation, type);
}

const FrameVector& SharedScan::getFrames()
{
  // on a restart with existing frame files try to load these
  if(m_frames.empty() && m_load_frames_file == true) {
    ClientInterface* client = ClientInterface::getInstance();
    client->loadFramesFile(this);
    // don't try to load again if frames are still empty
    m_load_frames_file = false;
  }
  return m_frames;
}

void SharedScan::saveFrames()
{
  ClientInterface* client = ClientInterface::getInstance();
  client->saveFramesFile(this);
  // we just saved the file, no need to read it
  m_load_frames_file = false;
}

double* SharedScan::getPose()
{
  if(m_pose == 0) {
    ClientInterface* client = ClientInterface::getInstance();
    client->getPose(this);
  }
  return m_pose.get();
}

DataXYZ SharedScan::getXYZ() {
  return m_xyz.get()->getCacheData<SharedScan::onCacheMiss>();
}

DataRGB SharedScan::getRGB() {
  return m_rgb->getCacheData<SharedScan::onCacheMiss>();
}

DataReflectance SharedScan::getReflectance() {
  return m_reflectance->getCacheData<SharedScan::onCacheMiss>();
}

DataTemperature SharedScan::getTemperature() {
  return m_temperature->getCacheData<SharedScan::onCacheMiss>();
}

DataAmplitude SharedScan::getAmplitude() {
  return m_amplitude->getCacheData<SharedScan::onCacheMiss>();
}

DataType SharedScan::getType() {
  return m_type->getCacheData<SharedScan::onCacheMiss>();
}

DataDeviation SharedScan::getDeviation() {
  return m_deviation->getCacheData<SharedScan::onCacheMiss>();
}

DataXYZ SharedScan::getXYZReduced() {
  return m_xyz_reduced->getCacheData<SharedScan::onCacheMiss>();
}

DataXYZ SharedScan::createXYZReduced(unsigned int size) {
  // size is in units of double[3], scale to bytes
  return m_xyz_reduced->createCacheData<SharedScan::onAllocation>(size*3*sizeof(double));
}


DataReflectance SharedScan::createReflectance(unsigned int size) {
  // size is in units of double[1], scale to bytes
  return m_reflectance->createCacheData<SharedScan::onAllocation>(size*1*sizeof(double));
}


DataXYZ SharedScan::getXYZReducedOriginal() {
  return m_xyz_reduced_original->getCacheData<SharedScan::onCacheMiss>();
}

DataXYZ SharedScan::createXYZReducedOriginal(unsigned int size) {
  // size is in units of double[3], scale to bytes
  return m_xyz_reduced_original->createCacheData<SharedScan::onAllocation>(size*3*sizeof(double));
}

TripleArray<float> SharedScan::getXYZReducedShow() {
  return m_show_reduced->getCacheData<SharedScan::onCacheMiss>();
}

TripleArray<float> SharedScan::createXYZReducedShow(unsigned int size) {
  return m_show_reduced->createCacheData<SharedScan::onAllocation>(size*3*sizeof(float));
}

DataPointer SharedScan::getOcttree() {
  return m_octtree->getCacheData<SharedScan::onCacheMiss>();
}

DataPointer SharedScan::createOcttree(unsigned int size) {
  return m_octtree->createCacheData<SharedScan::onAllocation>(size);
}

void SharedScan::onCacheMiss(CacheObject* obj)
{
  ClientInterface* client = ClientInterface::getInstance();
  client->loadCacheObject(obj);
}

void SharedScan::onAllocation(CacheObject* obj, unsigned int size)
{
  ClientInterface* client = ClientInterface::getInstance();
  client->allocateCacheObject(obj, size);
}

void SharedScan::onInvalidation(CacheObject* obj)
{
  ClientInterface* client = ClientInterface::getInstance();
  client->invalidateCacheObject(obj);
}
