/*
 * kdMeta implementation
 *
 * Copyright (C) Andreas Nuechter, Kai Lingemann, Thomas Escher
 *
 * Released under the GPL version 3.
 *
 */

/** @file 
 *  @brief An optimized k-d tree implementation. MetaScan variant.
 *  @author Andreas Nuechter. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Thomas Escher. Inst. of CS, University of Osnabrueck, Germany.
 */

#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#endif

#include "slam6d/kdMeta.h"
#include "slam6d/globals.icc"
#include "slam6d/scan.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <algorithm>
using std::swap;
#include <cmath>
#include <cstring>

// KDtree class static variables
template<class PointData, class AccessorData, class AccessorFunc>
KDParams KDTreeImpl<PointData, AccessorData, AccessorFunc>::params[MAX_OPENMP_NUM_THREADS];

KDtreeMetaManaged::KDtreeMetaManaged(const vector<Scan*>& scans) :
  m_count_locking(0)
{
  // create scan pointer and data pointer arrays
  m_size = scans.size();
  m_scans = new Scan*[m_size];
  for(unsigned int i = 0; i < m_size; ++i)
    m_scans[i] = scans[i];
  m_data = new DataXYZ*[m_size];
  
  lock();
  create(m_data, prepareTempIndices(scans), getPointsSize(scans));
  unlock();
  
  // allocate in prepareTempIndices, deleted here
  delete[] m_temp_indices;
}

KDtreeMetaManaged::~KDtreeMetaManaged()
{
  delete[] m_scans;
  delete[] m_data;
}

Index* KDtreeMetaManaged::prepareTempIndices(const vector<Scan*>& scans)
{
  unsigned int n = getPointsSize(scans);

  m_temp_indices = new Index[n];
  unsigned int s = 0, j = 0;
  unsigned int scansize = scans[s]->size<DataXYZ>("xyz reduced");
  for(unsigned int i = 0; i < n; ++i) {
    m_temp_indices[i].set(s, j++);
    // switch to next scan
    if(j >= scansize) {
      ++s;
      j = 0;
      if(s < scans.size())
        scansize = scans[s]->size<DataXYZ>("xyz reduced");
    }
  }
  return m_temp_indices;
}

unsigned int KDtreeMetaManaged::getPointsSize(const vector<Scan*>& scans)
{
  unsigned int n = 0;
  for(vector<Scan*>::const_iterator it = scans.begin(); it != scans.end(); ++it) {
    n += (*it)->size<DataXYZ>("xyz reduced");
  }
  return n;
}

double* KDtreeMetaManaged::FindClosest(double *_p,
                                       double maxdist2,
                                       int threadNum) const
{
  params[threadNum].closest = 0;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  _FindClosest(m_data, threadNum);
  return params[threadNum].closest;
}

double* KDtreeMetaManaged::FindClosestAlongDir(double *_p,
                                               double *_dir,
                                               double maxdist2,
                                               int threadNum) const
{
  params[threadNum].closest = NULL;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  params[threadNum].dir = _dir;
  _FindClosestAlongDir(m_data, threadNum);
  return params[threadNum].closest;
}

void KDtreeMetaManaged::lock()
{
  boost::lock_guard<boost::mutex> lock(m_mutex_locking);
  if(m_count_locking == 0) {
    // lock all the contained scans, metascan uses the transformed points
    for(unsigned int i = 0; i < m_size; ++i) {
      m_data[i] = new DataXYZ(m_scans[i]->get("xyz reduced"));
    }
  }
  ++m_count_locking;
}

void KDtreeMetaManaged::unlock()
{
  boost::lock_guard<boost::mutex> lock(m_mutex_locking);
  --m_count_locking;
  if(m_count_locking == 0) {
    // delete each locking object
    for(unsigned int i = 0; i < m_size; ++i) {
      delete m_data[i];
    }
  }
}
