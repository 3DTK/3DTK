/*
 * kdManaged implementation
 *
 * Copyright (C) Andreas Nuechter, Kai Lingemann, Thomas Escher
 *
 * Released under the GPL version 3.
 *
 */

/** @file 
 *  @brief An optimized k-d tree implementation
 *  @author Andreas Nuechter. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Thomas Escher. Inst. of CS, University of Osnabrueck, Germany.
 */

#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#endif

#include "slam6d/kdManaged.h"
#include "slam6d/scan.h"
#include "slam6d/globals.icc"

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

KDtreeManaged::KDtreeManaged(Scan* scan) :
  m_scan(scan), m_data(0), m_count_locking(0)
{
  create(scan->get("xyz reduced original"),
         prepareTempIndices(scan->size<DataXYZ>("xyz reduced original")),
         scan->size<DataXYZ>("xyz reduced original"));
  // allocate in prepareTempIndices, deleted here
  delete[] m_temp_indices;
}

unsigned int* KDtreeManaged::prepareTempIndices(unsigned int n)
{
  m_temp_indices = new unsigned int[n];
  for(unsigned int i = 0; i < n; ++i)
    m_temp_indices[i] = i;
  return m_temp_indices;
}

double* KDtreeManaged::FindClosest(double *_p,
                                   double maxdist2,
                                   int threadNum) const
{
  params[threadNum].closest = 0;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  _FindClosest(*m_data, threadNum);
  return params[threadNum].closest;
}

double* KDtreeManaged::FindClosestAlongDir(double *_p,
                                           double *_dir,
                                           double maxdist2, int
                                           threadNum) const
{
  params[threadNum].closest = NULL;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  params[threadNum].dir = _dir;
  _FindClosestAlongDir(*m_data, threadNum);
  return params[threadNum].closest;
}

void KDtreeManaged::lock()
{
  boost::lock_guard<boost::mutex> lock(m_mutex_locking);
  ++m_count_locking;
  if(m_data == 0) {
    // aquire an array lock from the scan and hold it while the tree is in use
    m_data = new DataXYZ(m_scan->get("xyz reduced original"));
  }
}

void KDtreeManaged::unlock()
{
  boost::lock_guard<boost::mutex> lock(m_mutex_locking);
  --m_count_locking;
  if(m_count_locking == 0 && m_data != 0) {
    delete m_data;
    m_data = 0;
  }
}
