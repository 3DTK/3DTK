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
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Thomas Escher. Institute of Computer Science, University of Osnabrueck, Germany.
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
KDParams KDtreeManagedBase::params[MAX_OPENMP_NUM_THREADS];

/**
 * Constructor
 *
 * Create a KD tree from the points pointed to by the array pts
 *
 * @param pts 3D array of points
 * @param n number of points
 */
KDtreeManagedBase::KDtreeManagedBase(const DataXYZ& pts, unsigned int* indices, unsigned int n)
{
  // Find bbox
  double xmin = pts[indices[0]][0], xmax = pts[indices[0]][0];
  double ymin = pts[indices[0]][1], ymax = pts[indices[0]][1];
  double zmin = pts[indices[0]][2], zmax = pts[indices[0]][2];
  for(unsigned int i = 1; i < n; i++) {
    xmin = min(xmin, pts[indices[i]][0]);
    xmax = max(xmax, pts[indices[i]][0]);
    ymin = min(ymin, pts[indices[i]][1]);
    ymax = max(ymax, pts[indices[i]][1]);
    zmin = min(zmin, pts[indices[i]][2]);
    zmax = max(zmax, pts[indices[i]][2]);
  }
  
  // Leaf nodes
  if ((n > 0) && (n <= 10)) {
    npts = n;
    leaf.p = new unsigned int[n];
    // fill leaf index array with indices
    for(unsigned int i = 0; i < n; ++i) {
      leaf.p[i] = indices[i];
    }
    return;
  }
  
  // Else, interior nodes
  npts = 0;
  
  node.center[0] = 0.5 * (xmin+xmax);
  node.center[1] = 0.5 * (ymin+ymax);
  node.center[2] = 0.5 * (zmin+zmax);
  node.dx = 0.5 * (xmax-xmin);
  node.dy = 0.5 * (ymax-ymin);
  node.dz = 0.5 * (zmax-zmin);
  node.r2 = sqr(node.dx) + sqr(node.dy) + sqr(node.dz);

  // Find longest axis
  if (node.dx > node.dy) {
    if (node.dx > node.dz) {
      node.splitaxis = 0;
    } else {
      node.splitaxis = 2;
    }
  } else {
    if (node.dy > node.dz) {
      node.splitaxis = 1;
    } else {
      node.splitaxis = 2;
    }
  }

  // Partition
  double splitval = node.center[node.splitaxis];

  if ( fabs(max(max(node.dx,node.dy),node.dz)) < 0.01 ) {
    npts = n;
    leaf.p = new unsigned int[n];
    // fill leaf index array with indices
    for(unsigned int i = 0; i < n; ++i) {
      leaf.p[i] = indices[i];
    }
    return;
  }

  unsigned int* left = indices, * right = indices + n - 1;
  while(true) {
    while(pts[*left][node.splitaxis] < splitval)
      left++;
    while(pts[*right][node.splitaxis] >= splitval)
      right--;
    if(right < left)
      break;
    swap(*left, *right);
  }
  
  // Build subtrees
  int i;
#ifdef WITH_OPENMP_KD                   // does anybody know the reason why this is slower ?? --Andreas
  omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for schedule(dynamic) 
#endif
  for (i = 0; i < 2; i++) {
    if (i == 0) node.child1 = new KDtreeManagedBase(pts,	indices, left - indices);
    if (i == 1) node.child2 = new KDtreeManagedBase(pts, left, n - (left - indices));
  }
}

KDtreeManagedBase::~KDtreeManagedBase()
{
  if (!npts) {
#ifdef WITH_OPENMP_KD
    omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < 2; i++) {
      if (i == 0 && node.child1) delete node.child1;
      if (i == 1 && node.child2) delete node.child2;
    }
  } else {
      if (leaf.p) delete [] leaf.p;
  }
}

/**
 * Wrapped function 
 */
void KDtreeManagedBase::_FindClosest(const DataXYZ& pts, int threadNum) const
{
  // Leaf nodes
  if (npts) {
    for (int i = 0; i < npts; i++) {
      double myd2 = Dist2(params[threadNum].p, pts[leaf.p[i]]);
      if (myd2 < params[threadNum].closest_d2) {
        params[threadNum].closest_d2 = myd2;
        params[threadNum].closest = pts[leaf.p[i]];
      }
    }
    return;
  }

  // Quick check of whether to abort  
  double approx_dist_bbox = max(max(fabs(params[threadNum].p[0]-node.center[0])-node.dx,
							 fabs(params[threadNum].p[1]-node.center[1])-node.dy),
						  fabs(params[threadNum].p[2]-node.center[2])-node.dz);
  if (approx_dist_bbox >= 0 && sqr(approx_dist_bbox) >= params[threadNum].closest_d2)
    return;

  // Recursive case
  double myd = node.center[node.splitaxis] - params[threadNum].p[node.splitaxis];
  if (myd >= 0.0) {
    node.child1->_FindClosest(pts, threadNum);
    if (sqr(myd) < params[threadNum].closest_d2) {
      node.child2->_FindClosest(pts, threadNum);
    }
  } else {
    node.child2->_FindClosest(pts, threadNum);
    if (sqr(myd) < params[threadNum].closest_d2) {
      node.child1->_FindClosest(pts, threadNum);
    }
  }
}

KDtreeManaged::KDtreeManaged(Scan* scan) :
  KDtreeManagedBase(scan->get("xyz reduced original"), prepareTempIndices(scan->size<DataXYZ>("xyz reduced original")), scan->size<DataXYZ>("xyz reduced original")),
  m_scan(scan), m_data(0), m_count_locking(0)
{
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

double* KDtreeManaged::FindClosest(double *_p, double maxdist2, int threadNum) const
{
  params[threadNum].closest = 0;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  _FindClosest(*m_data, threadNum);
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
