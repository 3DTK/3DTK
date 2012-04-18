/** @file 
 *  @brief An optimized k-d tree implementation. MetaScan variant.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Thomas Escher
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
KDParams KDtreeMeta::params[MAX_OPENMP_NUM_THREADS];

KDtreeMeta::KDtreeMeta()
{
}

/**
 * Create a KD tree from the points pointed to by the array pts
 *
 * @param pts 3D array of points
 * @param n number of points
 */
void KDtreeMeta::create(const DataXYZ* const* pts, Index* indices, unsigned int n)
{
  // Find bbox
  double xmin = point(pts, indices[0])[0], xmax = point(pts, indices[0])[0];
  double ymin = point(pts, indices[0])[1], ymax = point(pts, indices[0])[1];
  double zmin = point(pts, indices[0])[2], zmax = point(pts, indices[0])[2];
  for(unsigned int i = 1; i < n; i++) {
    xmin = min(xmin, point(pts, indices[i])[0]);
    xmax = max(xmax, point(pts, indices[i])[0]);
    ymin = min(ymin, point(pts, indices[i])[1]);
    ymax = max(ymax, point(pts, indices[i])[1]);
    zmin = min(zmin, point(pts, indices[i])[2]);
    zmax = max(zmax, point(pts, indices[i])[2]);
  }

  // Leaf nodes
  if ((n > 0) && (n <= 10)) {
    npts = n;
    leaf.p = new Index[n];
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
    leaf.p = new Index[n];
    // fill leaf index array with indices
    for(unsigned int i = 0; i < n; ++i) {
      leaf.p[i] = indices[i];
    }
    return;
  }

  Index* left = indices, * right = indices + n - 1;
  while(true) {
    while(point(pts, *left)[node.splitaxis] < splitval)
      left++;
    while(point(pts, *right)[node.splitaxis] >= splitval)
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
    if (i == 0) {
      node.child1 = new KDtreeMeta();
      node.child1->create(pts, indices, left - indices);
    }
    if (i == 1) {
      node.child2 = new KDtreeMeta();
      node.child2->create(pts, left, n - (left - indices));
    }
  }
}

KDtreeMeta::~KDtreeMeta()
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
void KDtreeMeta::_FindClosest(const DataXYZ* const* pts, int threadNum) const
{
  // Leaf nodes
  if (npts) {
    for (int i = 0; i < npts; i++) {
      double myd2 = Dist2(params[threadNum].p, point(pts, leaf.p[i]));
      if (myd2 < params[threadNum].closest_d2) {
        params[threadNum].closest_d2 = myd2;
        params[threadNum].closest = point(pts, leaf.p[i]);
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

KDtreeMeta::Index* KDtreeMetaManaged::prepareTempIndices(const vector<Scan*>& scans)
{
  unsigned int n = getPointsSize(scans);

  m_temp_indices = new Index[n];
  unsigned int s = 0, j = 0;
  for(unsigned int i = 0; i < n; ++i) {
    m_temp_indices[i].set(s, j++);
    // switch to next scan
    if(j >= scans[s]->size<DataXYZ>("xyz reduced")) {
      ++s;
      j = 0;
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

double* KDtreeMetaManaged::FindClosest(double *_p, double maxdist2, int threadNum) const
{
  params[threadNum].closest = 0;
  params[threadNum].closest_d2 = maxdist2;
  params[threadNum].p = _p;
  _FindClosest(m_data, threadNum);
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
