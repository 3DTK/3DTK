/** @file 
 *  @brief Representation of the optimized k-d tree.
 *  @author Remus Dumitru. Jacobs University Bremen, Germany
 *  @author Corneliu-Claudiu Prodescu. Jacobs University Bremen, Germany
 *  @author Andreas Nuechter. Jacobs University Bremen, Germany
 *  @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany
 *  @author Momchil Ivanov Ivanov. Jacobs University Bremen, Germany
 *  @author Igor Pener. Jacobs University Bremen, Germany
 */

#ifndef __KD_TREE_IMPL_H__
#define __KD_TREE_IMPL_H__

#include "slam6d/kdparams.h"
#include "globals.icc"

#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

class PointCompare {
public:
    bool operator() (const std::pair<Point, double>& left,
				 const std::pair<Point, double>& right)
  {
    return left.second > right.second;
  }
};

/**
 * @brief The optimized k-d tree. 
 * 
 * A kD tree for points, with limited
 * capabilities (find nearest point to
 * a given point, or to a ray).
 *
 * PointData    the type of the input point data
 * AccessorData the type of indices
 * AccessorFunc retrieves data of type double[3], given an index of type
 *              AccessorData and the data of type PointData
 * PointType    the type that is stored in kdparams
 * ParamFunc    retrieves data of type PointType, given an index of type
 *              AccessorData and the data of type PointData
 **/
template<class PointData, class AccessorData, class AccessorFunc, class PointType, class ParamFunc>
class KDTreeImpl {
public:
  inline KDTreeImpl() { }
  
  virtual inline ~KDTreeImpl() {
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

  virtual void create(PointData pts, AccessorData *indices, size_t n) {
    AccessorFunc point;
    
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
      leaf.p = new AccessorData[n];
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
    node.r = sqrt(sqr(node.dx) + sqr(node.dy) + sqr(node.dz));

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
      leaf.p = new AccessorData[n];
      // fill leaf index array with indices
      for(unsigned int i = 0; i < n; ++i) {
        leaf.p[i] = indices[i];
      }
      return;
    }

    AccessorData* left = indices, * right = indices + n - 1;
    while(true) {
      while(point(pts, *left)[node.splitaxis] < splitval)
        left++;
      while(point(pts, *right)[node.splitaxis] >= splitval)
        right--;
      if(right < left)
        break;
      std::swap(*left, *right);
    }

    // Build subtrees
    int i;
#ifdef WITH_OPENMP_KD                   // does anybody know the reason why this is slower ?? --Andreas
    omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for schedule(dynamic) 
#endif
    for (i = 0; i < 2; i++) {
      if (i == 0) {
        node.child1 = new KDTreeImpl();
        node.child1->create(pts, indices, left - indices);
      }
      if (i == 1) {
        node.child2 = new KDTreeImpl();
        node.child2->create(pts, left, n - (left - indices));
      }
    }
  }

protected:
  /**
   * storing the parameters of the k-d tree, i.e., the current closest point,
   * the distance to the current closest point and the point itself.
   * These global variable are needed in this search.
   *
   * Padded in the parallel case.
   */
#ifdef _OPENMP
#ifdef __INTEL_COMPILER
  __declspec (align(16)) static KDParams params[MAX_OPENMP_NUM_THREADS];
#else
  static KDParams<PointType> params[MAX_OPENMP_NUM_THREADS];
#endif //__INTEL_COMPILER
#else
  static KDParams<PointType> params[MAX_OPENMP_NUM_THREADS];
#endif	

  /**
   * number of points. If this is 0: intermediate node. If nonzero: leaf.
   */
  int npts;
  
  /**
   * Cue the standard rant about anon unions but not structs in C++
   */
  union {
    /** 
     * in case of internal node... 
     */
    struct {	 
      double center[3]; ///< storing the center of the voxel (R^3)
      double dx,  ///< defining the voxel itself
	        dy,  ///< defining the voxel itself
	        dz,  ///< defining the voxel itself
             r;  ///< defining the voxel itself
      int splitaxis;   ///< defining the kind of splitaxis
      KDTreeImpl *child1;  ///< pointers to the childs
      KDTreeImpl *child2;  ///< pointers to the childs
    } node;
    /** 
     * in case of leaf node ... 
     */
    struct {
      /** 
       * store the value itself
       * Here we store just a pointer to the data
       */
      AccessorData* p;
    } leaf;
  };

  void _FindClosest(const PointData& pts, int threadNum) const {
    AccessorFunc point;
    ParamFunc   pointparam;

    // Leaf nodes
    if (npts) {
      for (int i = 0; i < npts; i++) {
        double myd2 = Dist2(params[threadNum].p, point(pts, leaf.p[i]));
        if (myd2 < params[threadNum].closest_d2) {
          params[threadNum].closest_d2 = myd2;
          params[threadNum].closest = pointparam(pts, leaf.p[i]);
        }
      }
      return;
    }

    // Quick check of whether to abort  
    double approx_dist_bbox =
	 max(max(fabs(params[threadNum].p[0]-node.center[0])-node.dx,
		    fabs(params[threadNum].p[1]-node.center[1])-node.dy),
		fabs(params[threadNum].p[2]-node.center[2])-node.dz);
    if (approx_dist_bbox >= 0 &&
	   sqr(approx_dist_bbox) >= params[threadNum].closest_d2)
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

  void _FindClosestAlongDir(const PointData& pts, int threadNum) const {
    AccessorFunc point;
    ParamFunc pointparam;

    // Leaf nodes
    if (npts) {
      for (int i=0; i < npts; i++) {
        double p2p[] =  { params[threadNum].p[0] - point(pts, leaf.p[i])[0],
                          params[threadNum].p[1] - point(pts, leaf.p[i])[1],
                          params[threadNum].p[2] - point(pts, leaf.p[i])[2] };
        double myd2 = Len2(p2p) - sqr(Dot(p2p, params[threadNum].dir));
        if ((myd2 < params[threadNum].closest_d2)) {
          params[threadNum].closest_d2 = myd2;
          params[threadNum].closest = pointparam(pts, leaf.p[i]);
        }
      }
      return;
    }


    // Quick check of whether to abort
    double p2c[] = { params[threadNum].p[0] - node.center[0],
                     params[threadNum].p[1] - node.center[1],
                     params[threadNum].p[2] - node.center[2] };
    double myd2center = Len2(p2c) - sqr(Dot(p2c, params[threadNum].dir));
    if (myd2center > sqr(node.r + params[threadNum].closest_d))
      return;


    // Recursive case
    if (params[threadNum].p[node.splitaxis] < node.center[node.splitaxis] ) {
      node.child1->_FindClosestAlongDir(pts, threadNum);
      node.child2->_FindClosestAlongDir(pts, threadNum);
    } else {
      node.child2->_FindClosestAlongDir(pts, threadNum);
      node.child1->_FindClosestAlongDir(pts, threadNum);
    }
  }

  void _fixedRangeSearchBetween2Points(const PointData& pts, int threadNum) const {
    AccessorFunc point;
    ParamFunc pointparam;
    
    // Leaf nodes
    if (npts) {
	    for (int i = 0; i < npts; i++) {
        double p2p[] =  { params[threadNum].p[0] - point(pts, leaf.p[i])[0],
                          params[threadNum].p[1] - point(pts, leaf.p[i])[1],
                          params[threadNum].p[2] - point(pts, leaf.p[i])[2] };
        double myd2 = Len2(p2p) - sqr(Dot(p2p, params[threadNum].dir));
        if (myd2 < params[threadNum].closest_d2) {
		    //  cout << point(pts, leaf.p[i])[0] << " " << point(pts, leaf.p[i])[1] << " " << point(pts, leaf.p[i])[2] << " " << myd2 << endl;
          params[threadNum].range_neighbors.push_back(pointparam(pts, leaf.p[i]));
	      }
	    }
	    return;
    }
    
    // Quick check of whether to abort
    double c2c[] = { params[threadNum].p[0] - node.center[0],
                     params[threadNum].p[1] - node.center[1],
                     params[threadNum].p[2] - node.center[2] };
    
    double my_dist_2 = Len2(c2c); // Distance^2 camera node center
    double myd2center = my_dist_2 - sqr(Dot(c2c, params[threadNum].dir));
    //if (myd2center > (node.r2 + params[threadNum].closest_d2 + 2.0f * max(node.r2, params[threadNum].closest_d2)))
    
    if (myd2center > sqr(node.r + params[threadNum].closest_d))
      return;
    //if (myd2center > (node.r2 + params[threadNum].closest_d2 + 2.0f * sqrt(node.r2) * sqrt(params[threadNum].closest_d2))) return;

    // check if not between points
    
    double p2c[] = { params[threadNum].p0[0] - node.center[0],
                     params[threadNum].p0[1] - node.center[1],
                     params[threadNum].p0[2] - node.center[2] };

    double distXP2 = Len2(p2c);
    if(params[threadNum].dist_2 > sqr(distXP2 + node.r)) return;
    
    if(params[threadNum].dist_2 > sqr(sqrt(my_dist_2) + node.r)) return;
    
    // Recursive case
    if (params[threadNum].p[node.splitaxis] < node.center[node.splitaxis] ) {
      node.child1->_fixedRangeSearchAlongDir(pts, threadNum);
      node.child2->_fixedRangeSearchAlongDir(pts, threadNum);
    } else {
      node.child2->_fixedRangeSearchAlongDir(pts, threadNum);
      node.child1->_fixedRangeSearchAlongDir(pts, threadNum);
    }
  
  }

  
  void _fixedRangeSearchAlongDir(const PointData& pts, int threadNum) const {
    AccessorFunc point;
    ParamFunc pointparam;
    
    // Leaf nodes
    if (npts) {
	    for (int i = 0; i < npts; i++) {
        /*
        double p2pb[] =  { point(pts, leaf.p[i])[0] - params[threadNum].p[0],
                          point(pts, leaf.p[i])[1] - params[threadNum].p[1],
                          point(pts, leaf.p[i])[2] - params[threadNum].p[2]};
        double blub[3];
        Cross(p2pb, params[threadNum].dir, blub);
        double myd2b = Len2(blub) / Len2(params[threadNum].dir);
	      */
        
        double p2p[] =  { params[threadNum].p[0] - point(pts, leaf.p[i])[0],
                          params[threadNum].p[1] - point(pts, leaf.p[i])[1],
                          params[threadNum].p[2] - point(pts, leaf.p[i])[2] };
        double myd2 = Len2(p2p) - sqr(Dot(p2p, params[threadNum].dir));
        if (myd2 < params[threadNum].closest_d2) {
		    //  cout << point(pts, leaf.p[i])[0] << " " << point(pts, leaf.p[i])[1] << " " << point(pts, leaf.p[i])[2] << " " << myd2 << endl;
          params[threadNum].range_neighbors.push_back(pointparam(pts, leaf.p[i]));
	      }
	    }
	    return;
    }
    
    // Quick check of whether to abort
    double p2c[] = { params[threadNum].p[0] - node.center[0],
                     params[threadNum].p[1] - node.center[1],
                     params[threadNum].p[2] - node.center[2] };
    double myd2center = Len2(p2c) - sqr(Dot(p2c, params[threadNum].dir));
    //if (myd2center > (node.r2 + params[threadNum].closest_d2 + 2.0f * max(node.r2, params[threadNum].closest_d2)))
    if (myd2center > sqr(node.r + params[threadNum].closest_d))
      return;

    // Recursive case
    if (params[threadNum].p[node.splitaxis] < node.center[node.splitaxis] ) {
      node.child1->_fixedRangeSearchAlongDir(pts, threadNum);
      node.child2->_fixedRangeSearchAlongDir(pts, threadNum);
    } else {
      node.child2->_fixedRangeSearchAlongDir(pts, threadNum);
      node.child1->_fixedRangeSearchAlongDir(pts, threadNum);
    }
  
  }

  /*
   * search for points inside the axis aligned bounding box given by p and p0
   * where p[0] < p0[0] && p[1] < p0[1] && p[2] < p0[2]
   */
  void _AABBSearch(const PointData& pts, int threadNum) const {
    AccessorFunc point;
    ParamFunc pointparam;

    // Leaf nodes
    if (npts) {
	 for (int i = 0; i < npts; i++) {
         double* tp = point(pts, leaf.p[i]);
         if (tp[0] >= params[threadNum].p[0] && tp[0] <= params[threadNum].p0[0]
          && tp[1] >= params[threadNum].p[1] && tp[1] <= params[threadNum].p0[1]
          && tp[2] >= params[threadNum].p[2] && tp[2] <= params[threadNum].p0[2]) {
             params[threadNum].range_neighbors.push_back(pointparam(pts, leaf.p[i]));
	   }
	 }
	 return;
    }

    // Quick check of whether to abort
    if (node.center[0]+node.dx < params[threadNum].p[0]
     || node.center[1]+node.dy < params[threadNum].p[1]
     || node.center[2]+node.dz < params[threadNum].p[2]
     || node.center[0]-node.dx > params[threadNum].p0[0]
     || node.center[1]-node.dy > params[threadNum].p0[1]
     || node.center[2]-node.dz > params[threadNum].p0[2])
        return;

    // Recursive case
    if (node.center[node.splitaxis] > params[threadNum].p[node.splitaxis]) {
        node.child1->_AABBSearch(pts, threadNum);
        if (node.center[node.splitaxis] < params[threadNum].p0[node.splitaxis]) {
            node.child2->_AABBSearch(pts, threadNum);
        }
    } else {
        node.child2->_AABBSearch(pts, threadNum);
    }
  }

  void _FixedRangeSearch(const PointData& pts, int threadNum) const {
    AccessorFunc point;
    ParamFunc pointparam;

    // Leaf nodes
    if (npts) {
	 for (int i = 0; i < npts; i++) {
	   double myd2 = Dist2(params[threadNum].p, point(pts, leaf.p[i]));
	   if (myd2 < params[threadNum].closest_d2) {

		params[threadNum].range_neighbors.push_back(pointparam(pts, leaf.p[i]));
		
	   }
	 }
	 return;
    }

    // Quick check of whether to abort
    double approx_dist_bbox =
	 max(max(fabs(params[threadNum].p[0]-node.center[0])-node.dx,
		    fabs(params[threadNum].p[1]-node.center[1])-node.dy),
		fabs(params[threadNum].p[2]-node.center[2])-node.dz);
    if (approx_dist_bbox >= 0 &&
	   sqr(approx_dist_bbox) >= params[threadNum].closest_d2)
	 return;

    // Recursive case
    double myd = node.center[node.splitaxis] - params[threadNum].p[node.splitaxis];
    if (myd >= 0.0) {
	 node.child1->_FixedRangeSearch(pts, threadNum);
	 if (sqr(myd) < params[threadNum].closest_d2) {
	   node.child2->_FixedRangeSearch(pts, threadNum);
	 }
    } else {
	 node.child2->_FixedRangeSearch(pts, threadNum);
	 if (sqr(myd) < params[threadNum].closest_d2) {
	   node.child1->_FixedRangeSearch(pts, threadNum);
	 }
    }
  }


  void _KNNSearch(const PointData& pts, int threadNum) const {
    AccessorFunc point;
    ParamFunc pointparam;

    // Leaf nodes
    if (npts) {
	 for (int i = 0; i < npts; i++) {
	   double myd2 = Dist2(params[threadNum].p, point(pts, leaf.p[i]));

        for (int j = 0; j < params[threadNum].k; j++)
            if (params[threadNum].distances[j] < 0.0f || params[threadNum].distances[j] > myd2) {
            
                for (int l = params[threadNum].k - 1; l > j; --l) {
                    params[threadNum].closest_neighbors[l] = params[threadNum].closest_neighbors[l-1];
                    params[threadNum].distances[l] = params[threadNum].distances[l-1];
                }
                
                params[threadNum].closest_neighbors[j] = pointparam(pts, leaf.p[i]);
                params[threadNum].distances[j] = myd2;
            }
      }
      return;
    }

    /*int kN = params[threadNum].k-1;
    if (params[threadNum].closest_neighbors[kN] != 0) {
        // Quick check of whether to abort  
        double approx_dist_bbox
		= max(max(fabs(params[threadNum].p[0]-node.center[0])-node.dx,
				fabs(params[threadNum].p[1]-node.center[1])-node.dy),
			 fabs(params[threadNum].p[2]-node.center[2])-node.dz);
        if (approx_dist_bbox >= 0 &&
		  sqr(approx_dist_bbox) >= params[threadNum].distances[kN])
		return;
    }

    // Recursive case
    double myd = node.center[node.splitaxis] - params[threadNum].p[node.splitaxis];
    if (myd >= 0.0) {
	 node.child1->_KNNSearch(pts, threadNum);
	 if (sqr(myd) < params[threadNum].closest_d2) {
	   node.child2->_KNNSearch(pts, threadNum);
	 }
    } else {
	 node.child2->_KNNSearch(pts, threadNum);
	 if (sqr(myd) < params[threadNum].closest_d2) {
	   node.child1->_KNNSearch(pts, threadNum);
	 }
    }*/
    if (params[threadNum].p[node.splitaxis] < node.center[node.splitaxis] ) {
      node.child1->_KNNSearch(pts, threadNum);
      node.child2->_KNNSearch(pts, threadNum);
    } else {
      node.child2->_KNNSearch(pts, threadNum);
      node.child1->_KNNSearch(pts, threadNum);
    }
  }

  void _segmentSearch_all(const PointData& pts, int threadNum) const {
    AccessorFunc point;
    ParamFunc pointparam;
    
    double p2p[3], proj[3];
    double t, *comp;
    // Leaf nodes
    if (npts) {
        for (int i = 0; i < npts; i++) {
            p2p[0] = point(pts, leaf.p[i])[0] - params[threadNum].p[0];
            p2p[1] = point(pts, leaf.p[i])[1] - params[threadNum].p[1];
            p2p[2] = point(pts, leaf.p[i])[2] - params[threadNum].p[2];
            t = Dot(p2p, params[threadNum].segment_dir);
            if (t < 0.0) {
                // point is beyond point1 of the segment
                comp = params[threadNum].p;
            } else if (t > params[threadNum].segment_len2) {
                // point is beyond point2 of the segment
                comp = params[threadNum].p0;
            } else {
                // point is within the segment
                // calculate its projection onto the line
                proj[0] = params[threadNum].p[0] + t*params[threadNum].segment_n[0];
                proj[1] = params[threadNum].p[1] + t*params[threadNum].segment_n[1];
                proj[2] = params[threadNum].p[2] + t*params[threadNum].segment_n[2];
                comp = proj;
            }
            if (Dist2(comp,point(pts, leaf.p[i])) < params[threadNum].maxdist_d2) {
                params[threadNum].range_neighbors.push_back(pointparam(pts, leaf.p[i]));
            }
        }
        return;
    }
    
    // Quick check of whether to abort
    p2p[0] = node.center[0] - params[threadNum].p[0];
    p2p[1] = node.center[1] - params[threadNum].p[1];
    p2p[2] = node.center[2] - params[threadNum].p[2];
    t = Dot(p2p, params[threadNum].segment_dir);
    if (t < 0.0) {
        // point is beyond point1 of the segment
        comp = params[threadNum].p;
    } else if (t > params[threadNum].segment_len2) {
        // point is beyond point2 of the segment
        comp = params[threadNum].p0;
    } else {
        // point is within the segment
        // calculate projection
        proj[0] = params[threadNum].p[0] + t*params[threadNum].segment_n[0];
        proj[1] = params[threadNum].p[1] + t*params[threadNum].segment_n[1];
        proj[2] = params[threadNum].p[2] + t*params[threadNum].segment_n[2];
        comp = proj;
    }
    if (Dist2(comp,node.center) > sqr(node.r+params[threadNum].maxdist_d))
        return;

    // Recursive case
    if (params[threadNum].p[node.splitaxis] < node.center[node.splitaxis] ) {
      node.child1->_segmentSearch_all(pts, threadNum);
      node.child2->_segmentSearch_all(pts, threadNum);
    } else {
      node.child2->_segmentSearch_all(pts, threadNum);
      node.child1->_segmentSearch_all(pts, threadNum);
    }
  
  }

  void _segmentSearch_1NearestPoint(const PointData& pts, int threadNum) const {
    AccessorFunc point;
    ParamFunc pointparam;

    double p2p[3], proj[3];
    double t, newdist2;
    // Leaf nodes
    if (npts) {
        for (int i = 0; i < npts; i++) {
            p2p[0] = point(pts, leaf.p[i])[0] - params[threadNum].p[0];
            p2p[1] = point(pts, leaf.p[i])[1] - params[threadNum].p[1];
            p2p[2] = point(pts, leaf.p[i])[2] - params[threadNum].p[2];
            t = Dot(p2p, params[threadNum].segment_dir);
            if (t < 0.0) {
                // point is beyond point1 of the segment
                if (Dist2(params[threadNum].p,point(pts, leaf.p[i])) >= params[threadNum].maxdist_d2)
                    continue;
            } else if (t > params[threadNum].segment_len2) {
                // point is beyond point2 of the segment
                if (Dist2(params[threadNum].p0,point(pts, leaf.p[i])) >= params[threadNum].maxdist_d2)
                    continue;
            } else {
                // point is within the segment
                // calculate its projection onto the line
                proj[0] = params[threadNum].p[0] + t*params[threadNum].segment_n[0];
                proj[1] = params[threadNum].p[1] + t*params[threadNum].segment_n[1];
                proj[2] = params[threadNum].p[2] + t*params[threadNum].segment_n[2];
                if (Dist2(proj,point(pts, leaf.p[i])) >= params[threadNum].maxdist_d2)
                    continue;
            }
            newdist2 = Dist2(params[threadNum].p,point(pts, leaf.p[i]));
            if (newdist2 < params[threadNum].closest_d2) {
                params[threadNum].closest_d2 = newdist2;
                params[threadNum].closest = pointparam(pts, leaf.p[i]);
            }
        }
        return;
    }

    // Quick check of whether to abort
    p2p[0] = node.center[0] - params[threadNum].p[0];
    p2p[1] = node.center[1] - params[threadNum].p[1];
    p2p[2] = node.center[2] - params[threadNum].p[2];
    t = Dot(p2p, params[threadNum].segment_dir);
    if (t < 0.0) {
        // point is beyond point1 of the segment
        if (Dist2(params[threadNum].p,node.center) > sqr(node.r+params[threadNum].maxdist_d))
            return;
    } else if (t > params[threadNum].segment_len2) {
        // point is beyond point2 of the segment
        if (Dist2(params[threadNum].p0,node.center) > sqr(node.r+params[threadNum].maxdist_d))
            return;
    } else {
        // point is within the segment
        // calculate projection
        proj[0] = params[threadNum].p[0] + t*params[threadNum].segment_n[0];
        proj[1] = params[threadNum].p[1] + t*params[threadNum].segment_n[1];
        proj[2] = params[threadNum].p[2] + t*params[threadNum].segment_n[2];
        if (Dist2(proj,node.center) > sqr(node.r+params[threadNum].maxdist_d))
            return;
    }

    // Recursive case
    if (params[threadNum].p[node.splitaxis] < node.center[node.splitaxis] ) {
      node.child1->_segmentSearch_1NearestPoint(pts, threadNum);
      node.child2->_segmentSearch_1NearestPoint(pts, threadNum);
    } else {
      node.child2->_segmentSearch_1NearestPoint(pts, threadNum);
      node.child1->_segmentSearch_1NearestPoint(pts, threadNum);
    }
  
  }
};


#endif
