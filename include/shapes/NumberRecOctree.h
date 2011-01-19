#ifndef NRRANSAC_OCTREE_H
#define NRRANSAC_OCTREE_H

#include "shapes/geom_math.h"
#include "shapes/ransac_Boctree.h"
#include "slam6d/Boctree.h"



template <class T=double>
class NumberRecOctTree : public RansacOctTree<T> {

  public: 
//  NumberRecOctTree(vector<T *> &pts, T voxelSize, PointType<T> _pointtype = PointType<T>() ) : RansacOctTree<T>(pts, voxelSize, _pointtype) {}
  NumberRecOctTree(vector<const T *> &pts, T voxelSize, PointType _pointtype = PointType() ) : RansacOctTree<T>(pts, voxelSize, _pointtype) {}

  long PointsOnNumber(double plane[4], double maxdist, double _center[3], double radius) { 
    setNumber(plane, _center, radius, maxdist);
    return PointsOnNumber(*BOctTree<T>::root, BOctTree<T>::center, BOctTree<T>::size);
  }

  void PointsOnNumber(double plane[4], double maxdist, double _center[3], double radius, vector<T *> &n) { 
    setNumber(plane, _center, radius, maxdist);
    PointsOnNumber(*BOctTree<T>::root, BOctTree<T>::center, BOctTree<T>::size, n);
  }
 

  protected:
  long PointsOnNumber(bitoct &node, T *center, T size) {
    if (!SphereInAABB(center[0], center[1], center[2], size)) {
      return 0;
    }

    T ccenter[3];
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    long result = 0;

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        childcenter(center, ccenter, size, i);  // childrens center
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
          // check if leaf contains plane
          if ( SphereInAABB(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {
            pointrep *points = children->points;
            unsigned int length = points[0].length;
            T *point = &(points[1].v);  // first point
            for(unsigned int iterator = 0; iterator < length; iterator++ ) {
              if( closeToPlane(point) ) 
                result++;
              point += BOctTree<T>::POINTDIM;
            }
          }
        } else { // recurse
          result += PointsOnNumber( children->node, ccenter, size/2.0);
        }
        ++children; // next child
      }
    }

    return result;
  }

  void PointsOnNumber(bitoct &node, T *center, T size, vector<T*> &n) {
    if (!SphereInAABB(center[0], center[1], center[2], size)) {
      return ;
    }

    T ccenter[3];
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        childcenter(center, ccenter, size, i);  // childrens center
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
          // check if leaf contains plane
          if ( SphereInAABB(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {
            pointrep *points = children->points;
            unsigned int length = points[0].length;
            T *point = &(points[1].v);  // first point
            for(unsigned int iterator = 0; iterator < length; iterator++ ) {
              if( closeToPlane(point) ) 
                n.push_back(point);
              point+=BOctTree<T>::POINTDIM;
            }
          }
        } else { // recurse
          PointsOnNumber( children->node, ccenter, size/2.0, n);
        }
        ++children; // next child
      }
    }
  }

};


#endif
