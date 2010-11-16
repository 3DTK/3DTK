/**
 * @file 
 * @brief Efficient representation of an octree for ransac
 * @author Jan Elsberg. Automation Group, Jacobs University Bremen gGmbH, Germany. 
 */

#ifndef RANSAC_OCTREE_H
#define RANSAC_OCTREE_H

#include <stdio.h>

#include <vector>
using std::vector;
#include <deque>
using std::deque;
#include <set>
using std::set;
#include <list>
using std::list;
#include <iostream>
#include <fstream>
#include <string>

#include "shapes/geom_math.h"

#include "slam6d/globals.icc"
#include "slam6d/Boctree.h"
#include "shape.h"

/**
 * @brief Octree
 * 
 * A cubic bounding box is calculated
 * from the given 3D points. Then it
 * is recusivly subdivided into smaller
 * subboxes
 */
template <class T>
class RansacOctTree : public BOctTree<T> {

public:
  
  RansacOctTree(const T **pts, int n, T _voxelSize, PointType<T> _pointtype = PointType<T>() ) : BOctTree<T>(pts, n, _voxelSize, _pointtype) {}

  RansacOctTree(vector<T *> &pts, T voxelSize, PointType<T> _pointtype = PointType<T>() ) : BOctTree<T>(pts, voxelSize, _pointtype) {}

  RansacOctTree(std::string filename) : BOctTree<T> (filename) {}

  void DrawPoints(vector<T *> &p, unsigned char nrp) {
    DrawPoints(p, *BOctTree<T>::root, nrp);
  }
 
  /*
  long PointsOnPlane(double plane[4], double maxdist) {
    return PointsOnPlane(*BOctTree<T>::root, BOctTree<T>::center, BOctTree<T>::size, plane[0], plane[1], plane[2], plane[3], maxdist);
  }*/


  unsigned long PointsOnShape(CollisionShape<T> &shape) {
    return PointsOnShape(*BOctTree<T>::root, BOctTree<T>::center, BOctTree<T>::size, shape);
  }


protected:
  long PointsOnPlane(bitoct &node, T *center, T size, CollisionShape<T> &shape ) {
    if (! shape.isInCube(center[0], center[1], center[2], size)) {
      return 0;
    }

    double ccenter[3];
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    long result = 0;

    for (unsigned char i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        childcenter(center, ccenter, size, i);  // childrens center
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
          // check if leaf contains shape
          if ( shape.isInCube(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {
            pointrep *points = children->points;
            unsigned int length = points[0].length;
            double *point = &(points[1].v);  // first point
            for(unsigned int iterator = 0; iterator < length; iterator++ ) {
              if ( shape.containsPoint(point) )
                result++;
              point+= BOctTree<T>::POINTDIM;
            }
          }
        } else { // recurse
          result += PointsOnShape( children->node, ccenter, size/2.0, shape);
        }
        ++children; // next child
      }
    }

    return result;
  }
  
/*
  long PointsOnPlane(bitoct &node, double *center, double size, float nx, float ny, float nz, float d, float maxdist) { 
    if (!PlaneInCube(center[0], center[1], center[2], size, nx, ny, nz, d)) {
      return 0;
    }

    double ccenter[3];
    bitunion *children;
    bitoct::getChildren(node, children);

    long result = 0;

    for (short i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        childcenter(center, ccenter, size, i);  // childrens center
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
          // check if leaf contains plane
          if ( PlaneInCube(ccenter[0], ccenter[1], ccenter[2], size/2.0, nx, ny, nz, d) ) {
            pointrep *points = children->points;
            unsigned int length = points[0].length;
            double *point = &(points[1].v);  // first point
            for(unsigned int iterator = 0; iterator < length; iterator++ ) {
              if( fabs(planeDist(point, nx, ny, nz, d)) < maxdist ) 
                result++;
              point+=POINTDIM;
            }
          }
        } else { // recurse
          result += PointsOnPlane( children->node, ccenter, size/2.0, nx, ny, nz, d, maxdist);
        }
        ++children; // next child
      }
    }

    return result;
  }
  */

  void DrawPoints(vector<T *> &p, bitoct &node, unsigned char nrp) {
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    unsigned short n_children = POPCOUNT(node.valid);
    unsigned short r = randUC(n_children);
    if (r == n_children) r--;

    bool leaf = false;
    unsigned char child_index = 0;
    for (unsigned char i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        if (child_index == r) {
          if (  ( 1 << i ) & node.leaf ) {   // if ith node exists
            leaf = true; 
          }
          break;
        }
        child_index++;
      }
    }

    if (leaf) {
      pointrep *points = children[r].points;
      unsigned int length = points[0].length;
      if (length < nrp) return;
      if (length == nrp) {
        for (char c = 0; c < 3; c++) {
          p.push_back(&(points[BOctTree<T>::POINTDIM*c+1].v));
        }
        return;
      }

      // randomly get nrp points, we will not check if this succeeds in getting nrp distinct points
      for (char c = 0; c < nrp; c++) {
        int tmp = rand(points[0].length);
        p.push_back(&(points[BOctTree<T>::POINTDIM*tmp+1].v));
      }
    } else {
      DrawPoints(p, children[r].node, nrp);
    }
  }

};

#endif
