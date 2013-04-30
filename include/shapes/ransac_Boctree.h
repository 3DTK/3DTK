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
  
  template <class P>
  RansacOctTree(P* const* pts, int n, T _voxelSize, PointType _pointtype = PointType() ) : BOctTree<T>(pts, n, _voxelSize, _pointtype) {}

  //  RansacOctTree(vector<const T *> &pts, T voxelSize, PointType _pointtype = PointType() ) : BOctTree<T>(pts, voxelSize, _pointtype) {}

  RansacOctTree(std::string filename) : BOctTree<T> (filename) {}

  void DrawPoints(vector<T *> &p, unsigned char nrp) {
    DrawPoints(p, *BOctTree<T>::root, nrp);
  }
 

  unsigned long PointsOnShape(CollisionShape<T> &shape) {
    return PointsOnShape(*BOctTree<T>::root, BOctTree<T>::center, BOctTree<T>::size, shape);
  }

  void PointsOnShape(CollisionShape<T> &shape, vector<T *> &points) {
    PointsOnShape(*BOctTree<T>::root, BOctTree<T>::center, BOctTree<T>::size, shape, points);
//    exit(0);
  }

protected:
void showbits(char a)
{
  int i  , k , mask;

  for( i =7 ; i >= 0 ; i--)
  {
     mask = 1 << i;
     k = a & mask;
     if( k == 0)
        cout<<"0 ";
     else
        cout<<"1 ";
  }
}

  long PointsOnShape(bitoct &node, T *center, T size, CollisionShape<T> &shape ) {
    if (! shape.isInCube(center[0], center[1], center[2], size)) {
      return 0;
    }

    T ccenter[3];
    bitunion<T> *children;
    bitoct::getChildren(node, children);
    
  /*  
    printf("parent %p   children: %p \n", &node, children);
    cout << "  ";
    showbits(node.valid);
    cout << endl;
    cout << "  ";
    showbits(node.leaf);
    cout << endl;
*/
    long result = 0;

//    int r = 0;

    for (unsigned char i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
//    printf("i: %u r: %d  parent %p   child[r]: %p \n", i, r, &node, children);
//    r++;

	BOctTree<T>::childcenter(center, ccenter, size, i);  // childrens center
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
          // check if leaf contains shape
          if ( shape.isInCube(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {
            pointrep *points = children->getPointreps();
            unsigned int length = points[0].length;
            T *point = &(points[1].v);  // first point
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
        //r++;
      }
    }

    return result;
  }
  
  void PointsOnShape(bitoct &node, T *center, T size, CollisionShape<T> &shape, vector<T*> &vpoints) {
    if (! shape.isInCube(center[0], center[1], center[2], size)) {
      return;
    }

    T ccenter[3];
    bitunion<T> *children;
    bitoct::getChildren(node, children);

    for (unsigned char i = 0; i < 8; i++) {
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
	BOctTree<T>::childcenter(center, ccenter, size, i);  // childrens center
        if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
          // check if leaf contains shape
          if ( shape.isInCube(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {
            pointrep *points = children->getPointreps();
            unsigned int length = points[0].length;
            T *point = &(points[1].v);  // first point
            for(unsigned int iterator = 0; iterator < length; iterator++ ) {
//              cerr << point[0] << " " << point[1] << " " << point[2] << endl;
              if ( shape.containsPoint(point) ) {
                T * p = new T[BOctTree<T>::POINTDIM];
                for (unsigned int iterator = 0; iterator < BOctTree<T>::POINTDIM; iterator++) {
                  p[iterator] = point[iterator];
                }
                vpoints.push_back(p);
              }
              point+= BOctTree<T>::POINTDIM;
            }
          }
        } else { // recurse
          PointsOnShape( children->node, ccenter, size/2.0, shape, vpoints);
        }
        ++children; // next child
      }
    }
  }


  void DrawPoints(vector<T *> &p, bitoct &node, unsigned char nrp) {
    bitunion<T> *children;
    bitoct::getChildren(node, children);
    unsigned char n_children = POPCOUNT(node.valid);
    unsigned char r = randUC(n_children);
    if (r == n_children) r--;

/*    cout << (unsigned int)r << " nc " << (unsigned int)n_children << endl;
    showbits(node.valid);
    cout << endl;
    showbits(node.leaf);
    cout << endl;
*/
    bool leaf = false;
    unsigned char child_index = 0;
/*    if (r == 2) {
    for (unsigned char i = 0; i < 8; i++) {
      cout << "i " << (unsigned int)i << endl;
      if (  ( 1 << i ) & node.valid ) {   // if ith node exists
        cout << "valid " << endl;
        if (child_index == r) {
          cout << "ci == r" << endl;
          if (  ( 1 << i ) & node.leaf ) {   // if ith node exists
            cout << "leaf" << endl;
            leaf = true; 
          }
          cout << "no leaf" << endl;
          break;
        }
        child_index++;
      }
    }
    } else {*/
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
//    }
//    cout << (unsigned int)r << " nc " << (unsigned int)n_children <<  " " << (unsigned int)child_index << endl;
    if (child_index != r) return;   // bitmask valid might be all zero

    if (leaf) {
/*      cout << "STOPPED" << endl;
        return;*/
     
      pointrep *points = children[r].getPointreps();
      unsigned int length = points[0].length;
      if (length < nrp) return;
      if (length == nrp) {
        for (char c = 0; c < nrp; c++) {
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
/*      printf("r: %d   parent %p  children %p   child[r]: %p \n", r, &node, children, &(children[r].node));
    showbits(node.valid);
    cout << endl;
    showbits(node.leaf);
    cout << endl;
      cout << "RECURSED" << endl;*/
      DrawPoints(p, children[r].node, nrp);
    }
  }

};

#endif
