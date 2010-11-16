#ifndef __RANSAC_H__
#define __RANSAC_H__

#include "shape.h"
#include "slam6d/scan.h"
#include "shapes/ransac_Boctree.h"

// TODO implement some parameters to modify ransac (maybe in CollisionShape?)
template <class T>
void Ransac(CollisionShape<T> &shape, Scan *scan) {
  // stores 3 sample points    
  vector<T *> ps;
  // create octree from the points
  RansacOctTree<T> *oct = new RansacOctTree<T>(scan->get_points_red() , 50.0, PointType<T>::USE_REFLECTANCE );
 
  while(true) {
    ps.clear();
    oct->DrawPoints(ps, shape.getNrPoints());

    if ( shape.hypothesize(ps) ) {
      
      // count number of points on the plane
      int r =  oct->PointsOnShape(shape);
      
      if (r > 10) {
        vector<T *> points;
        oct->PointsOnShape(shape, points);
      }
    }
  }

  delete oct;


}

#endif
