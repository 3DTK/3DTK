#ifndef __RANSAC_H__
#define __RANSAC_H__

#include "shape.h"
#include "slam6d/scan.h"
#include "shapes/ransac_Boctree.h"

// TODO implement some parameters to modify ransac (maybe in CollisionShape?)
template <class T>
void Ransac(CollisionShape<T> &shape, Scan *scan, vector<T*> *best_points = 0) {
  int best_score = 0;
  CollisionShape<T> *best = shape.copy();

  // stores 3 sample points    
  vector<T *> ps;
  // create octree from the points
  RansacOctTree<T> *oct = new RansacOctTree<T>(scan->get_points_red(), scan->get_points_red_size(), 50.0 );

  for(int i = 0; i < 10000; i++) {
    ps.clear();
    // randomly select points from the octree
    oct->DrawPoints(ps, shape.getNrPoints());

    // compute shape parameters from points
    if ( shape.hypothesize(ps) ) {
      
      // count number of points on the shape
      int r =  oct->PointsOnShape(shape);
      
      if (r > best_score) {
        if (best) delete best;
        // remember this best fitted shape
        if (best_points) {
          best_points->clear();
          oct->PointsOnShape(shape, *best_points);
        }
        best_score = r;
        best = shape.copy();
      }
    }
  }

  shape = *best;
  delete best;

  delete oct;


}

#endif
