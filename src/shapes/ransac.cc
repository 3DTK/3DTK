#include "shapes/ransac.h"
#include "shapes/ransac_Boctree.h"

static const unsigned int MIN_NR_PTS = 10;

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
      
      if (r > MIN_NR_PTS) {
        vector<T *> points;
        oct->PointsOnShape(shape, points);
      }
    }
  }

  delete oct;


}
