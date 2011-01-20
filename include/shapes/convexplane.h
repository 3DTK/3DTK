#ifndef __CONVEX_PLANE_H__
#define __CONVEX_PLANE_H__

#include "slam6d/point.h"

#include "wykobi/wykobi.hpp"
#include "wykobi/wykobi_algorithm.hpp"
//#include "shapes/ppair.h"
#include <vector>
using std::vector;

#include <set>
using std::set;

class ConvexPlane {
protected:
  ConvexPlane() {};
  
public:
  ConvexPlane(double _n[3], double _rho, char _direction, wykobi::polygon<double,2> _convex_hull);
  ConvexPlane(double plane[4], vector<Point> &points );
  ConvexPlane(double plane[4]);
  ConvexPlane(vector<ConvexPlane*> &partialplanes);
  void writePlane(std::string, int counter);
  void project(const double *p, double *p1);

  double n[3];
  double rho;
  char direction;
  int pointsize;
  wykobi::polygon<double,2> convex_hull;
  bool isWall();
  bool isHorizontal();
  void horizontalize();
  void verticalize();
};

#endif
