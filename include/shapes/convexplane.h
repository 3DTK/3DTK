#ifndef __CONVEX_PLANE_H__
#define __CONVEX_PLANE_H__

#include "slam6d/point.h"

#include <vector>
#include <list>

#include <set>



class ConvexPlane {
protected:
  ConvexPlane() {};

public:
  static bool furtherleft(double * point, double * start, double * end);
  static void JarvisMarchConvexHull(std::list<double*> &points, std::vector<double*> &convex_hull);
  ConvexPlane(double _n[3], double _rho, char _direction, std::vector<double*> _convex_hull);
  ConvexPlane(double plane[4], std::vector<Point> &points );
  ConvexPlane(double plane[4]);
  ConvexPlane(std::vector<ConvexPlane*> &partialplanes);
  ~ConvexPlane();
  void writePlane(std::string, int counter);
  void writeNormal(std::string, int counter);
  std::vector<double> getConvexHull();
  void getNormal(double* normal, double* origin);
  void project(const double *p, double *p1);

  double n[3];
  double rho;
  char direction;
  int pointsize;
  std::vector<double*> convex_hull;
  bool isWall();
  bool isHorizontal();
  void horizontalize();
  void verticalize();
};

#endif
