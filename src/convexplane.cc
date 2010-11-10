#include "convexplane.h"
#include "globals.icc"
//#include "hough.h"
//#include "octtree.h"
using std::string;
using std::ofstream;


ConvexPlane::ConvexPlane(double _n[3], double _rho, char _direction, wykobi::polygon<double,2> _convex_hull) {

  for(int i = 0; i < 3; i++) {
    n[i] = _n[i];
  }
  Normalize3(n);

  convex_hull = _convex_hull;
  direction = _direction;
  rho = _rho;
}

ConvexPlane::ConvexPlane(double plane[4], vector<Point> &points ) {

  for(int i = 0; i < 3; i++) {
    n[i] = plane[i];
  }
  rho = plane[3];
  Normalize3(n);

  if(fabs(n[0]) < fabs(n[1])) {
    if(fabs(n[1]) < fabs(n[2])) {
      direction = 'z';
    } else {
      direction = 'y';
    } 
  } else if (fabs(n[2]) < fabs(n[0])){
    direction = 'x';
  } else {
    direction = 'z';
  }

  vector< wykobi::point2d<double> > point_list;

  for (vector<Point>::iterator it = points.begin(); it != points.end(); it++) {
    Point p = (*it);
    wykobi::point2d<double> point; 
    switch(direction) {
      case 'x': point = wykobi::make_point(p.y, p.z); 
                break; 
      case 'y': point = wykobi::make_point(p.x, p.z); 
                break;
      case 'z': point = wykobi::make_point(p.x, p.y); 
                break;
      default: cout << "OHOH" << endl;
    }
    point_list.push_back(point);
  }

  if (point_list.size() > 0) {
    wykobi::algorithm::convex_hull_jarvis_march< wykobi::point2d<double> >(point_list.begin(),point_list.end(),std::back_inserter(convex_hull));
  }

}

void ConvexPlane::writePlane(string path, int counter) {

  ofstream out;
  out.open(path.c_str());

  for(vector< wykobi::point2d<double> >::iterator it = convex_hull.begin();
      it != convex_hull.end();
      it++) {


    switch(direction) {
      case 'x': 
        out << (rho - (*it).x * n[1] - (*it).y * n[2]) / n[0] << " ";
        out << (*it).x << " ";
        out << (*it).y << endl;
        break;
      case 'y':
        out << (*it).x << " ";
        out << (rho - (*it).x * n[0] - (*it).y * n[2]) / n[1] << " ";
        out << (*it).y << endl;
        break;
      case 'z': 
        out << (*it).x << " ";
        out << (*it).y << " ";
        out << (rho - (*it).x * n[0] - (*it).y * n[1]) / n[2] << endl;
        break;
      default: cout << "OHOH" << endl;
    }

  }
  out.close();

}

bool ConvexPlane::isWall() {
  return fabs(n[1]) < 0.1;
}

bool ConvexPlane::isHorizontal() {
  double skalar = n[1] / sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]) ;
  return (1-(fabs(skalar))) < 0.1;
}

void ConvexPlane::horizontalize() {

  n[0] = 0.0;
  n[1] = n[1] < 0 ? -1.0 : 1.0;
  n[2] = 0.0;

}

void ConvexPlane::verticalize() {
  n[1] = 0.0;
  Normalize3(n);
}

