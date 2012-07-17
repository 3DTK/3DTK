/*
 * convexplane implementation
 *
 * Copyright (C) Dorit Borrmann, Remus Dumitru, Andreas Nuechter
 *
 * Released under the GPL version 3.
 *
 */

#include "shapes/convexplane.h"
#include "slam6d/globals.icc"
using std::string;
using std::ofstream;

/**
  * Checks the position of a point with respect to the line given by two points.
  * @param start starting point of the line
  * @param end end point of the line
  * @param point point to be checked
  * @return true if the point is left of the line or on the line and further away 
  * from the start point.
  */

bool ConvexPlane::furtherleft(double * start, double * point, double * end) {
  double tmp = (end[0] - start[0])*(point[1] - start[1]) - (point[0] - start[0])*(end[1] - start[1]); 
  if(fabs(tmp) < 0.0000000001) {
    double l1 = (point[0] - start[0])*(point[0] - start[0]) 
              + (point[1] - start[1])*(point[1] - start[1]);
    double l2 = (end[0] - start[0])*(end[0] - start[0]) 
              + (end[1] - start[1])*(end[1] - start[1]);
    return (l1 > l2);
  } else if(tmp < 0) {
    return false;

  } else {
    return true;
  }
  exit(0);
}

/**
  * Calculates the convex hull of a 2d point set using the Jarvis March
  * algorithm.
  * 
  * 1. Find the point that is furthest left in the point set.
  * 2. Continue to select points such that the remaining point cloud always
  * stays on the right sight of that line spanned by the last point and the new
  * point.
  * 3. Stop when all remaining points are to the right of the line from the last
  * point to the starting point.
  */
void ConvexPlane::JarvisMarchConvexHull(list<double*> &points, vector<double*> &convex_hull) {
  //pointOnHull = leftmost point in S
  list<double*>::iterator itr = points.begin();
  list<double*>::iterator end = itr;
  while(itr != points.end()) {
    if((*end)[0] > (*itr)[0]) {
      end = itr;
    }
    itr++;
  }
  double * anchor = (*end);
  convex_hull.push_back(anchor);
  itr = points.begin();
  double * start = convex_hull[0]; 
  double * current = (*points.begin());
  bool closed = true;
  do {
    closed = true;
    itr = points.begin();
    end = points.begin();
    while(itr != points.end()) {
      if(furtherleft(start, (*itr), current)) {
        end = itr;
        current = (*end);
        closed = false;
      }
      itr++;
    }
    start = current;
    if(!closed) {
      convex_hull.push_back(current);
      end = points.erase(end);
    }
    current = anchor;
  } while(start != anchor);
  // convex_hull.pop_back();
  
  itr = points.begin();
  while(itr != points.end()) {
    if((*itr) == anchor) {
      itr=points.erase(itr);
      break;
    } else {
      itr++;
    }
  }
  // cout << "End of Convex " << convex_hull.size() << endl;
}

/**
  * Constructor of a convex plane given the normal vector, the distance, the
  * direction of the plane (largest coordinate of the normal vector) and a
  * vector of points that form the convex hull of the plane.
  */
ConvexPlane::ConvexPlane(double _n[3], double _rho, char _direction,
vector<double*> _convex_hull) {

  for(int i = 0; i < 3; i++) {
    n[i] = _n[i];
  }
  Normalize3(n);

  convex_hull = _convex_hull;
  direction = _direction;
  rho = _rho;
}

/**
  * Constructor of a convex plane given the normal vector and distance of the
  * plane.
  */
ConvexPlane::ConvexPlane(double plane[4]) {
  for(int i = 0; i < 3; i++) {
    n[i] = plane[i];
  }
  rho = plane[3];
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
}

/** 
  * Constructor of a convex plane given several partial planes
  */
ConvexPlane::ConvexPlane(vector<ConvexPlane*> &partialplanes) {
  int size = partialplanes.size();
  for(int i = 0; i < size; i++) {
    for(int j = 0; j < 3; j++) {
      n[j] += partialplanes[i]->n[j];
      rho += partialplanes[i]->rho;
    }
  }
  for(int j = 0; j < 3; j++) {
    n[j] /= size;
  }
  rho /= size;
}

/**
  * Constructor of a convex plane given the normal vector and distance of the
  * plane and a vector of points that lie on the plane.
  */
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

  list<double *> point_list;

  for (vector<Point>::iterator it = points.begin(); it != points.end(); it++) {
    Point p = (*it);
    double * point = new double[2];
    switch(direction) {
      case 'x': point[0] = p.y;
                point[1] = p.z; 
                break; 
      case 'y': point[0] = p.x;
                point[1] = p.z;
                break;
      case 'z': point[0] = p.x;
                point[1] = p.y;
                break;
      default: throw runtime_error("default branch taken");
    }
    point_list.push_back(point);
  }

  if (point_list.size() > 0) {
    JarvisMarchConvexHull(point_list, convex_hull);
  }
}
ConvexPlane::~ConvexPlane() {
  for(vector<double* >::iterator it = convex_hull.begin();
    it != convex_hull.end(); it++) {
    double* tmp = (*it);
    delete[] tmp;
  }
}

/**
  * Writes the plane as normalXXX.3d to the directory given in the path. XXX is
  * the three digit representation of the counter.
  * This function writes the center and the normal of the plane to the file.
  */
void ConvexPlane::writeNormal(string path, int counter) {
  ofstream out;
  out.open(path.c_str());
  double center[3];
  for(int i = 0; i < 3; i++) {
    center[i] = 0.0;
  }
  
  for(vector<double*>::iterator it = convex_hull.begin();
      it != convex_hull.end();
      it++) {

    
    switch(direction) {
      case 'x': 
        center[0] += (rho - (*it)[0] * n[1] - (*it)[1] * n[2]) / n[0];
        center[1] += (*it)[0];
        center[2] += (*it)[1];
        break;
      case 'y':
        center[0] += (*it)[0];
        center[1] += (rho - (*it)[0] * n[0] - (*it)[1] * n[2]) / n[1];
        center[2] += (*it)[1];
        break;
      case 'z': 
        center[0] += (*it)[0];
        center[1] += (*it)[1];
        center[2] += (rho - (*it)[0] * n[0] - (*it)[1] * n[1]) / n[2];
        break;
      default: throw runtime_error("default branch taken");
    }

  }
  
  for(int i = 0; i < 3; i++) {
    center[i] /= convex_hull.size();
  }


  out << n[0] << " " << n[1] << " " << n[2] << endl;
  out << center[0] << " " << center[1] << " " << center[2] << endl;

  out.close();

}

/**
  * Writes the convex hull of the plane as planeXXX.3d to the directory given in the path.
  * XXX is the three digit representation of the counter. 
  */
void ConvexPlane::writePlane(string path, int counter) {

  ofstream out;
  out.open(path.c_str());
  
  for(vector<double*>::iterator it = convex_hull.begin();
      it != convex_hull.end();
      it++) {

    switch(direction) {
      case 'x': 
        out << (rho - (*it)[0] * n[1] - (*it)[1] * n[2]) / n[0] << " ";
        out << (*it)[0] << " ";
        out << (*it)[1] << endl;
        break;
      case 'y':
        out << (*it)[0] << " ";
        out << (rho - (*it)[0] * n[0] - (*it)[1] * n[2]) / n[1] << " ";
        out << (*it)[1] << endl;
        break;
      case 'z': 
        out << (*it)[0] << " ";
        out << (*it)[1] << " ";
        out << (rho - (*it)[0] * n[0] - (*it)[1] * n[1]) / n[2] << endl;
        break;
      default: throw runtime_error("default branch taken");
    }

  }
  out.flush();
  out.close();
}

vector<double> ConvexPlane::getConvexHull() {
    vector<double> hull;

    for(vector<double*>::iterator it = convex_hull.begin();
        it != convex_hull.end();
        it++) {

        double point[3];

      switch(direction) {

        case 'x':
          point[0] = (rho - (*it)[0] * n[1] - (*it)[1] * n[2]) / n[0];
          point[1] = (*it)[0];
          point[2] = (*it)[1];
          break;
        case 'y':
          point[0] = (*it)[0];
          point[1] = (rho - (*it)[0] * n[0] - (*it)[1] * n[2]) / n[1];
          point[2] = (*it)[1];
          break;
        case 'z':
          point[0] = (*it)[0];
          point[1] = (*it)[1];
          point[2] = (rho - (*it)[0] * n[0] - (*it)[1] * n[1]) / n[2];
          break;
        default: throw runtime_error("default branch taken");
      }

      hull.push_back(point[0]);
      hull.push_back(point[1]);
      hull.push_back(point[2]);
    }

    return hull;
}

void ConvexPlane::getNormal(double* normal, double* origin) {
    double center[3];
    for(int i = 0; i < 3; i++) {
      center[i] = 0.0;
    }

    for(vector<double*>::iterator it = convex_hull.begin();
        it != convex_hull.end();
        it++) {

      switch(direction) {
        case 'x':
          center[0] += (rho - (*it)[0] * n[1] - (*it)[1] * n[2]) / n[0];
          center[1] += (*it)[0];
          center[2] += (*it)[1];
          break;
        case 'y':
          center[0] += (*it)[0];
          center[1] += (rho - (*it)[0] * n[0] - (*it)[1] * n[2]) / n[1];
          center[2] += (*it)[1];
          break;
        case 'z':
          center[0] += (*it)[0];
          center[1] += (*it)[1];
          center[2] += (rho - (*it)[0] * n[0] - (*it)[1] * n[1]) / n[2];
          break;
        default: throw runtime_error("default branch taken");
      }
    }

    for(int i = 0; i < 3; i++) {
      center[i] /= convex_hull.size();
    }

    normal[0] = n[0];
    normal[1] = n[1];
    normal[2] = n[2];

    origin[0] = center[0];
    origin[1] = center[1];
    origin[2] = center[2];
}

void ConvexPlane::project(const double *p, double *p1) {
  double dist = n[0] * p[0] + n[1] * p[1] + n[2] * p[2] - rho;
  
  p1[0] = p[0] - n[0] * dist;
  p1[1] = p[1] - n[1] * dist;
  p1[2] = p[2] - n[2] * dist;
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

