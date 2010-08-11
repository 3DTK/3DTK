/**
 *  @file
 *  @brief Representation of a 3D point
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __POINT_H__
#define __POINT_H__

#include <cmath>
#include <iostream>
using std::ostream;
using std::istream;

#include <stdexcept>
using std::runtime_error;

/**
 * @brief Representation of a point in 3D space
 */
class Point {

public:
  /**
   *	Default constructor
   */
  inline Point() { x = y = z = 0.0; type = 0; reflectance = 0.0; amplitude = 0.0; deviation = 0.0; };
  /**
   *	Copy constructor
   */
  inline Point(const Point& p) { x = p.x; y = p.y; z = p.z; type = p.type; reflectance = p.reflectance; amplitude = p.amplitude; deviation = p.deviation; };
  /**
   *	Constructor with an array, i.e., vecctor of coordinates
   */
  inline Point(const double *p) { x = p[0]; y = p[1]; z = p[2]; };

  /**
   *	Constructor with three double values 
   */
  inline Point(const double _x, const double _y, const double _z) { x = _x; y = _y; z = _z; };
  
  static inline Point cross(const Point &X, const Point &Y) {
    Point res;
    res.x = X.y * Y.z - X.z * Y.y;
    res.y = X.z * Y.x - X.x * Y.z;
    res.z = X.x * Y.y - X.y * Y.x;
    return res;
  };
  
  static inline Point norm(const Point &p) {
    double l = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
    Point res(p.x/l, p.y/l, p.z/l);
    return res;
  };

  inline Point operator+(const Point &p) const {
    Point res;
    res.x = x + p.x;
    res.y = y + p.y;
    res.z = z + p.z;
    return res; 
  };
  
  inline Point operator-(const Point &p) const {
    Point res;
    res.x = x - p.x;
    res.y = y - p.y;
    res.z = z - p.z;
    return res; 
  };
  


  inline void transform(const double alignxf[16]);
  inline double distance(const Point& p);
  inline friend ostream& operator<<(ostream& os, const Point& p);
  inline friend istream& operator>>(istream& is, Point& p);

  // also public; set/get functions not necessary here
  /// x coordinate in 3D space
  double x;
  /// y coordinate in 3D space
  double y;
  /// z coordinate in 3D space
  double z;
  /// additional information about the point, e.g., semantic 
  int type;

  float reflectance;
  float amplitude;
  float deviation;

};
  
inline Point operator*(const double &v, const Point &p) {
  Point res;
  res.x = v * p.x;
  res.y = v * p.y;
  res.z = v * p.z;
  return res;
}

#include "point.icc"

#endif
