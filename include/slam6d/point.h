/**
 *  @file
 *  @brief Representation of a 3D point
 *  @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Inst. of CS, University of Osnabrueck, Germany.
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
  inline Point();
  
  /**
   *	Copy constructor
   */
  inline Point(const Point& p);
  
  /**
   *	Constructor with an array, i.e., vecctor of coordinates
   */
  inline Point(const double *p);

  /**
   *	Constructor with an array, i.e., vecctor of coordinates and color
   */
  inline Point(const double *p, const char *c);

  /**
   *	Constructor with three double values
   */
  inline Point(const double _x, const double _y, const double _z);

  /**
   *	Constructor with three double values and three color values
   */
  inline Point(const double _x, const double _y, const double _z,
			const char _r, const char _g, const char _b);

  /**
   *	Constructor with three double values and three color values
   */
  inline Point(const double _x,  const double _y,  const double _z,
			const double _nx, const double _ny, const double _nz);
 
  inline Point operator+(const Point &p) const;
  inline Point operator-(const Point &p) const;
  inline Point& operator-=(const Point &p);
  inline Point& operator+=(const Point &p);

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
  /// normal x direction in 3D space
  double nx;
  /// normal x direction in 3D space
  double ny;
  /// normal x direction in 3D space
  double nz;
  /// additional information about the point, e.g., semantic
  ///  also used in veloscan for distiuguish moving or static
  int type;

  /////////////////////////for veloslam/////////////////////////////
  double rad;
  ///    tang in  cylindrical coordinates for veloscan
  double tan_theta;
  // point id in points for veloscan , you can use it find point.
  long point_id;
  /////////////////////////for veloslam/////////////////////////////

  // color information of the point between 0 and 255
  // rgb
  unsigned char rgb[3];

  float reflectance;
  float temperature;
  float amplitude;
  float deviation;

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
