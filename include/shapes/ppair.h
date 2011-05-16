/**
 * @file
 * @brief Definition of plane pairs
 *
 *  @author 
 *  @author Jan Elseberg. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Dorit Borrmann. School of Engineering and Science, Jacobs University Bremen gGmbH, Germany.
 */
#ifndef __PPAIR_H__
#define __PPAIR_H__

#include "convexplane.h"
/**
 * @brief Representing point pairs
 */
class PPair {
public:
  enum TYPE {UNDEFINED, EQUAL, PARALLEL, PERPENDICULAR};
  
  inline PPair(ConvexPlane *p1, ConvexPlane *p2);
  inline PPair() : p1(0), p2(0) {};

  ConvexPlane *p1;  ///< The two points forming the pair
  ConvexPlane *p2;  ///< The two points forming the pair
  double angle;
  double center_distance; // or poly distance?
  double plane_distance;
  // intersection linie
  Point istart, iend;

  TYPE type;

};
  
inline PPair::PPair(ConvexPlane *_p1, ConvexPlane *_p2) {
  type = UNDEFINED;
  p1 = _p1;
  p2 = _p2;

  // TODO hier richtigen wert berechnen
  center_distance = 0.0;
  plane_distance = fabs(p1->rho - p2->rho);
  double tmp = p1->n[0] * p2->n[0] + p1->n[1] * p2->n[1] + p1->n[2] * p2->n[2];
  if (tmp > 1.0) {
    angle = 0.0;
  } else if (tmp < -1.0) {
    angle = M_PI;
  } else {
    angle = acos( tmp );
  }
  if (angle > M_PI / 2.0) {
    angle = M_PI - angle;
  }

  double nnx = p2->n[0]/p2->n[0];

  double y = (p2->rho - nnx * p1->rho) / (p2->n[1] - nnx * p1->n[1]);
  double x = (p1->rho - p1->n[1] * y) / p1->n[0];
  double z = 0.0;

  istart = Point(x,y,z);
  
  double nn[3];
  Cross(p1->n, p2->n, nn);

  iend = Point(x + nn[0], y + nn[1], nn[2]);
}

#endif
