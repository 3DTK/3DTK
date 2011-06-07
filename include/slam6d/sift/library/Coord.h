/**
 * @file Coords.h
 * @brief 
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#ifndef COORD_H
#define COORD_H
#include <iostream>

using namespace std;

enum Coord_POLAR 
  {
    POLAR
  };

class Coord 
{
 public :
  float x;
  float y;
  float z;
  
  float depth;
  
  Coord() {x = 0 ; y = 0 ; z = 0;}
  Coord(float *arr);
  Coord(float xx, float yy, float zz);
  Coord(Coord_POLAR polar, float a, float b, float r);
  Coord operator+(Coord c); //add vectors
  Coord operator-(Coord c); //subtract vectors
  Coord operator*(float c); //multiple by scalar
  Coord operator/(float c); //divide by scalar
  Coord operator*(Coord c); //cross product
  float operator%(Coord c); //dot product
  void toArray(double *arr);
  friend ostream &operator<<(ostream &stream, Coord o) 
  {
    stream << o.x << " " << o.y << " " << o.z << endl;
    return stream;
  }
  Coord normalize();
  Coord operator&(Coord c); //multiplication i.e x = x1*x2, y = y1*y2, z = z1 * z2
  float abs();
};
#endif
