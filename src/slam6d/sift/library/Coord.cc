/**
 * @file Coords.cc
 * @brief Implementation of Class Coord
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Grmany.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @author Darko Makreshanski. Jacobs University Bremen gGmbH, Germany.
 */
#include "slam6d/sift/library/Coord.h"
#include <iostream>
#include <math.h>

using namespace std;

Coord::Coord(float *arr) 
{
  x = arr[0];
  y = arr[1];
  z = arr[2];
}

Coord::Coord(float xx, float yy, float zz) 
{
  x = xx;
  y = yy;
  z = zz;
}

Coord Coord::operator+(Coord c) 
{
  Coord temp(c.x + x, c.y + y, c.z + z);
  return temp;
}

Coord Coord::operator-(Coord c) 
{
  Coord temp(x - c.x, y - c.y, z - c.z);
  return temp;
}

Coord Coord::operator*(float c) 
{
  Coord temp(x * c, y * c, z * c);
  return temp;
}

Coord Coord::operator/(float c) 
{
  Coord temp(x / c, y / c, z / c);
  return temp;
}

Coord Coord::operator*(Coord c) 
{
  Coord temp(
	     y * c.z - z * c.y,
	     z * c.x - x * c.z,
	     x * c.y - y * c.x
	     );
  return temp;
}

float Coord::operator%(Coord c) 
{
  return x*c.x + y*c.y + z*c.z;
}

Coord Coord::normalize() 
{
  float n = sqrt(x * x + y * y + z * z);
  Coord res(x,y,z);
  res = res / n;
  return res;
}

Coord Coord::operator&(Coord c) 
{
  Coord temp(
	     x * c.x,
	     y * c.y,
	     z * c.z
	     );
  return temp;
}

float Coord::abs() 
{
  return sqrt(x*x + y*y + z*z);
}

Coord::Coord(Coord_POLAR polar, float a, float b, float r) 
{
  x = r * cos(a) * cos(b);
  y = r * sin(b);
  z = r * sin(a) * cos(b);
}

void Coord::toArray(double *arr)
{
  arr[0] = x;
  arr[1] = y;
  arr[2] = z;
}
