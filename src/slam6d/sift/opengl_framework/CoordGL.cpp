/*
 * Coord.cpp
 *
 *  Created on: Mar 14, 2010
 *      Author: darko
 */

#include "CoordGL.h"
#include <math.h>

using namespace std;


CoordGL::CoordGL(double xx, double yy, double zz) {
	x = xx;
	y = yy;
	z = zz;
}

CoordGL CoordGL::operator+(CoordGL c) {
	CoordGL temp(c.x + x, c.y + y, c.z + z);
	return temp;
}

CoordGL CoordGL::operator-(CoordGL c) {
	CoordGL temp(x - c.x, y - c.y, z - c.z);
	return temp;
}

CoordGL CoordGL::operator*(double c) {
	CoordGL temp(x * c, y * c, z * c);
	return temp;
}

CoordGL CoordGL::operator/(double c) {
	CoordGL temp(x / c, y / c, z / c);
	return temp;
}

CoordGL CoordGL::operator*(CoordGL c) {
	CoordGL temp(
				y * c.z - z * c.y,
				z * c.x - x * c.z,
				x * c.y - y * c.x
			);
	return temp;
}

double CoordGL::operator%(CoordGL c) {
	return x*c.x + y*c.y + z*c.z;
}

CoordGL CoordGL::normalize() {
	float n = sqrt(x * x + y * y + z * z);
	CoordGL res(x,y,z);
	res = res / n;
	return res;
}

CoordGL CoordGL::operator&(CoordGL c) {
	CoordGL temp(
				x * c.x,
				y * c.y,
				z * c.z
			);
	return temp;
}

double CoordGL::abs() {
	return sqrt(x*x + y*y + z*z);
}

CoordGL CoordGL::fromPolar(double a, double b, double r) {
	CoordGL t(r * cos(a) * sin(b), r * sin(a) * sin(b), r * cos(b));
	return t;
}




