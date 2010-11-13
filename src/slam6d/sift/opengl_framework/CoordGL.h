/*
 * Coord.h
 *
 *  Created on: Mar 14, 2010
 *      Author: darko
 */

#ifndef COORDGL_H_
#define COORDGL_H_

#include <iostream>

using namespace std;

class CoordGL {
	public :
	double x;
	double y;
	double z;
	CoordGL() {x = 0 ; y = 0 ; z = 0;}
	CoordGL(double xx, double yy, double zz);
	CoordGL operator+(CoordGL c); //add vectors
	CoordGL operator-(CoordGL c); //subtract vectors
	CoordGL operator*(double c); //multiple by scalar
	CoordGL operator/(double c); //divide by scalar
	CoordGL operator*(CoordGL c); //cross product
	double operator%(CoordGL c); //dot product
	friend ostream &operator<<(ostream &stream, CoordGL o) {
		stream << o.x << " " << o.y << " " << o.z << endl;
		return stream;
	}
	CoordGL normalize();
	CoordGL operator&(CoordGL c); //multiplication i.e x = x1*x2, y = y1*y2, z = z1 * z2
	double abs();

	static CoordGL fromPolar(double a, double b, double r);

};


#endif /* CoordGL_H_ */
