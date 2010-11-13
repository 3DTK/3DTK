/*
 * Point.h
 *
 *  Created on: Mar 14, 2010
 *      Author: darko
 */

#ifndef POINTC_H_
#define POINTC_H_

#include "Coord.h"


class PointC: public Coord {
public:
	PointC() : Coord() {r = 0;}
	PointC(float x, float y, float z, float r) : Coord(x,y,z) {this->r = r;}
	PointC(Coord_POLAR polar, float a, float b, float d, float r) : Coord(polar, a,b,d) {this->r = r;}
	virtual ~PointC();

public:
	/**
	 * reflectance
	 */
	float r;
};


#endif /* POINT_H_ */
