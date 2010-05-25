/*
 * PanoramaMap_gl.cpp
 *
 *  Created on: Mar 15, 2010
 *      Author: darko
 */

#include "PanoramaMap_gl.h"
#include <iostream>

using namespace std;


//PanoramaMap_gl::PanoramaMap_gl() {
//	// TODO Auto-generated constructor stub
//
//}
//
//PanoramaMap_gl::~PanoramaMap_gl() {
//	// TODO Auto-generated destructor stub
//}

 void PanoramaMap_gl::show() 
{


//	cout << "TESTSHOW\n";

	int w = map->width;
	int h = map->height;
	for (int x = 0 ; x < w ; x++) {
		for (int y = 0 ; y < h ; y++) {
			glBegin(GL_POLYGON);
//			cout << map->data[x][y].value << " ";	
			double val = map->data[x][y].value;
			glColor3f(val, val, val);
			double xscale = x*scale;
			double yscale = y*scale;
			glVertex3f(xscale, yscale,0);
			glVertex3f(xscale, yscale+scale,0);
			glVertex3f(xscale+scale, yscale+scale,0);
			glVertex3f(xscale+scale, yscale,0);
			glEnd();
		}
	}
}
