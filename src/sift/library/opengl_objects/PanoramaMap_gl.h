/*
 * PanoramaMap_gl.h
 *
 *  Created on: Mar 15, 2010
 *      Author: darko
 */

#ifndef PANORAMAMAP_GL_H_
#define PANORAMAMAP_GL_H_

#include "../PanoramaMap.h"
#include <opengl-framework/Object_gl.h>
#include <GL/glut.h>
#include <stdio.h>

class PanoramaMap_gl: public Object_gl {
public:
	
	static const float scale = 0.01;

	PanoramaMap_gl(PanoramaMap *map) {
		this->map = map;
	}
	virtual ~PanoramaMap_gl(){}

	void show();

private:
	PanoramaMap *map;
};

#endif /* PANORAMAMAP_GL_H_ */
