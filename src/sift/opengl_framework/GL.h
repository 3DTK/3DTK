/*
 * GL.h
 *
 *  Created on: Feb 21, 2010
 *      Author: darko
 */

#ifndef GL_H_
#define GL_H_

#define M_PI 3.141592653589

#include <list>
#include "Object_gl.h"
#include "CoordGL.h"
#include <GL/glut.h>
#include <iostream>
#include <math.h>
#include <time.h>
#include <FTGL/ftgl.h>

#include <X11/Xlib.h>

using namespace std;

class GL {
private:
	static int w_width;
	static int w_height;
	static double aspect;

	static double fovy;
	static double near;
	static double far;

	static CoordGL pos;

	static float theta;
	static float phi;

	static Display *dsp;
	static Window *wnd;

	static clock_t lastMove;

	static FTGLPixmapFont font;

	GL();


public:

	static std::list <Object_gl*> objects;

	virtual ~GL();

	static void execute();

	static void addObject(Object_gl* obj);
	void getFrustum(double fovY, double aspectRatio, double front, double back, double *frustum);

	static void display();
	static void changeSize(int w, int h);
	static void processKeys(unsigned char key, int x, int y);
	static void processMouse(int x, int y);

};

#endif /* GL_H_ */
