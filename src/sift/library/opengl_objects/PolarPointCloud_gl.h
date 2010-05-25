/*
 * PolarPointCloud_gl.h
 *
 *  Created on: Feb 20, 2010
 *      Author: darko
 */

//TODO: i must include the definitions in the header file because for some reason multiple definition occurs when the definition is
//in the source file.

#ifndef POLARPOINTCLOUD_GL_H_
#define POLARPOINTCLOUD_GL_H_

#include "PolarPointCloud.h"
#include "opengl_framework/Object_gl.h"
#include <GL/glut.h>
#include "Coord.h"

class PolarPointCloud_gl : public Object_gl {
public:
	PolarPointCloud_gl(PolarPointCloud *cl)
	{
		this->x = 0;
		this->y = 0;
		this->z = 0;
		this->cl = cl;
	};

	PolarPointCloud_gl(PolarPointCloud *cl, float x, float y, float z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
		this->cl = cl;
	};
	~PolarPointCloud_gl() {};

	void show()
	{
		PolarPointCloud *ppc = this->cl;
		const std::vector<PolarPoint> *pps = ppc->getData();

		long pc = ppc->getLength();

//		glColor3f(0.0, 0.0, 1.0);
//		glutWireSphere(0.5,10,10);
//		glBegin(GL_POINTS);
		double a,b,d,r,x,y,z;
		for (long l = 0 ; l < pc ; l++)
		{
			r = (*pps)[l].r;
			glColor3f(r,r,r);

			a = (*pps)[l].a;
			b = (*pps)[l].b;
			r = (*pps)[l].d;
//			Coord c(POLAR , a,b,r);
//			glVertex3f(c.x, c.y, c.z);
//			glVertex3f(a,b,0);
			double step = 0.00025;
			glBegin(GL_QUADS);
			glVertex3f(a,b,0);
			glVertex3f(a,b+step,0);
			glVertex3f(a+step,b+step,0);
			glVertex3f(a+step,b,0);
			glEnd();
		}
//		glEnd();
	};

private:
	float x;
	float y;
	float z;
	PolarPointCloud *cl;
};

#endif /* POLARPOINTCLOUD_GL_H_ */
