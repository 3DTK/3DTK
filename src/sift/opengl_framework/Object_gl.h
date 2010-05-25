/*
 * Object_gl.h
 *
 *  Created on: Feb 20, 2010
 *      Author: darko
 */

#ifndef OBJECT_GL_H_
#define OBJECT_GL_H_

#include <GL/glut.h>
#include <iostream>

class Object_gl {
private:
	double MultMat[16];
	bool changed;	
public:
	Object_gl();

	double TrX;
	double TrZ;
	double TrY;

	void updateMultMat();

	double RotM[9];
	void setRotM(double *nr);

	void setMultMat(double mat[][4]);
	
	virtual ~Object_gl();

	void preshow() {
//		std::cout << "SDFDS\n";
		glPushMatrix();
//		glTranslatef(TrX, TrY, TrZ);
		if (changed) {
			glMultMatrixd(MultMat);
		}
	}
	
	void aftershow() {
		glPopMatrix();
	}

	virtual void show() = 0;

};

#endif /* OBJECT_GL_H_ */
