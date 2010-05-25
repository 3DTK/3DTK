/*
 * Object_gl.cpp
 *
 *  Created on: Feb 20, 2010
 *      Author: darko
 */

#include "Object_gl.h"

Object_gl::Object_gl() {
	// TODO Auto-generated constructor stub
	TrX = 0;
	TrY = 0;
	TrZ = 0;
	
	RotM[0] = 1; RotM[3] = 0; RotM[6] = 0;
	RotM[1] = 0; RotM[4] = 1; RotM[7] = 0;
	RotM[2] = 0; RotM[5] = 0; RotM[8] = 1;
	
	this->updateMultMat();
	
	changed = false;
}

Object_gl::~Object_gl() {
	// TODO Auto-generated destructor stub
}

void Object_gl::setRotM(double *nr)
{
	RotM[0] = nr[0]; RotM[3] = nr[3]; RotM[6] = nr[6];
	RotM[1] = nr[1]; RotM[4] = nr[4]; RotM[7] = nr[7];
	RotM[2] = nr[2]; RotM[5] = nr[5]; RotM[8] = nr[8];
}

void Object_gl::updateMultMat()
{
	MultMat[0] = RotM[0]; MultMat[4] = RotM[3]; MultMat[8] = RotM[6];   MultMat[12] = TrX;
	MultMat[1] = RotM[1]; MultMat[5] = RotM[4]; MultMat[9] = RotM[7];   MultMat[13] = TrY;
	MultMat[2] = RotM[2]; MultMat[6] = RotM[5]; MultMat[10] = RotM[8];  MultMat[14] = TrZ;
	MultMat[3] = 0; 	  MultMat[7] = 0; 	    MultMat[11] = 0;       	MultMat[15] = 1;
	changed = true;
}

void Object_gl::setMultMat(double mat[][4])
{
	MultMat[0] = mat[0][0]; MultMat[4] = mat[0][1]; MultMat[8]  = mat[0][2]; MultMat[12] = mat[0][3];
	MultMat[1] = mat[1][0]; MultMat[5] = mat[1][1]; MultMat[9]  = mat[1][2]; MultMat[13] = mat[1][3];
	MultMat[2] = mat[2][0]; MultMat[6] = mat[2][1]; MultMat[10] = mat[2][2]; MultMat[14] = mat[2][3];
	MultMat[3] = mat[3][0]; MultMat[7] = mat[3][1]; MultMat[11] = mat[3][2]; MultMat[15] = mat[3][3];
	changed = true;
}

//void Object_gl::show() {
//
//}
