#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <GL/glut.h>
#include <GL/glu.h>

#ifndef _MSC_VER
#include <stdbool.h>
#endif

#ifndef __VIEWCULL_H__
#define __VIEWCULL_H__

bool HitBoundingBox(double center[3], double size );
void calcRay(int x, int y, double znear, double zfar);  
float RayDist(double *point); 
short ScreenDist(double *point);
void ExtractFrustum(short detail);


bool CubeInFrustum( float x, float y, float z, float size );
int  CubeInFrustum2( float x, float y, float z, float size );

void remViewport();
bool LOD(float x, float y, float z, float size);

#endif
