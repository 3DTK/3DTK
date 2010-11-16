#ifndef __GEOM_MATH_H__
#define __GEOM_MATH_H__

double planeDist(double *p, float nx, float ny, float nz, float d); 
bool SphereInAABB( float x, float y, float z, float size ) ;
void setNumber(double *plane, double *center, double _radius, double _maxDist) ;
bool PlaneInCube( float x, float y, float z, float size, float nx, float ny, float nz, float d);
bool PlaneInCube( float x, float y, float z, float size);
bool closeToPlane(double *p);

#endif
