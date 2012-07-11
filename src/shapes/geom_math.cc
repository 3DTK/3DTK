/*
 * geom_math implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#include "math.h"
#include "shapes/geom_math.h"

float Nx,Ny,Nz,Nd;
float Cx, Cy, Cz;
float maxDist = 1.0;     // cm
float maxRadius = 105.0; // cm
float CxM, CxP, CyM, CyP, CzM, CzP;

bool SphereInAABB( float x, float y, float z, float size ) {
  if ( x + size < CxM ||
       x - size > CxP ||
       y + size < CyM ||
       y - size > CyP ||
       z + size < CzM ||
       z - size > CzP ) {
    return false;
  }
  return true;
}


void setNumber(double *plane, double *center, double _radius, double _maxDist) {
  maxDist = _maxDist;
  maxRadius = _radius;

  Nx = plane[0];
  Ny = plane[1];
  Nz = plane[2];
  Nd = plane[3];

  Cx = center[0];
  Cy = center[1];
  Cz = center[2];
  CxM = Cx - maxRadius;
  CxP = Cx + maxRadius;
  CyM = Cy - maxRadius;
  CyP = Cy + maxRadius;
  CzM = Cz - maxRadius;
  CzP = Cz + maxRadius;
}




bool PlaneInCube( float x, float y, float z, float size, float nx, float ny, float nz, float d)
{
   float xm, xp, ym, yp, zm, zp;
   float Fxm, Fxp, Fym, Fyp, Fzm, Fzp;
   xm = x - size;
   xp = x + size;
   ym = y - size;
   yp = y + size;
   zm = z - size;
   zp = z + size;

   Fxm = nx * xm;
   Fym = ny * ym;
   Fzm = nz * zm;

   bool positive = (Fxm + Fym + Fzm + d > 0);


   Fxp = nx * xp;
   if( (Fxp + Fym + Fzm + d < 0) == positive)
     return true;

   Fyp = ny * yp;
   if( (Fxm + Fyp + Fzm + d < 0) == positive )
     return true;

   if( (Fxp + Fyp + Fzm + d < 0) == positive )
     return true;

   Fzp = nz * zp;
   if( (Fxm + Fym + Fzp + d < 0) == positive )
     return true;

   if( (Fxp + Fym + Fzp + d < 0) == positive )
     return true;
   if( (Fxm + Fyp + Fzp + d < 0) == positive )
     return true;
   if( (Fxp + Fyp + Fzp + d < 0) == positive )
     return true;

   return false;
} 


bool PlaneInCube( float x, float y, float z, float size)
{
   float xm, xp, ym, yp, zm, zp;
   float Fxm, Fxp, Fym, Fyp, Fzm, Fzp;
   xm = x - size;
   xp = x + size;
   ym = y - size;
   yp = y + size;
   zm = z - size;
   zp = z + size;

   Fxm = Nx * xm;
   Fym = Ny * ym;
   Fzm = Nz * zm;

   bool positive = (Fxm + Fym + Fzm + Nd > 0);


   Fxp = Nx * xp;
   if( (Fxp + Fym + Fzm + Nd < 0) == positive)
     return true;

   Fyp = Ny * yp;
   if( (Fxm + Fyp + Fzm + Nd < 0) == positive )
     return true;

   if( (Fxp + Fyp + Fzm + Nd < 0) == positive )
     return true;

   Fzp = Nz * zp;
   if( (Fxm + Fym + Fzp + Nd < 0) == positive )
     return true;

   if( (Fxp + Fym + Fzp + Nd < 0) == positive )
     return true;
   if( (Fxm + Fyp + Fzp + Nd < 0) == positive )
     return true;
   if( (Fxp + Fyp + Fzp + Nd < 0) == positive )
     return true;

   return false;
} 

bool closeToPlane(double *p) {
  return ( fabs(planeDist(p, Nx, Ny, Nz, Nd)) < maxDist );
}

