/**
 * @file
 * @brief Frustrum culling routines
 * 
 * @author Jan Elseberg. Automation Group, Jacobs University Bremen gGmbH, Germany. 
 */
#include "viewcull.h"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <GL/glut.h>
#include <GL/glu.h>

#include "../globals.icc"

/** The 6 planes of the viewing frustum */
float frustum[6][4];

/** the modelview * projection matrix to map a model point to an onscreen coordinate */
float matrix[16];
/** some useful variables for faster calculation of the pixel coordinates */
short VP[4];
/** a unit vector pointing to the right of the screen */
float right[3];
/** how much detail is shown, 0 means everything is plotted */
short DETAIL;

double SX, SY, SZ, EX, EY, EZ;
float origin[3], dir[3];   /*ray */
float dist;
int rayX,rayY;
float  rayVP[4];

void calcRay(int x, int y, double znear, double zfar)  {

  GLdouble modelMatrix[16];
  GLdouble projMatrix[16];
  int viewport[4];
  glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
  glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
  glGetIntegerv(GL_VIEWPORT,viewport);

  gluUnProject(x, viewport[3]-y, zfar, modelMatrix, projMatrix, viewport, &SX, &SY, &SZ);
  gluUnProject(x, viewport[3]-y, znear, modelMatrix, projMatrix, viewport, &EX, &EY, &EZ);

  origin[0] = SX;
  origin[1] = SY;
  origin[2] = SZ;
  dir[0] = EX-SX;
  dir[1] = EY-SY;
  dir[2] = EZ-SZ;
  double t = sqrt( dir[0] * dir[0] + dir[1] * dir[1] + dir[2] * dir[2] );
  dir[0] /= t;
  dir[1] /= t;
  dir[2] /= t;

  dist = SX * dir[0] + SY * dir[1] + SZ * dir[2];
  rayVP[0] = 0.5*viewport[2];
  rayVP[1] = 0.5*viewport[2] + viewport[0];

  rayVP[2] = 0.5*viewport[3];
  rayVP[3] = 0.5*viewport[3] + viewport[1];
  rayX = x;
  rayY = viewport[3]-y;
}

void remViewport() {
  GLdouble modelMatrix[16];
  GLdouble projMatrix[16];
  int viewport[4];
  glGetDoublev(GL_MODELVIEW_MATRIX,modelMatrix);
  glGetDoublev(GL_PROJECTION_MATRIX,projMatrix);
  glGetIntegerv(GL_VIEWPORT,viewport);

  MMult( projMatrix, modelMatrix, matrix );
  VP[0] = 0.5*viewport[2];
  VP[1] = 0.5*viewport[2] + viewport[0];

  VP[2] = 0.5*viewport[3];
  VP[3] = 0.5*viewport[3] + viewport[1];

  right[0] = modelMatrix[0];
  right[1] = modelMatrix[4];
  right[2] = modelMatrix[8];
}

void ExtractFrustum(short detail)
{
   DETAIL = detail + 1;
   remViewport();

   float   proj[16];
   float   modl[16];
   float   clip[16];
   float   t;

   /* Get the current PROJECTION matrix from OpenGL */
   glGetFloatv( GL_PROJECTION_MATRIX, proj );

   /* Get the current MODELVIEW matrix from OpenGL */
   glGetFloatv( GL_MODELVIEW_MATRIX, modl );

   /* Combine the two matrices (multiply projection by modelview) */
   clip[ 0] = modl[ 0] * proj[ 0] + modl[ 1] * proj[ 4] + modl[ 2] * proj[ 8] + modl[ 3] * proj[12];
   clip[ 1] = modl[ 0] * proj[ 1] + modl[ 1] * proj[ 5] + modl[ 2] * proj[ 9] + modl[ 3] * proj[13];
   clip[ 2] = modl[ 0] * proj[ 2] + modl[ 1] * proj[ 6] + modl[ 2] * proj[10] + modl[ 3] * proj[14];
   clip[ 3] = modl[ 0] * proj[ 3] + modl[ 1] * proj[ 7] + modl[ 2] * proj[11] + modl[ 3] * proj[15];

   clip[ 4] = modl[ 4] * proj[ 0] + modl[ 5] * proj[ 4] + modl[ 6] * proj[ 8] + modl[ 7] * proj[12];
   clip[ 5] = modl[ 4] * proj[ 1] + modl[ 5] * proj[ 5] + modl[ 6] * proj[ 9] + modl[ 7] * proj[13];
   clip[ 6] = modl[ 4] * proj[ 2] + modl[ 5] * proj[ 6] + modl[ 6] * proj[10] + modl[ 7] * proj[14];
   clip[ 7] = modl[ 4] * proj[ 3] + modl[ 5] * proj[ 7] + modl[ 6] * proj[11] + modl[ 7] * proj[15];

   clip[ 8] = modl[ 8] * proj[ 0] + modl[ 9] * proj[ 4] + modl[10] * proj[ 8] + modl[11] * proj[12];
   clip[ 9] = modl[ 8] * proj[ 1] + modl[ 9] * proj[ 5] + modl[10] * proj[ 9] + modl[11] * proj[13];
   clip[10] = modl[ 8] * proj[ 2] + modl[ 9] * proj[ 6] + modl[10] * proj[10] + modl[11] * proj[14];
   clip[11] = modl[ 8] * proj[ 3] + modl[ 9] * proj[ 7] + modl[10] * proj[11] + modl[11] * proj[15];

   clip[12] = modl[12] * proj[ 0] + modl[13] * proj[ 4] + modl[14] * proj[ 8] + modl[15] * proj[12];
   clip[13] = modl[12] * proj[ 1] + modl[13] * proj[ 5] + modl[14] * proj[ 9] + modl[15] * proj[13];
   clip[14] = modl[12] * proj[ 2] + modl[13] * proj[ 6] + modl[14] * proj[10] + modl[15] * proj[14];
   clip[15] = modl[12] * proj[ 3] + modl[13] * proj[ 7] + modl[14] * proj[11] + modl[15] * proj[15];

   /* Extract the numbers for the RIGHT plane */
   frustum[0][0] = clip[ 3] - clip[ 0];
   frustum[0][1] = clip[ 7] - clip[ 4];
   frustum[0][2] = clip[11] - clip[ 8];
   frustum[0][3] = clip[15] - clip[12];

   /* Normalize the result */
   t = sqrt( frustum[0][0] * frustum[0][0] + frustum[0][1] * frustum[0][1] + frustum[0][2] * frustum[0][2] );
   frustum[0][0] /= t;
   frustum[0][1] /= t;
   frustum[0][2] /= t;
   frustum[0][3] /= t;

   /* Extract the numbers for the LEFT plane */
   frustum[1][0] = clip[ 3] + clip[ 0];
   frustum[1][1] = clip[ 7] + clip[ 4];
   frustum[1][2] = clip[11] + clip[ 8];
   frustum[1][3] = clip[15] + clip[12];

   /* Normalize the result */
   t = sqrt( frustum[1][0] * frustum[1][0] + frustum[1][1] * frustum[1][1] + frustum[1][2] * frustum[1][2] );
   frustum[1][0] /= t;
   frustum[1][1] /= t;
   frustum[1][2] /= t;
   frustum[1][3] /= t;

   /* Extract the BOTTOM plane */
   frustum[2][0] = clip[ 3] + clip[ 1];
   frustum[2][1] = clip[ 7] + clip[ 5];
   frustum[2][2] = clip[11] + clip[ 9];
   frustum[2][3] = clip[15] + clip[13];

   /* Normalize the result */
   t = sqrt( frustum[2][0] * frustum[2][0] + frustum[2][1] * frustum[2][1] + frustum[2][2] * frustum[2][2] );
   frustum[2][0] /= t;
   frustum[2][1] /= t;
   frustum[2][2] /= t;
   frustum[2][3] /= t;

   /* Extract the TOP plane */
   frustum[3][0] = clip[ 3] - clip[ 1];
   frustum[3][1] = clip[ 7] - clip[ 5];
   frustum[3][2] = clip[11] - clip[ 9];
   frustum[3][3] = clip[15] - clip[13];

   /* Normalize the result */
   t = sqrt( frustum[3][0] * frustum[3][0] + frustum[3][1] * frustum[3][1] + frustum[3][2] * frustum[3][2] );
   frustum[3][0] /= t;
   frustum[3][1] /= t;
   frustum[3][2] /= t;
   frustum[3][3] /= t;

   /* Extract the FAR plane */
   frustum[4][0] = clip[ 3] - clip[ 2];
   frustum[4][1] = clip[ 7] - clip[ 6];
   frustum[4][2] = clip[11] - clip[10];
   frustum[4][3] = clip[15] - clip[14];

   /* Normalize the result */
   t = sqrt( frustum[4][0] * frustum[4][0] + frustum[4][1] * frustum[4][1] + frustum[4][2] * frustum[4][2] );
   frustum[4][0] /= t;
   frustum[4][1] /= t;
   frustum[4][2] /= t;
   frustum[4][3] /= t;

   /* Extract the NEAR plane */
   frustum[5][0] = clip[ 3] + clip[ 2];
   frustum[5][1] = clip[ 7] + clip[ 6];
   frustum[5][2] = clip[11] + clip[10];
   frustum[5][3] = clip[15] + clip[14];

   /* Normalize the result */
   t = sqrt( frustum[5][0] * frustum[5][0] + frustum[5][1] * frustum[5][1] + frustum[5][2] * frustum[5][2] );
   frustum[5][0] /= t;
   frustum[5][1] /= t;
   frustum[5][2] /= t;
   frustum[5][3] /= t;

}



void myProject(float x, float y, float z, short &Xi ) {
  float pn[2];
  // x coordinate on screen, not normalized
  pn[0] = x * matrix[0] + y * matrix[4] + z * matrix[8]  + matrix[12];
  // normalization
  pn[1] = x * matrix[3] + y * matrix[7] + z * matrix[11] + matrix[15];

  // normalized x coordinate on screen
  pn[0] /= pn[1];

  // true x coordinate in viewport coordinate system
  Xi = pn[0]*VP[0] + VP[1];
}


bool LOD(float x, float y, float z, float size)
{
  short X1;
  short X2;
 
  // onscreen position of the leftmost point
  myProject(x - size*right[0], y - size*right[1], z - size*right[2], X1);
  // onscreen position of the rightmost point
  myProject(x + size*right[0], y + size*right[1], z + size*right[2], X2);

  if (X1 > X2) {
    return (X1-X2) > DETAIL;
  } else {
    return (X2-X1) > DETAIL;
  }
}


/**
 * 0 if not in frustrum
 * 1 if partial overlap
 * 2 if entirely within frustrum
 */
int CubeInFrustum2( float x, float y, float z, float size )
{
   int p;

   float xm, xp, ym, yp, zm, zp;
   float Fxm, Fxp, Fym, Fyp, Fzm, Fzp;
   xm = x - size;
   xp = x + size;
   ym = y - size;
   yp = y + size;
   zm = z - size;
   zp = z + size;

   for( p = 0; p < 6; p++ )
   {
      Fxm = frustum[p][0] * xm;
      Fym = frustum[p][1] * ym;
      Fzm = frustum[p][2] * zm;
      if( Fxm + Fym + Fzm + frustum[p][3] > 0 )
         continue;

      Fxp = frustum[p][0] * xp;
      if( Fxp + Fym + Fzm + frustum[p][3] > 0 )
         continue;

      Fyp = frustum[p][1] * yp;
      if( Fxm + Fyp + Fzm + frustum[p][3] > 0 )
         continue;

      if( Fxp + Fyp + Fzm + frustum[p][3] > 0 )
         continue;
      
      Fzp = frustum[p][2] * zp;
      if( Fxm + Fym + Fzp + frustum[p][3] > 0 )
         continue;

      if( Fxp + Fym + Fzp + frustum[p][3] > 0 )
         continue;
      if( Fxm + Fyp + Fzp + frustum[p][3] > 0 )
         continue;
      if( Fxp + Fyp + Fzp + frustum[p][3] > 0 )
         continue;
      return 0;
   }

   // box is in frustrum, now check wether all corners are within. If one is outside return 1 otherwise 2
   for( p = 0; p < 6; p++ )
   {
      Fxm = frustum[p][0] * xm;
      Fym = frustum[p][1] * ym;
      Fzm = frustum[p][2] * zm;
      if( Fxm + Fym + Fzm + frustum[p][3] < 0 )
         return 1;
      
      Fxp = frustum[p][0] * xp;
      if( Fxp + Fym + Fzm + frustum[p][3] < 0 )
         return 1;
      
      Fyp = frustum[p][1] * yp;
      if( Fxm + Fyp + Fzm + frustum[p][3] < 0 )
         return 1;

      if( Fxp + Fyp + Fzm + frustum[p][3] < 0 )
         return 1;

      Fzp = frustum[p][2] * zp;
      if( Fxm + Fym + Fzp + frustum[p][3] < 0 )
         return 1;
      
      if( Fxp + Fym + Fzp + frustum[p][3] < 0 )
         return 1;
      if( Fxm + Fyp + Fzp + frustum[p][3] < 0 )
         return 1;
      if( Fxp + Fyp + Fzp + frustum[p][3] < 0 )
         return 1;
   }
   return 2;

}

int QuadInFrustrum2old( float x, float y, float z, float size )
{
   int p;

   for( p = 0; p < 6; p++ )
   {
      if( frustum[p][0] * (x - size) + frustum[p][1] * (y - size) + frustum[p][2] * (z - size) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + size) + frustum[p][1] * (y - size) + frustum[p][2] * (z - size) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x - size) + frustum[p][1] * (y + size) + frustum[p][2] * (z - size) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + size) + frustum[p][1] * (y + size) + frustum[p][2] * (z - size) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x - size) + frustum[p][1] * (y - size) + frustum[p][2] * (z + size) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + size) + frustum[p][1] * (y - size) + frustum[p][2] * (z + size) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x - size) + frustum[p][1] * (y + size) + frustum[p][2] * (z + size) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + size) + frustum[p][1] * (y + size) + frustum[p][2] * (z + size) + frustum[p][3] > 0 )
         continue;
      return 0;
   }

   // box is in frustrum, now check wether all corners are within. If one is outside return 1 otherwise 2
   for( p = 0; p < 6; p++ )
   {
      if( frustum[p][0] * (x - size) + frustum[p][1] * (y - size) + frustum[p][2] * (z - size) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x + size) + frustum[p][1] * (y - size) + frustum[p][2] * (z - size) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x - size) + frustum[p][1] * (y + size) + frustum[p][2] * (z - size) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x + size) + frustum[p][1] * (y + size) + frustum[p][2] * (z - size) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x - size) + frustum[p][1] * (y - size) + frustum[p][2] * (z + size) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x + size) + frustum[p][1] * (y - size) + frustum[p][2] * (z + size) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x - size) + frustum[p][1] * (y + size) + frustum[p][2] * (z + size) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x + size) + frustum[p][1] * (y + size) + frustum[p][2] * (z + size) + frustum[p][3] < 0 )
         return 1;
   }
   return 2;

}

bool CubeInFrustum( float x, float y, float z, float size )
{
   int p;

   float xm, xp, ym, yp, zm, zp;
   float Fxm, Fxp, Fym, Fyp, Fzm, Fzp;
   xm = x - size;
   xp = x + size;
   ym = y - size;
   yp = y + size;
   zm = z - size;
   zp = z + size;

   for( p = 0; p < 6; p++ )
   {
      Fxm = frustum[p][0] * xm;
      Fym = frustum[p][1] * ym;
      Fzm = frustum[p][2] * zm;
      if( Fxm + Fym + Fzm + frustum[p][3] > 0 )
         continue;

      Fxp = frustum[p][0] * xp;
      if( Fxp + Fym + Fzm + frustum[p][3] > 0 )
         continue;

      Fyp = frustum[p][1] * yp;
      if( Fxm + Fyp + Fzm + frustum[p][3] > 0 )
         continue;

      if( Fxp + Fyp + Fzm + frustum[p][3] > 0 )
         continue;
      
      Fzp = frustum[p][2] * zp;
      if( Fxm + Fym + Fzp + frustum[p][3] > 0 )
         continue;

      if( Fxp + Fym + Fzp + frustum[p][3] > 0 )
         continue;
      if( Fxm + Fyp + Fzp + frustum[p][3] > 0 )
         continue;
      if( Fxp + Fyp + Fzp + frustum[p][3] > 0 )
         continue;
      return false;
   }
   return true;
}



#define NUMDIM  3
#define RIGHT 0
#define LEFT  1
#define MIDDLE  2

float minB[NUMDIM], maxB[NUMDIM];    /*box */
float coord[NUMDIM];       /* hit point */

bool HitBoundingBox(double center[NUMDIM], double size )
{
  minB[0] = center[0] - size;
  minB[1] = center[1] - size;
  minB[2] = center[2] - size;

  maxB[0] = center[0] + size;
  maxB[1] = center[1] + size;
  maxB[2] = center[2] + size;

  bool inside = true;
  char quadrant[NUMDIM];
  register int i;
  int whichPlane;
  float maxT[NUMDIM];
  float candidatePlane[NUMDIM];

  /* Find candidate planes; this loop can be avoided if
    rays cast all from the eye(assume perpsective view) */
  for (i=0; i<NUMDIM; i++)
    if(origin[i] < minB[i]) {
      quadrant[i] = LEFT;
      candidatePlane[i] = minB[i];
      inside = false;
    }else if (origin[i] > maxB[i]) {
      quadrant[i] = RIGHT;
      candidatePlane[i] = maxB[i];
      inside = false;
    }else {
      quadrant[i] = MIDDLE;
    }

  /* Ray origin inside bounding box */
  if(inside)  {
    return (true);
  }


  /* Calculate T distances to candidate planes */
  for (i = 0; i < NUMDIM; i++)
    if (quadrant[i] != MIDDLE && dir[i] !=0.)
      maxT[i] = (candidatePlane[i]-origin[i]) / dir[i];
    else
      maxT[i] = -1.;

  /* Get largest of the maxT's for final choice of intersection */
  whichPlane = 0;
  for (i = 1; i < NUMDIM; i++)
    if (maxT[whichPlane] < maxT[i])
      whichPlane = i;

  /* Check final candidate actually inside box */
  if (maxT[whichPlane] < 0.) return (false);
  for (i = 0; i < NUMDIM; i++)
    if (whichPlane != i) {
      coord[i] = origin[i] + maxT[whichPlane] *dir[i];
      if (coord[i] < minB[i] || coord[i] > maxB[i])
        return (false);
    } else {
      coord[i] = candidatePlane[i];
    }
  return (true);        /* ray hits box */
} 

float RayDist(double *point) 
{
  return point[0] * dir[0] + point[1] * dir[1] + point[2] * dir[2] - dist;
}

short ScreenDist(double *point) {
  float pn[3];
  // x coordinate on screen, not normalized
  pn[0] = point[0] * matrix[0] + point[1] * matrix[4] + point[2] * matrix[8]  + matrix[12];
  pn[1] = point[0] * matrix[1] + point[1] * matrix[5] + point[2] * matrix[9]  + matrix[13];
  // normalization
  pn[2] = point[0] * matrix[3] + point[1] * matrix[7] + point[2] * matrix[11] + matrix[15];

  // normalized x coordinate on screen
  pn[0] /= pn[2];
  pn[1] /= pn[2];

  // true x coordinate in viewport coordinate system
  //Xi = pn[0]*VP[0] + VP[1];
  //fTempo[4]*0.5+0.5)*viewport[2]+viewport[0];
  
  float XX = ( (pn[0])*rayVP[0] + rayVP[1]);
  float YY = ( (pn[1])*rayVP[2] + rayVP[3]);

  short dx, dy;
  if (XX > rayX) dx = XX-rayX;
  else dx = rayX-XX;
  
  if (YY > rayY) dy = YY-rayY;
  else dy = rayY-YY;

  return sqrt(dx*dx + dy*dy);

}
