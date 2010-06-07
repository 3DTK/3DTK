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


GLUquadric *quadSphere;
int swCull = 1; /* toggles software culling, control with the <space> key */


/*
 * matrix and math utility routines and macros
 */

 void matrixConcatenate (float *result, float *ma, float *mb)
{
    int i;
    double mb00, mb01, mb02, mb03,
           mb10, mb11, mb12, mb13,
           mb20, mb21, mb22, mb23,
           mb30, mb31, mb32, mb33;
    double mai0, mai1, mai2, mai3;

    mb00 = mb[0];  mb01 = mb[1];
    mb02 = mb[2];  mb03 = mb[3];
    mb10 = mb[4];  mb11 = mb[5];
    mb12 = mb[6];  mb13 = mb[7];
    mb20 = mb[8];  mb21 = mb[9];
    mb22 = mb[10];  mb23 = mb[11];
    mb30 = mb[12];  mb31 = mb[13];
    mb32 = mb[14];  mb33 = mb[15];

    for (i = 0; i < 4; i++) {
        mai0 = ma[i*4+0];  mai1 = ma[i*4+1];
	    mai2 = ma[i*4+2];  mai3 = ma[i*4+3];

        result[i*4+0] = mai0 * mb00 + mai1 * mb10 + mai2 * mb20 + mai3 * mb30;
        result[i*4+1] = mai0 * mb01 + mai1 * mb11 + mai2 * mb21 + mai3 * mb31;
        result[i*4+2] = mai0 * mb02 + mai1 * mb12 + mai2 * mb22 + mai3 * mb32;
        result[i*4+3] = mai0 * mb03 + mai1 * mb13 + mai2 * mb23 + mai3 * mb33;
    }
}

#define vectorLength(nin) \
    sqrt((nin)[0]*(nin)[0] + (nin)[1]*(nin)[1] + (nin)[2]*(nin)[2])
#define distanceFromPlane(peq,p) \
    ((peq)[0]*(p)[0] + (peq)[1]*(p)[1] + (peq)[2]*(p)[2] + (peq)[3])




/*
 * Wiew volume plane storage and calculation
 */

/* Storage for the six planes, left right top bottom near far */
float planeEqs[6][4];

/* Calculates the six view volume planes in object coordinate (OC) space.
       
       Algorithm:
       
       A view volume plane in OC is transformed into CC by multiplying it by
       the inverse of the combined ModelView and Projection matrix (M).
       Algebraically, this is written:
              -1
         P   M   = P
          oc        cc
       
       The resulting six view volume planes in CC are:
         [ -1  0  0  1 ]
         [  1  0  0  1 ]
         [  0 -1  0  1 ]
         [  0  1  0  1 ]
         [  0  0 -1  1 ]
         [  0  0  1  1 ]
       
       To transform the CC view volume planes into OC, we simply multiply
       the CC plane equations by the combined ModelView and Projection matrices
       using standard vector-matrix multiplication. Algebraically, this is written:  
         P   M = P
          cc      oc
       
       Since all of the CC plane equation components are 0, 1, or -1, full vector-
       matrix multiplication is overkill. For example, the first element of the
       first OC plane equation is computed as:
         A = -1 * m0 + 0 * m1 + 0 * m2 + 1 * m3
       This simplifies to:
         A = m3 - m0
       
       Other terms simpliofy similarly. In fact, all six plane equations can be
       computed as follows:
         [ m3-m0  m7-m4  m11-m8  m15-m12 ]
         [ m3+m0  m7+m4  m11+m8  m15+m12 ]
         [ m3-m1  m7-m5  m11-m9  m15-m13 ]
         [ m3+m1  m7+m5  m11+m9  m15+m13 ]
         [ m3-m2  m7-m6  m11-m10 m15-m14 ]
         [ m3+m2  m7+m6  m11+m10 m15+m14 ]
     */
 void calcViewVolumePlanes ()
{
    GLfloat ocEcMat[16], ecCcMat[16], ocCcMat[16];


    /* Get the modelview and projection matrices */
    glGetFloatv (GL_MODELVIEW_MATRIX, ocEcMat);
    glGetFloatv (GL_PROJECTION_MATRIX, ecCcMat);

    /* ocCcMat transforms from OC (object coordinates) to CC (clip coordinates) */
    matrixConcatenate (ocCcMat, ocEcMat, ecCcMat);

    /* Calculate the six OC plane equations. */
    planeEqs[0][0] = ocCcMat[3] - ocCcMat[0]; 
    planeEqs[0][1] = ocCcMat[7] - ocCcMat[4]; 
    planeEqs[0][2] = ocCcMat[11] - ocCcMat[8]; 
    planeEqs[0][3] = ocCcMat[15] - ocCcMat[12]; 

    planeEqs[1][0] = ocCcMat[3] + ocCcMat[0]; 
    planeEqs[1][1] = ocCcMat[7] + ocCcMat[4]; 
    planeEqs[1][2] = ocCcMat[11] + ocCcMat[8]; 
    planeEqs[1][3] = ocCcMat[15] + ocCcMat[12]; 

    planeEqs[2][0] = ocCcMat[3] + ocCcMat[1]; 
    planeEqs[2][1] = ocCcMat[7] + ocCcMat[5]; 
    planeEqs[2][2] = ocCcMat[11] + ocCcMat[9]; 
    planeEqs[2][3] = ocCcMat[15] + ocCcMat[13]; 

    planeEqs[3][0] = ocCcMat[3] - ocCcMat[1]; 
    planeEqs[3][1] = ocCcMat[7] - ocCcMat[5]; 
    planeEqs[3][2] = ocCcMat[11] - ocCcMat[9]; 
    planeEqs[3][3] = ocCcMat[15] - ocCcMat[13]; 

    planeEqs[4][0] = ocCcMat[3] + ocCcMat[2]; 
    planeEqs[4][1] = ocCcMat[7] + ocCcMat[6]; 
    planeEqs[4][2] = ocCcMat[11] + ocCcMat[10]; 
    planeEqs[4][3] = ocCcMat[15] + ocCcMat[14]; 

    planeEqs[5][0] = ocCcMat[3] - ocCcMat[2]; 
    planeEqs[5][1] = ocCcMat[7] - ocCcMat[6]; 
    planeEqs[5][2] = ocCcMat[11] - ocCcMat[10]; 
    planeEqs[5][3] = ocCcMat[15] - ocCcMat[14]; 
}



/* Test a sphere's bounding box against the six clip planes */
 int culled (float *p) 
{
    int i;
    int culled;

    for (i=0; i<6; i++) {
        culled = 0;
        if (!(distanceFromPlane(planeEqs[i],p) < 0.))
          return 1;
    }
    /* Not trivially culled. visible. */
    return 0;
}
/* 
static int culled (double p[3])
{
    int i, j;
    int culled;

    for (i=0; i<3; i++) {
        culled = 0;
        for (j=0; j<8; j++) {   // point must be inside of all planes
            if (distanceFromPlane(planeEqs[i], p[j]) < 0.)
                culled |= 1<<j;
        }
        if (culled==0xff)
            // All eight vertices of bounding box are trivially culled
            return 1;
    }
    // Not trivially culled. Probably visible.
    return 0;
}
*/


float frustum[6][4];

 void ExtractFrustum()
{
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


bool PointInFrustum( float x, float y, float z )
{
   int p;

   for( p = 0; p < 6; p++ )
      if( frustum[p][0] * x + frustum[p][1] * y + frustum[p][2] * z + frustum[p][3] <= 0 )
         return false;
   return true;
}

bool SphereInFrustum( float x, float y, float z, float radius )
{
  int p;

  for( p = 0; p < 6; p++ )
    if( frustum[p][0] * x + frustum[p][1] * y + frustum[p][2] * z + frustum[p][3] <= -radius )
      return false;
  return true;
}

/**
 * 0 if not in frustrum
 * 1 if partial overlap
 * 2 if entirely within frustrum
 */
int QuadInFrustrum2( float x, float y, float z, float xsize, float ysize, float zsize )
{
   int p;

   for( p = 0; p < 6; p++ )
   {
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] > 0 )
         continue;
      return 0;
   }

   // box is in frustrum, now check wether all corners are within. If one is outside return 1 otherwise 2
   for( p = 0; p < 6; p++ )
   {
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] < 0 )
         return 1;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] < 0 )
         return 1;
   }
   return 2;

}

bool QuadInFrustrum( float x, float y, float z, float xsize, float ysize, float zsize )
{
   int p;

   for( p = 0; p < 6; p++ )
   {
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z - zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y - ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x - xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] > 0 )
         continue;
      if( frustum[p][0] * (x + xsize) + frustum[p][1] * (y + ysize) + frustum[p][2] * (z + zsize) + frustum[p][3] > 0 )
         continue;
      return false;
   }
   return true;
}

bool CubeInFrustum( float x, float y, float z, float size )
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
      return false;
   }
   return true;
}

