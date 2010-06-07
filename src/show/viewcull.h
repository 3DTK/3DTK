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

/*
 * matrix and math utility routines and macros
 */

void matrixConcatenate (float *result, float *ma, float *mbu);

#define vectorLength(nin) \
    sqrt((nin)[0]*(nin)[0] + (nin)[1]*(nin)[1] + (nin)[2]*(nin)[2])
#define distanceFromPlane(peq,p) \
    ((peq)[0]*(p)[0] + (peq)[1]*(p)[1] + (peq)[2]*(p)[2] + (peq)[3])


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
void calcViewVolumePlanes ();

/* Test a sphere's bounding box against the six clip planes */
int culled (float *p); 

void ExtractFrustum();


bool PointInFrustum( float x, float y, float z );
bool QuadInFrustrum( float x, float y, float z, float xsize, float ysize, float zsize );
int  QuadInFrustrum2( float x, float y, float z, float xsize, float ysize, float zsize );

bool CubeInFrustum( float x, float y, float z, float size );

bool SphereInFrustum( float x, float y, float z, float radius );

#endif
