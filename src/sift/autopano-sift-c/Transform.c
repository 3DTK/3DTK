
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* Transform.cs
 *
 * Simple 2d transformation functions.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 */

#include "AutoPanoSift.h"

AffineTransform2D*  AffineTransform2D_new0 ()
{
    AffineTransform2D* self = (AffineTransform2D*)malloc(sizeof(AffineTransform2D));
    return self;
}

void AffineTransform2D_delete(AffineTransform2D* self)
{
    if (self) {
	SimpleMatrix_delete((SimpleMatrix*)self);
    }
}

AffineTransform2D*  AffineTransform2D_clone (AffineTransform2D* self)
{
    AffineTransform2D* cl = AffineTransform2D_new0 ();
    cl->base.values =  DoubleMap_new(self->base.yDim, self->base.xDim);

    int x,y;
    for ( y = 0 ; y < self->base.yDim ; ++y)
	for ( x = 0 ; x < self->base.xDim ; ++x)
	    cl->base.values[y][x] = self->base.values[y][x];
    
    return (cl);
}

AffineTransform2D* AffineTransform2D_new ()
{
    AffineTransform2D* self  = AffineTransform2D_new0 ();
    SimpleMatrix_init(&self->base, 3, 3);
    return self;
}

// The shift length in pixels.
int AffineTransform2D_GetShiftWidth(AffineTransform2D* self)
{
    SimpleMatrix* X = SimpleMatrix_new (3, 1);
    X->values[0][0] = 0.0;
    X->values[1][0] = 0.0;
    X->values[2][0] = 1.0;
    
    SimpleMatrix* xI1 = SimpleMatrix_Mul((SimpleMatrix*)self, X);
    /*WriteLine ("xI1[0,0] = {0}, xI1[1,0] = {1}",
      xI1[0, 0], xI1[1, 0]);*/

    SimpleMatrix_delete(X);

    double length = sqrt (pow (xI1->values[0][ 0], 2.0) +
			  pow (xI1->values[1][ 0], 2.0));
    //WriteLine ("length = %f", length);
    SimpleMatrix_delete(xI1);

    return ((int) length);
}


	// Calculate a new transformation from two matching points in two views.
	// That is, one view (lets call it Image I1) has two points
	// A1 = (a1x, a1y), B1 = (b1x, b1y). Those points should anchor a
	// translated, rotated and scaled coordinate system so that they transform
	// affinely to A2 = (a2x, a2y), B2 = (b2x, b2y) in I2.
	//
	// The transformation matrix build is homogeneous 3x3:
	//   s * cos(T)   s * (-sin(T))  s * (cos(T)*(-a2x) - sin(T)*(-a2y)) + a1x
	//   s * sin(T)   s * cos(T)     s * (sin(T)*(-a2x) + cos(T)*(-a2y)) + a1y
	//   0            0              1
	// where
	//   s is the scale factor with
	//      s = |(a1x ; a1y)| / |(a2x ; a2y)|
	//   T is the rotation angle with
	//      T = atan2 (b1y - a1y ; b1x - a1x) - atan2 (b2y - a2y ; b2x - a2x)
	//
	// The matrix is combined from T_2 * S * R * T_1, where T_1 transforms the
	// points in I2 to the origin, R rotates it into the direction of the line
	// A1-B1, S scales it so the length of A1-B1 and A2-B2 match and T_2
	// transforms it relative to A1.
	//
	// The final transform is a mapping from points of I2 into I1.
AffineTransform2D* AffineTransform2D_BuildTransformFromTwoPairs (
		double a1x, double a1y, double b1x, double b1y,
		double a2x, double a2y, double b2x, double b2y,
		int centerX, int centerY)
{
    /*WriteLine ("Build from: A1(%f;%f)-B1(%f;%f) to A2(%f;%f)-B2(%f;%f)",
      a1x, a1y, b1x, b1y, a2x, a2y, b2x, b2y);*/

    // Calculate the rotation angle for later use.
    double angle = atan2 (b1y - a1y, b1x - a1x) - atan2 (b2y - a2y, b2x - a2x);
    double sinAngle = sin (angle);
    double cosAngle = cos (angle);

    // Calculate the scale for later use.
    double s = sqrt ((b2x - a2x)*(b2x - a2x) + (b2y - a2y)*(b2y - a2y));
    if (s == 0.0)
        // A2 is identical to B2, zero length. Cannot continue.
        return NULL;

    s = sqrt ((b1x - a1x)*(b1x - a1x) + (b1y - a1y)*(b1y - a1y)) / s;

    AffineTransform2D* trans = AffineTransform2D_new ();
    trans->rotationAngle = angle;

    trans->base.values[0][ 0] = s * cosAngle;
    trans->base.values[0][ 1] = s * -sinAngle;
    trans->base.values[0][ 2] = s * (cosAngle * (-a2x) - sinAngle * (-a2y)) + a1x;
    trans->base.values[1][ 0] = s * sinAngle;
    trans->base.values[1][ 1] = s * cosAngle;
    trans->base.values[1][ 2] = s * (sinAngle * (-a2x) + cosAngle * (-a2y)) + a1y;
    trans->base.values[2][ 0] = 0.0;
    trans->base.values[2][ 1] = 0.0;
    trans->base.values[2][ 2] = 1.0;

    // Calculate the center-center angle from center-east to center-east
    // of both images
    SimpleMatrix* X = SimpleMatrix_new (3, 1);
    X->values[0][ 0] = centerX;
    X->values[1][ 0] = centerY;
    X->values[2][ 0] = 1.0;

    SimpleMatrix* xI1 = SimpleMatrix_Mul((SimpleMatrix*)trans, X);
    SimpleMatrix_delete(X);

    trans->centerAngle = atan2 (xI1->values[1][ 0] - centerY,
				xI1->values[0][ 0] - centerX);
    SimpleMatrix_delete(xI1);

    /*
      WriteLine ("  C ({0},{1}) -> ({2},{3}) -> ({4},{5}) => {6}",
      centerX, centerY, xI1[0, 0], xI1[1, 0],
      xI1[0, 0] - centerX, xI1[1, 0] - centerY, trans.CenterAngle);
    */
    
    return (trans);
}

#ifdef TEST_MAIN
int main (int argc, char* argv[])
{
    AffineTransform2D* M = AffineTransform2D_BuildTransformFromTwoPairs
	(1.0, 2.0, 4.0, 2.0, 3.0, 2.0, 6.0, 4.0, 320, 240);
    
    WriteLine ("M = %s", SimpleMatrix_ToString ((SimpleMatrix*)M));
    
    SimpleMatrix* X = SimpleMatrix_new (3, 1);
    X->values[0][ 0] = 4;
    X->values[1][ 0] = 5;
    X->values[2][ 0] = 1;

    SimpleMatrix* m = SimpleMatrix_Mul((SimpleMatrix*)M, X);
    WriteLine ("M * X = %s", SimpleMatrix_ToString(m));

    AffineTransform2D_delete(M);
    SimpleMatrix_delete(X);
    SimpleMatrix_delete(m);
    return 0;
}
#endif
