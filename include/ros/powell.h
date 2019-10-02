#ifndef __POWELL_H__
#define __POWELL_H__
#include "calibration.h"
//  Maximum allowed iterations.
#define ITMAX 200
//                                     Tolerance passed to brent.
#define TOL 2.0e-4

void powell(float p[], float **xi, int n, float ftol, int *iter, float *fret,
    calibration *cal);
//    float (calibration::*func)(float []));

//void linmin(float p[], float xi[], int n, float *fret, float (calibration::*func)(float []));
void linmin(float p[], float xi[], int n, float *fret, calibration *cal);
float f1dim(float x);


#define ITMAXB 100
#define CGOLD 0.3819660
#define ZEPS 1.0e-10
/*
   Here ITMAXB is the maximum allowed number of iterations; CGOLD is the golden ratio; ZEPS is
   a small number that protects against trying to achieve fractional accuracy for a minimum that
   happens to be exactly zero.*/
#define SHFT(a,b,c,d) (a)=(b);(b)=(c);(c)=(d);
float brent(float ax, float bx, float cx, float (*f)(float), float tol,
    float *xmin);

#define GOLD 1.618034
#define GLIMIT 100.0
#define TINY 1.0e-20
//  Here GOLD is the default ratio by which successive intervals are magnified; GLIMIT is the
//  maximum magnification allowed for a parabolic-fit step.


void mnbrak(float *ax, float *bx, float *cx, float *fa, float *fb, float *fc,
    float (*func)(float));


#endif
