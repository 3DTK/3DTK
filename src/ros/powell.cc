/*
  Here fE is the function value at an “extrapolated” point somewhat further along
  the proposed new direction. Also define ∆f to be the magnitude of the largest
  decrease along one particular direction of the present basic procedure iteration. (∆f
      is a positive number.) Then:
  1. If fE ≥ f0 , then keep the old set of directions for the next basic procedure,
  because the average direction PN − P0 is all played out.
  2. If 2 (f0 − 2fN + fE ) [(f0 − fN ) − ∆f]2 ≥ (f0 − fE )2 ∆f, then keep the old
  set of directions for the next basic procedure, because either (i) the decrease along
  the average direction was not primarily due to any single direction’s decrease, or
  (ii) there is a substantial second derivative along the average direction and we seem
  to be near to the bottom of its minimum.
  The following routine implements Powell’s method in the version just described.
  In the routine, xi is the matrix whose columns are the set of directions ni ; otherwise
  the correspondence of notation should be self-evident.
*/

#include "ros/nrutil.h"
#include "ros/powell.h"
#include "slam6d/globals.icc"

#include <stdio.h>
#include <math.h>


//                                     Global variables communicate with f1dim.
int ncom;
float *pcom,*xicom;
//float (calibration::*nrfunc)(float []);
calibration *nrcal;

void powell(float p[], float **xi, int n, float ftol, int *iter, float *fret,
    calibration *cal)
//    float (calibration::*func)(float []))
 /*
 Minimization of a function func of n variables. Input consists of an initial starting point
 p[1..n]; an initial matrix xi[1..n][1..n], whose columns contain the initial set of di-
 rections (usually the n unit vectors); and ftol, the fractional tolerance in the function value
 such that failure to decrease by more than this amount on one iteration signals doneness. On
 output, p is set to the best point found, xi is the then-current direction set, fret is the returned
 function value at p, and iter is the number of iterations taken. The routine linmin is used.
 */
{
  int i,ibig,j;
  float del,fp,fptt,t,*pt,*ptt,*xit;
  pt=fvector(1,n);
  ptt=fvector(1,n);
  xit=fvector(1,n);
  //*fret=(c->*func)(p);
  *fret = cal->function(p);
  //  Save the initial point.
  for (j=1;j<=n;j++) pt[j]=p[j];
  for (*iter=1;;++(*iter)) {
    fp=(*fret);
    ibig=0;
    //    Will be the biggest function decrease.
    del=0.0;

    //                             In each iteration, loop over all directions in the set.
    for (i=1;i<=n;i++) {
      //                                                   Copy the direction,
      for (j=1;j<=n;j++) xit[j]=xi[j][i];
      fptt=(*fret);
      //                                              minimize along it,
//      linmin(p,xit,n,fret,func);
      linmin(p,xit,n,fret,cal);
      //                                              and record it if it is the largest de-
      if (fabs(fptt-(*fret)) > del) {
        //                                                   crease so far.
        del=fabs(fptt-(*fret));
        ibig=i;
      }
    }
    printf("Checking wether to terminate... %f  %f \n", fp, *fret);
    if (2.0*fabs(fp-(*fret)) <= ftol*(fabs(fp)+fabs(*fret))) {
      //                                              Termination criterion.
      free_vector(xit,1,n);
      free_vector(ptt,1,n);
      free_vector(pt,1,n);
      return;
    }
    if (*iter == ITMAX) nrerror("powell exceeding maximum iterations.");
    //                                              Construct the extrapolated point and
    for (j=1;j<=n;j++) {
      //                                                   the average direction moved. Save
      ptt[j]=2.0*p[j]-pt[j];
      //                                                   the old starting point.
      xit[j]=p[j]-pt[j];
      pt[j]=p[j];
    }
    //                                              Function value at extrapolated point.
//    fptt=(c->*func)(ptt);
    fptt=cal->function(ptt);
    if (fptt < fp) {
      t=2.0*(fp-2.0*(*fret)+fptt)*sqr(fp-(*fret)-del)-del*
        sqr(fp-fptt);
      if (t < 0.0) {
        //                                              Move to the minimum of the new
//        linmin(p,xit,n,fret,func);
        linmin(p,xit,n,fret,cal);
        //                                                   direction, and save the new di-
        for (j=1;j<=n;j++) {
          //                                                   rection.
          xi[j][ibig]=xi[j][n];
          xi[j][n]=xit[j];
        }
      }
    }
    //                                              Back for another iteration.
  }
}


void linmin(float p[], float xi[], int n, float *fret,
    calibration *cal)
//    float (calibration::*func) (float []))
// Given an n-dimensional point p[1..n] and an n-dimensional direction xi[1..n], moves and
// resets p to where the function func(p) takes on a minimum along the direction xi from p,
// and replaces xi by the actual vector displacement that p was moved. Also returns as fret
// the value of func at the returned location p. This is actually all accomplished by calling the
// routines mnbrak and brent.
{
     float f1dim(float x);
     int j;
     float xx,xmin,fx,fb,fa,bx,ax;
//                                     Define the global variables.
     ncom=n;
     pcom=fvector(1,n);
     xicom=fvector(1,n);
//     nrfunc=func;
     nrcal = cal;
     for (j=1;j<=n;j++) {
         pcom[j]=p[j];
         xicom[j]=xi[j];
     }
//                                     Initial guess for brackets.
     ax=0.0;
     xx=1.0;
     mnbrak(&ax,&xx,&bx,&fa,&fx,&fb,f1dim);
     *fret=brent(ax,xx,bx,f1dim,TOL,&xmin);
//                                     Construct the vector results to return.
     for (j=1;j<=n;j++) {
         xi[j] *= xmin;
         p[j] += xi[j];
     }
     free_vector(xicom,1,n);
     free_vector(pcom,1,n);
}
float f1dim(float x)
// Must accompany linmin.
{
     int j;
     float f,*xt;
     xt=fvector(1,ncom);
     for (j=1;j<=ncom;j++) xt[j]=pcom[j]+x*xicom[j];
//     f=(*nrfunc)(xt);
     f=nrcal->function(xt);
     free_vector(xt,1,ncom);
     return f;
}


float brent(float ax, float bx, float cx, float (*f)(float), float tol,
    float *xmin)
/*
   Given a function f, and given a bracketing triplet of abscissas ax, bx, cx (such that bx is
   between ax and cx, and f(bx) is less than both f(ax) and f(cx)), this routine isolates
   the minimum to a fractional precision of about tol using Brent’s method. The abscissa of
   the minimum is returned as xmin, and the minimum function value is returned as brent, the
   returned function value.*/
{
  int iter;
  float a,b,d,etemp,fu,fv,fw,fx,p,q,r,tol1,tol2,u,v,w,x,xm;
  //  This will be the distance moved on
  float e=0.0;
  //  the step before last.
  //    a and b must be in ascending order,
  a=(ax < cx ? ax : cx);
  //  but input abscissas need not be.
  b=(ax > cx ? ax : cx);
  //  Initializations...
  x=w=v=bx;
  fw=fv=fx=(*f)(x);
  //  Main program loop.
  for (iter=1;iter<=ITMAXB;iter++) {
    xm=0.5*(a+b);
    tol2=2.0*(tol1=tol*fabs(x)+ZEPS);
    //      Test for done here.
    if (fabs(x-xm) <= (tol2-0.5*(b-a))) {
      *xmin=x;
      return fx;
    }
    //      Construct a trial parabolic fit.
    if (fabs(e) > tol1) {
      r=(x-w)*(fx-fv);
      q=(x-v)*(fx-fw);
      p=(x-v)*q-(x-w)*r;
      q=2.0*(q-r);
      if (q > 0.0) p = -p;
      q=fabs(q);

      etemp=e;
      e=d;
      if (fabs(p) >= fabs(0.5*q*etemp) || p <= q*(a-x) || p >= q*(b-x))
        d=CGOLD*(e=(x >= xm ? a-x : b-x));
//      The above conditions determine the acceptability of the parabolic fit. Here we
//        take the golden section step into the larger of the two segments.
      else {
//        Take the parabolic step.
          d=p/q;
        u=x+d;
        if (u-a < tol2 || b-u < tol2)
          d=__MY_SIGN(tol1,xm-x);
      }
    } else {
      d=CGOLD*(e=(x >= xm ? a-x : b-x));
    }
    u=(fabs(d) >= tol1 ? x+d : x+__MY_SIGN(tol1,d));
    fu=(*f)(u);
//    This is the one function evaluation per iteration.
//      Now decide what to do with our func-
      if (fu <= fx) {
//        tion evaluation.
          if (u >= x) a=x; else b=x;
//        Housekeeping follows:
          SHFT(v,w,x,u)
          SHFT(fv,fw,fx,fu)
      } else {
        if (u < x) a=u; else b=u;
        if (fu <= fw || w == x) {
          v=w;
          w=u;
          fv=fw;
          fw=fu;
        } else if (fu <= fv || v == x || v == w) {
          v=u;
          fv=fu;
        }
//        Done with housekeeping. Back for
      }
//    another iteration.
  }
  nrerror("Too many iterations in brent");
  return -1;
}





void mnbrak(float *ax, float *bx, float *cx, float *fa, float *fb, float *fc,
    float (*func)(float))
/*  Given a function func, and given distinct initial points ax and bx, this routine searches in
  the downhill direction (defined by the function as evaluated at the initial points) and returns
  new points ax, bx, cx that bracket a minimum of the function. Also returned are the function
  values at the three points, fa, fb, and fc.*/
{
  float ulim,u,r,q,fu,dum;
  *fa=(*func)(*ax);
  *fb=(*func)(*bx);
//  Switch roles of a and b so that we can go
    if (*fb > *fa) {
//      downhill in the direction from a to b.
        SHFT(dum,*ax,*bx,dum)
        SHFT(dum,*fb,*fa,dum)
    }
//  First guess for c.
    *cx=(*bx)+GOLD*(*bx-*ax);
  *fc=(*func)(*cx);
//  Keep returning here until we bracket.
    while (*fb > *fc) {
//      Compute u by parabolic extrapolation from
        r=(*bx-*ax)*(*fb-*fc);
//      a, b, c. TINY is used to prevent any pos-
        q=(*bx-*cx)*(*fb-*fa);
        //      sible division by zero.
        u=(*bx)-((*bx-*cx)*q-(*bx-*ax)*r) / (2.0*__MY_SIGN(FMAX(fabs(q-r),TINY),q-r));
        ulim=(*bx)+GLIMIT*(*cx-*bx);
        //        We won’t go farther than this. Test various possibilities:
        //          Parabolic u is between b and c: try it.
        if ((*bx-u)*(u-*cx) > 0.0) {
          fu=(*func)(u);
          //            Got a minimum between b and c.
          if (fu < *fc) {
            *ax=(*bx);
            *bx=u;
            *fa=(*fb);
            *fb=fu;
            return;
            //                Got a minimum between between a and u.
          } else if (fu > *fb) {
            *cx=u;
            *fc=fu;
            return;
          }
          //            Parabolic fit was no use. Use default mag-
          u=(*cx)+GOLD*(*cx-*bx);
          //            nification.
          fu=(*func)(u);
          //            Parabolic fit is between c and its
        } else if ((*cx-u)*(u-ulim) > 0.0) {
          //            allowed limit.
          fu=(*func)(u);
          if (fu < *fc) {
            SHFT(*bx,*cx,u,*cx+GOLD*(*cx-*bx))
            SHFT(*fb,*fc,fu,(*func)(u))
          }
          //            Limit parabolic u to maximum
        } else if ((u-ulim)*(ulim-*cx) >= 0.0) {
          //            allowed value.
          u=ulim;
          fu=(*func)(u);
          //            Reject parabolic u, use default magnifica-
        } else {
          //            tion.
          u=(*cx)+GOLD*(*cx-*bx);
          fu=(*func)(u);
        }
        //        Eliminate oldest point and continue.
        SHFT(*ax,*bx,*cx,u)
        SHFT(*fa,*fb,*fc,fu)
    }
}
