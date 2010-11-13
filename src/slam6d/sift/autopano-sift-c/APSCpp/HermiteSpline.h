/* HermiteSpline.h  16 Feb 2008 TKS

  Hermite Spline Interpolation Routines 
  lifted & adapted from John Burkardt's collection at
  http://people.scs.fsu.edu/~burkardt/cpp_src/spline/spline.html
  NOTE FORTRAN style 1-origin array indexing

*/
/*
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public
 *  License as published by the Free Software Foundation; either
 *  version 2 of the License, or (at your option) any later version.
 *
 *  This software is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public
 *  License along with this software; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef _HERMITESPLINE_H
#define _HERMITESPLINE_H


#ifdef __cplusplus
extern "C" {
#endif

/* tabulate coefficients for Hermite spline interpolation of y(t)
   tdata[ndata], ydata[ndata] are tabulated y{t} values
   ypdata[ndata] are its 1st derivatives at same t's (how you set
     these determines the curviness of the spline)
   Note t values must be monotonic but intervals can vary
   Returns pointer to a new array c[4*ndata] of coefficients
   Note c[] is allocated with malloc()
*/
double *spline_hermite_set ( int ndata, double tdata[], double ydata[], double ypdata[] );

/* compute hermite spline interpolation
   tdata[ndata] = tabulated t values; c[4*ndata] = coefficients
   tval = interpolation point
   sval -> returned interpolated y value
   spval -> returned derivative at that point (0: no derivative computed)
  returns 0: bad input or 1: sucess
*/
int spline_hermite_val ( int ndata, double tdata[], double c[],
                          double tval, double * sval, double * spval );

/* set tangents (1st divided differences) for a tabulated function
     whose tabulation interval need not be uniform.
     these can serve as the derivatives for setting up a spline.
     
   Input: ndata, tdata[ndata], ydata[ndata] as above.
          ypdata[ndata] -- place for the result
  
   returns 0 if tdata[] is not monotonic or ndata < 2 or the 
             range of t or y is < 1e-11, else
		   1 if ydata[] is monotonic, else
	      -1 if ydata[] is not monotonic

   If y is monotonic, the tangents are adjusted to ensure that all
     interpolated values will be monotonic as well.

*/
int spline_tangents_set( int ndata, double tdata[], double ydata[], double ypdata[] );

/*  Find the interval (if any) in t[n] that contains x
    t[] must be monotonic ascending or descending
	returns i if x == t[i]
  returns index (0:n-2) of the left end of the interval, or
          -1 if x is out of range
		  -2 if t is not monotonic
  method: linear interpolation search
*/
int findInterval( int n, double t[], double x );


#ifdef __cplusplus
}
#endif

#endif  //ndef _HERMITESPLINE_H


