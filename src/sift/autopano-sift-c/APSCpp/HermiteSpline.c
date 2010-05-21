
/* HermiteSpline.c  16 Feb 2008 TKS

  Hermite Spline Interpolation Routines 
  adapted from John Burkardt's collection at
  http://people.scs.fsu.edu/~burkardt/cpp_src/spline/spline.html
  NOTE FORTRAN style 1-origin array indexing in some places

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


#include <stdlib.h>
#include <math.h>
#include "HermiteSpline.h"
 
/* some functions below test monotonicity by checking that
   the differece across n >= 2 intervals is >= n * fIEPS
   (the difference across a single interval can be 0)
*/
#define fIEPS 1.e-11	// tolerance for nonzero difference

    /** Tabulate coefficients for Hermit Spline Interpolation **/

double *spline_hermite_set ( int ndata, double tdata[], double ydata[], 
  double ypdata[] )

//****************************************************************************80
//
//  Purpose:
//
//    SPLINE_HERMITE_SET sets up a piecewise cubic Hermite interpolant.
//
//  Discussion:
//
//    Once the array C is computed, then in the interval
//    (TDATA(I), TDATA(I+1)), the interpolating Hermite polynomial
//    is given by
//
//      SVAL(TVAL) =                 C(1,I)
//         + ( TVAL - TDATA(I) ) * ( C(2,I)
//         + ( TVAL - TDATA(I) ) * ( C(3,I)
//         + ( TVAL - TDATA(I) ) *   C(4,I) ) )
//
//    This is algorithm CALCCF from Conte and deBoor.
//
//  Modified:
//
//    11 February 2004
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Samuel Conte, Carl deBoor,
//    Elementary Numerical Analysis,
//    Second Edition,
//    McGraw Hill, 1972,
//    ISBN: 07-012446-4.
//
//  Parameters:
//
//    Input, int NDATA, the number of data points.
//    NDATA must be at least 2.
//
//    Input, double TDATA[NDATA], the abscissas of the data points.
//    The entries of TDATA are assumed to be strictly increasing.
//
//    Input, double Y[NDATA], YP[NDATA], the value of the
//    function and its derivative at TDATA(1:NDATA).
//
//    Output, double SPLINE_HERMITE_SET[4*NDATA], the coefficients of 
//    the Hermite polynomial.  We will refer to this array as "C".
//    C(1,1:NDATA) = Y(1:NDATA) and C(2,1:NDATA) = YP(1:NDATA).
//    C(3,1:NDATA-1) and C(4,1:NDATA-1) are the quadratic and cubic
//    coefficients.
//
{
  double *c;
  double divdif1;
  double divdif3;
  double dt;
  int i;
  int j;

  // c = new double[4*ndata];
  c = (double *)malloc(4*ndata*sizeof(double));

  for ( j = 0; j < ndata; j++ )
  {
    c[0+j*4] = ydata[j];
  }

  for ( j = 0; j < ndata; j++ )
  {
    c[1+j*4] = ypdata[j];
  }

  for ( i = 1; i <= ndata-1; i++ )
  {
    dt = tdata[i] - tdata[i-1];
    divdif1 = ( c[0+i*4] - c[0+(i-1)*4] ) / dt;
    divdif3 = c[1+(i-1)*4] + c[1+i*4] - 2.0 * divdif1;
    c[2+(i-1)*4] = ( divdif1 - c[1+(i-1)*4] - divdif3 ) / dt;
    c[3+(i-1)*4] = divdif3 / ( dt * dt );
  }

  c[2+(ndata-1)*4] = 0.0;
  c[3+(ndata-1)*4] = 0.0;

  return c;
}
//****************************************************************************80


      /** Compute a Hermite Spline Interpolation **/
      
int spline_hermite_val ( int ndata, double tdata[], double c[], double tval, 
  double *sval, double *spval )

//****************************************************************************80
//
//  Purpose:
//
//    SPLINE_HERMITE_VAL evaluates a piecewise cubic Hermite interpolant.
//
//  Discussion:
//
//    SPLINE_HERMITE_SET must be called first, to set up the
//    spline data from the raw function and derivative data.
//
//    In the interval (TDATA(I), TDATA(I+1)), the interpolating
//    Hermite polynomial is given by
//
//      SVAL(TVAL) =                 C(1,I)
//         + ( TVAL - TDATA(I) ) * ( C(2,I)
//         + ( TVAL - TDATA(I) ) * ( C(3,I)
//         + ( TVAL - TDATA(I) ) *   C(4,I) ) )
//
//    and
//
//      SVAL'(TVAL) =                    C(2,I)
//         + ( TVAL - TDATA(I) ) * ( 2 * C(3,I)
//         + ( TVAL - TDATA(I) ) *   3 * C(4,I) )
//
//    This is algorithm PCUBIC from Conte and deBoor.
//
//  Modified:
//
//    24 February 2004
//
//  Author:
//
//    John Burkardt
//
//  Reference:
//
//    Samuel Conte, Carl deBoor,
//    Elementary Numerical Analysis,
//    Second Edition,
//    McGraw Hill, 1972,
//    ISBN: 07-012446-4.
//
//  Parameters:
//
//    Input, int NDATA, the number of data points.
//    NDATA is assumed to be at least 2.
//
//    Input, double TDATA[NDATA], the abscissas of the data points.
//    The entries of TDATA are assumed to be strictly increasing.
//
//    Input, double C[4*NDATA], the coefficient data computed by
//    SPLINE_HERMITE_SET.
//
//    Input, double TVAL, the point where the interpolant is to
//    be evaluated.
//
//    Output, double *SVAL, *SPVAL, the value of the interpolant
//    and its derivative at TVAL.
//
/** TKS mods Feb 2008
    add a return value: 0 failure, 1 success
	use interpolating interval search; fail if arg out of range
	don't compute derivative if spval is null
    declare local vars register
**/
{
  register double dt;
  register int left;

   left = findInterval( ndata, tdata, tval );
   if( left < 0 ) return 0;
//
//  Evaluate the cubic polynomial.
//
  dt = tval - tdata[left];

  *sval =        c[0+(left)*4] 
        + dt * ( c[1+(left)*4] 
        + dt * ( c[2+(left)*4] 
        + dt *   c[3+(left)*4] ) );
        
  if( spval )
  *spval =             c[1+(left)*4] 
        + dt * ( 2.0 * c[2+(left)*4] 
        + dt *   3.0 * c[3+(left)*4] );

  return 1;
}

/*  Compute array of tangents for a tabulated function
    to serve as derivatives for setting up a Hermite spline.

  t values must be strictly monotonic (no zero differences) 
    but need not be equally spaced.
  If y is monotonic, the tangents will be adjusted to ensure that
    interpolated values are monotonic, too; else no adjustment.

  Input: 
    ndata, tdata[ndata], ydata[ndata] as above
           ypd[ndata] -- receives the tangents
  Output: 
    returns 0 if tdata[] is not monotonic or ndata < 2 (fail)
		    1 if y is monotonic (tangents set)
		   -1 if y is not monotonic (tangents set)

  Method: 
    divided differences, suitable for unequally spaced data
    (the spline 1st derivative may not be perfectly smooth)

  References:
    Wikipedia: "Cubic Hermite spline"
    Wikipedia: "Monotone cubic interpolation"

*/
int spline_tangents_set( int ndata, double tdata[], double ydata[], double ypd[] ){
  int i, mono;
  double d, *del = 0;
// check t
  if( ndata < 2 ) return 0;
  d  = (tdata[ndata-1] - tdata[0]) / (ndata - 1);  // avg t difference 
  if( fabs(d) < fIEPS ) return 0;  // implausibly small!
  if( d < 0 ){
    for( i = 1; i < ndata; i++ ) if( tdata[i] - tdata[i-1] >= 0 ) return 0;
  } else {
    for( i = 1; i < ndata; i++ ) if( tdata[i] - tdata[i-1] <= 0 ) return 0;
  }

// allocate temp array or die
  del = (double *)malloc(ndata*sizeof(double));
  if( !del ) return 0;

// compute divided y differences, test monotonicity
  mono = 1;
  d = 0;	// previous difference
  for( i = 1; i < ndata; i++ ){
    register double yd = ydata[i] - ydata[i-1];
    if( yd * d < 0 ) mono = 0;  // not monotone
	d = yd;
    del[i] = yd / (tdata[i] - tdata[i-1]);
  }
  if( mono ){
	  if( fabs( ydata[ndata - 1] - ydata[0] ) < ndata * fIEPS ){
		  mono = 0;
	  }
  }

// compute tangents
  ypd[0] = del[1];
  ypd[ndata-1] = del[ndata-1];
  for( i = 1; i < ndata - 1; i++ ){
    ypd[i] = 0.5 * (del[i] + del[i+1]); 
  }

// adjust for monotonicity
  if( mono ){
    register double a, b, c;
    for( i = 1; i < ndata; i++ ){
      if( fabs(del[i]) < 1.e-11 ){
        ypd[i-1] = ypd[i] = 0;
      } else {
        a = ypd[i-1] / del[i];
        b = ypd[i] / del[i];
        c = a * a + b * b;
        if( c > 9.0 ){  // adjust...
          c = 3.0 / sqrt( c );
          ypd[i-1] *= c;
          ypd[i] *= c;
        }
      }
    }
  }

  free( del );
  return ( mono ? 1 : -1 );
}

/*  Find the interval (if any) in t[n] that contains x
    t[] must be monotonic ascending or descending
	  (isolated intervals with zero difference OK)
	returns i if x == t[i]
  returns index (0:n-2) of the left end of the interval, or
          -1 if x is out of range
		  -2 if t is not monotonic 
  method: binary search intialized with secant
*/

int findInterval( int n, double t[], double x )
{   int l, r, s;
    double d;

	if ( n < 2 ) return -2;	// no data

	l = 0; r = n-1;
	d = t[r] - t[l];
	if( fabs( d ) < fIEPS ) return -2;	// insufficient slope
	s = (int) ((x - t[0]) / d);			// splitting idex
	if( s < l || s > r ) return -1;		// out of range

	if( d > 0 ){
		while ( r - l > 1 ){
			if( x < t[s] ) r = s;
			else l = s;
			s = ( r + l ) / 2;
		}
	} else {
		while ( r - l > 1 ){
			if( x > t[s] ) r = s;
			else l = s;
			s = ( r + l ) / 2;
		}
	}

	return l;
}

