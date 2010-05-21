/* saInterp.c  17 Feb 2008 TKS

  interpolation engine for SphereAlign,  
  specialized from Dersch's panotools code.

  The setup routine is independent of pixel data type,
  but you have to call a typed interpolator.
  
  The interpolation kernel.is chosen at compile time,
  default is 6x6 spline.

  Optional source masks are separate byte arrays.

  Source may be a contiguous array (src -> data[xdim*ydim])
  or a vector of rows (src[y] -> data[xdim]) 
  or a vector of columns (src[x] -> data[ydim])

  Mask array must be contiguous,
  
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


#include <math.h>

/*  point interpolation engines

  computes one pixel of a destination image given the corresponding 
  position in the source image, handling image edges and an optional
  mask.  Can treat the source image as cylindrical in X.
 
  The position is given in source index units relative to the lowest 
  addressed pixel of the source image.
  
  Return 1 if position is inside [masked] source, with destination
  pixel set to interpolated value, else 0 with destination pixel 0.
 
*/

// interpolator is SPLINE36 (can change here)
#define ndim 6
#define	intpol( x, a, NDIM )														\
	a[5] = ( ( -  1.0/11.0  * x +  12.0/ 209.0 ) * x +   7.0/ 209.0  ) * x;				\
	a[4] = ( (    6.0/11.0  * x -  72.0/ 209.0 ) * x -  42.0/ 209.0  ) * x;				\
	a[3] = ( ( - 13.0/11.0  * x + 288.0/ 209.0 ) * x + 168.0/ 209.0  ) * x;				\
	a[2] = ( (   13.0/11.0  * x - 453.0/ 209.0 ) * x -   3.0/ 209.0  ) * x + 1.0;		\
	a[1] = ( ( -  6.0/11.0  * x + 270.0/ 209.0 ) * x - 156.0/ 209.0  ) * x;				\
	a[0] = ( (    1.0/11.0  * x -  45.0/ 209.0 ) * x +  26.0/ 209.0  ) * x;


/**  constant parameters posted by InterpSetup() **/
static char 
    * mask = 0,		// source mask array, 0 if none
    wrap_x = 0;		// source width is 360 degrees
static void 
    * csrc = 0,	   // contiguous source array
    ** vsrc = 0;   // source array of row vectors
static int
    srcwid,			// source dimensions..
    srchgt,
	srcspp,
	srcppr,
	srcvfmt;
static int		// mask...
    smrowlen,		// bytes per row
	smitemlen,		// bytes per pixel
	smidx;			// byte within pixel

/** Set up constant interpolation parameters **/
// retun 0 fail 1 success

int saInterpSetup( void * psrc,	// -> data or pointers
				  int vfmt,		// 0 contig 1 row vectors 2 col vectors
				  int wid,		// valid pixels per row
				  int hgt,		// valid rows
				  int spp,		// samples per pixel (>0)
				  int ppr,		// actual pixels per row (>=wid)
				  char wrap,	// true is X is cyclic
	 		      char * pmask,	// -> mask array, 0 if no mask
				  int mspp,		// mask samples per pixel
			      int mppr,		// actual pixels per row (>= wid)
				  int midx		// index of mask byte in pixel
			     )

{
	csrc = 0; vsrc = 0;
	switch( vfmt ){
	case 0:	// contig
		csrc = psrc;
		break;
	case 1:
	case 2:
		vsrc = (void **)psrc;
		break;
	default:
		return 0;
		break;
	}
	srcvfmt = vfmt;

	srcwid = wid; srchgt = hgt; 
	srcspp = spp; srcppr = ppr;
	wrap_x = wrap;

	mask = pmask; 
	smrowlen = mppr * mspp; 
	smitemlen = mspp;
	smidx = midx;

	return 1;
}

/** Compute one interpolated pixel **/
/* started as Dersch code, now heavily modified to handle 
   vector as well as contiguous source formats.
*/

int saInterp_float( float * pdest,    // -> destination pixel
					double Xpos, double Ypos
				  )
{
// working vars
  static float *row[ndim];
  static float cdata[ndim * ndim];

  int i, k, xs, ys, valid;
  int n2 = ndim / 2;
  double Dx, Dy;

// integer parts of dest posn in src
  int xc    =  (int)floor( Xpos ) ;
  int yc    =  (int)floor( Ypos ) ;

// fail if 0 or 2 sources
  if(!csrc && !vsrc ) return 0;
  if( csrc && vsrc ) return 0;

// test if posn is within source image
  valid =      (xc < srcwid ) 
            && (yc < srchgt ) 
            && (xc >= 0) && (yc >= 0);

// check source mask if used
  if( valid && mask ){
      valid = ( mask [ yc * smrowlen
		               + xc * smitemlen
					   + smidx ]
                != 0 );
  }
// done if position is outside source   
  if( !valid ){
	  *pdest = 0;
	  return 0;
  }

  ys = yc + 1 - n2 ; // smallest y-index used for interpolation
  xs = xc + 1 - n2 ; // smallest x-index used for interpolation

  if( srcvfmt < 2 ){  // contig. or row vector format...
  // fractions
	Dx = Xpos - (double) xc; 
	Dy = Ypos - (double) yc; 

  // k = number of cols outside the row, sign idicates end
    if( xs < 0 ) k = xs;  // - at left
    else {
      k = xs + ndim - srcwid; // + at right
	  if( k < 0 ) k = 0;	// all inside
    }

    // scan rows
    for( i = 0; i < ndim; i++ ){
	  register float * pr;
	  register int j;

    // set pr = pointer to a real row
	  j = ys + i;
	  if( j < 0 ) j = 0;
	  else if( j >= srchgt ) j = srchgt - 1;
      // choose source pointer 
	  if( csrc ) pr = (float *) csrc + srcwid * j;
	  else pr = (float *) vsrc[j];

	// set row[i] = pointer to real data
	  if( k == 0 ) row[i] = pr + xs;	// region is in source
	  else {							// must copy region to cdata...
		  register float * pc = &cdata[i];
		  row[i] = pc;
		  if( wrap_x ){					// copy both ends of row
			  register int l = k;
			  if( l > 0 ) l -= ndim;
			  j = ndim + l;
			  for( ; l < 0; l++ ) *pc++ = pr[l+srcwid];
			  for( ; l < j; l++ ) *pc++ = pr[l];
		  } else {						// copy one end of row,,,
		    register int l = k;
			if( l < 0 ){
				j = ndim + l;
			    for( ; l < 0; l++ ) *pc++ = *pr;
		        for( ; j > 0; j-- ) *pc++ = *pr++;
			} else {
				j = ndim - l;
				pr += srcwid - j;
			    for( ; j > 0; j-- ) *pc++ = *pr++;
				--pr;
		        for( ; l > 0; l-- ) *pc++ = *pr;
			}
		  }
	  }
    }
  } else {			// col vector format
  // fractions transposed
	Dy = Xpos - (double) xc; 
	Dx = Ypos - (double) yc; 
  // k = number of rows outside the col, sign idicates end
    if( ys < 0 ) k = ys;  // - at top
    else {
      k = ys + ndim - srchgt; // + at bottom
	  if( k < 0 ) k = 0;	// all inside
    }

    // scan cols
    for( i = 0; i < ndim; i++ ){
	  register float * pc;
	  register int j;

    // set pc = pointer to a real col
	  j = xs + i;
	  if( j < 0 ){
		  if( !wrap_x ) j = 0;
		  else j += srcwid;
	  }
	  else if( j >= srcwid ) {
		if( wrap_x ) j -= srcwid;
		else j = srcwid - 1;
	  }
	  pc = (float *) vsrc[j];

	// set row[i] = pointer to real data
	  if( k == 0 ) row[i] = pc + ys;	// region is in source
	  else {							// must copy region to cdata...
		register float * p = &cdata[i];
	    register int l = k;
		row[i] = p;
		if( l < 0 ){
			j = ndim + l;
		    for( ; l < 0; l++ ) *p++ = *pc;
	        for( ; j > 0; j-- ) *p++ = *pc++;
		} else {
			j = ndim - l;
			pc += srchgt - j;
		    for( ; j > 0; j-- ) *p++ = *pc++;
			--pc;
	        for( ; l > 0; l-- ) *p++ = *pc;
		}
	  }
    }
	Dy = Xpos - (double) xc;	// note transposed
	Dx = Ypos - (double) yc; 
  }
  
 // interpolate        
  { 
    static double yr[ndim], wx[ndim], wy[ndim];
    register double rd;
	register int k,i;

	intpol( Dx, wx, ndim )

    for( k = 0; k < ndim; k++ ){
      yr[k] =  row[k][0] * wx[0];                                     
      for(i=1; i<ndim; i++) yr[k] += row[k][i] * wx[i];
	}

	intpol( Dy, wy, ndim )

	rd = yr[0] * wy[0];
	for( i = 1; i < ndim; i++ ) rd += yr[i] * wy[i]; 
	
	*pdest = (float)rd;
  }

  return 1;
}
