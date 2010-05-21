/*  CamLens.c   20 Feb 2008 TKS

  implementaton of CamLens class
  
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

#include "sphereAlign.h"
#include <stdlib.h>
#include <memory.h>
#include <math.h>

/* radius <=> angle functions for lens projections
  R is noramlized image radius
  K is an adjustable parameter
  A is angle in radians
  NOTE no error checks, pass only reasonable values!
			 |A|		 |R|
  rect		<Pi/2		 any
  fish		 any        <= K
  mirr		 any		<= 2
  ster		 <Pi		 any
*/

static double r2a_rect( double R, double K ){
	return atan( R );
}

static double a2r_rect( double A, double K ){
	return tan( A );
}

static double r2a_fish( double R, double K ){
	return K * asin( R / K );
}

static double a2r_fish( double A, double K ){
	return K * sin( A / K );
}

static double r2a_mirr( double R, double K ){
	return  asin( 0.5 * R );
}

static double a2r_mirr( double A, double K ){
	return  sin( 0.5 * A );
}


static double r2a_ster( double R, double K ){
	return  2 * atan( R );
}

static double a2r_ster( double A, double K ){
	return tan(0.5 * A);
}



// create empty
pCamLens CamLens_new0()
{
	pCamLens p = (pCamLens) malloc(sizeof(CamLens));
	if( p ) memset(p, 0, sizeof(CamLens));
	return p;
}

/* initialize with minimal info
   fmt is a PT format code
   fails if fmt is not a "camera" projection
   hfov is the angle corresponding to wid
   flmm should be 0 for an ideal projection, >0 for a real lens
   other assumptions see CamLens_setProjection()

*/
pCamLens CamLens_new1( int wid, int hgt, int fmt, double hfov, double flmm )
{
	pCamLens p;
	double R, x, y;

	if( fmt < _rectilinear || fmt > _stereographic ) return 0;
	p = CamLens_new0();
	p->FLmm = flmm;
	if( CamLens_setProjection( p, fmt )){

		p->widpix = wid;
		p->hgtpix = hgt;
		p->FLmm = flmm;

	   // focal lengths in pixels
		R = CamLens_RofA( p, DEG2RAD( 0.5 * hfov ) ); 
		p->hFLpix = p->vFLpix = 0.5 * p->widpix / R;

	   // projection center
		p->hCpix = 0.5 * wid;
		p->vCpix = 0.5 * hgt;

	  // default crop radius circumscribes raster
        x = y = 0;
		p->Rcrop = CamLens_getR( p, x, y );

	   // reference radius inscribed in raster
		if( p->hFLpix * p->vCpix > p->vFLpix * p->hCpix )
			y = p->vCpix;	// wid < hgt
		else x = p->hCpix;	// wid >= hgt
		p->Rref = CamLens_getR( p, x, y );

	}

	if( p->widpix < 1 || p->hgtpix < 1 
		|| p->hFLpix == 0 ){
		free(p);
		p = 0;
	}

	return p;
}

/* set projection format

  fmt is an image format code defined in panorama.h

  returns 0 for unsupported format
          1 for OK
			
  Assumes --
  ideal projection if FLmm == 0, else lens projection
  equal-area fisheye lens for _equirectangular (the
    ideal is equal-angle)
  lens projects Y of cylindrical and equirectangular 
    and both axes of the elliptical formats

*/
int CamLens_setProjection( pCamLens p, int fmt )
{ // null format
	p->PT_fmt = -1;
	p->R2A = 0;
	p->A2R = 0;
	p->Klens = 0;
	p->lensAxis = (p->FLmm ? 3 : 0 ); // elliptical
    p->Rref = p->Rcrop = 0; // no limit
	switch( fmt ){
	case _rectilinear:
		p->R2A = r2a_rect;
		p->A2R = a2r_rect;
		p->Klens = 1.0;
		break;
	case _panorama:
		p->R2A = r2a_rect;
		p->A2R = a2r_rect;
		p->Klens = 1.0;
		p->lensAxis &= 2;  // lens Y only
		break;
	case _fisheye_circ:
	case _fisheye_ff:
		p->R2A = r2a_fish;
		p->A2R = a2r_fish;
		p->Klens = 2.0;
  		p->Rref = p->Rcrop = p->Klens;
		break;
	case _equirectangular:
		if( p->FLmm ){
			p->R2A = r2a_fish;
 			p->A2R = a2r_fish;
			p->Klens = 2.0;
	  		p->Rref = p->Rcrop = p->Klens;
			p->lensAxis = 2;	// Y only
		} 
		break;
	case _spherical_cp:
	case _spherical_tp:
		if( p->FLmm ){
			p->R2A = r2a_fish;
 			p->A2R = a2r_fish;
			p->Klens = 500.0;
	  		p->Rref = p->Rcrop = p->Klens;
		} 
		break;
	case _mirror:
		p->R2A = r2a_mirr;
		p->A2R = a2r_mirr;
		p->Klens = 1.0;
		break;
	case _orthographic:
		p->R2A = r2a_fish;
		p->A2R = a2r_fish;
		p->Klens = 1.0;
  		p->Rref = p->Rcrop = p->Klens;
		break;
	case _stereographic:
		p->R2A = r2a_ster;
		p->A2R = a2r_ster;
		p->Klens = 1.0;
		break;

	default:
		p->lensAxis = 0;
		return 0;	// unsupported format

	}
  // OK
	p->PT_fmt = fmt;
	return 1;		// OK
}

// initialize with full info
pCamLens CamLens_new( pCameraInfo pc, pLensInfo pl )
{
	return 0;  /// TBD ///
}

// normalized radius to angle in radians
double CamLens_AofR( pCamLens p, double R ){
   if( p->R2A == 0 ) return R;
   return (p->R2A)( R, p->Klens );
}

// angle in radians to normalized radius
double CamLens_RofA( pCamLens p, double A ){
   if( p->A2R == 0 ) return A;
   return (p->A2R)( A, p->Klens );
}

// get largest valid normalized radius
double CamLens_Rmax( pCamLens p ){
	return p->Rcrop;
}


/* pixel coordinates to normalized radius */
double CamLens_getR( CamLens *p, double x, double y )
{
	double dx = ( x - p->hCpix ) / p->hFLpix;
	double dy = ( y - p->vCpix ) / p->vFLpix;
	return sqrt( dx * dx + dy * dy );
}

