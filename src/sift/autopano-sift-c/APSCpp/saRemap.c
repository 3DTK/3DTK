/*  saRemap.c  19 feb 2008 TKS

  Coordinate mapping for interconverting image 
  projections that involve a lens.
  
  Both 'source' and 'destination' projections can be 
  elliptical, with independent aspect ratios.
  
  A CamLens struct supplies lens parameters for the 
  'source' projection.  The 'destination' projection 
  is the ideal one associated with a PT format code
  (except that it may be elliptical).
  
  Each image has an origin, which is the position of
  the projection center in raster coordinates. A pair
  of "FL's in pixels" sets the magnification and
  aspect ratio of the destination image (imagine an
  intermediate spherical projection).
    
  The radial remapping function is tabulated as a
  cubic spline.  In practical cases this will be
  monotonic, so its inverse is tabulated too. Then 
  coordinates can be remapped in either direction.

  The range of tabulated source radii is 0 thru the
  Rmax of the source CamLens; so only points inside 
  that circle can be remapped.

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
#include "HermiteSpline.h"

#include <stdlib.h>
#include <math.h>

#define SARM_NPTS	50;

saRemap * saRemap_new0( void ){
  saRemap * p = (saRemap *)malloc(sizeof(saRemap));
  if( p ){
	  p->npts = 0; // mark empty
	  p->xval = p->yval = 0;  // no arrays..
	  p->Cxy = p->Cyx = 0;
  }
  return p;
}

void saRemap_delete( saRemap *p ){
	if( p ){
		if( p->npts ){
			if(p->xval) free(p->xval); // does yval too
			if(p->Cxy) free(p->Cxy);
			if(p->Cyx) free(p->Cyx);
		}
		free( p );
	}
}


saRemap * saRemap_new( CamLens * ps, CamLens * pd )
{
	saRemap * p = saRemap_new0();
	if( p ){
		p->sX0 = ps->hCpix; 
		p->sY0 = ps->vCpix;
		p->sFx = ps->hFLpix; 
		p->sFy = ps->vFLpix;
		p->dX0 = pd->hCpix;
		p->dY0 = pd->vCpix;
		p->dFx = pd->hFLpix; 
		p->dFy = pd->vFLpix;

	// create the table
	    p->npts = SARM_NPTS;
		p->xval = (double *)malloc(2 * p->npts * sizeof(double));
		p->yval = p->xval + p->npts;
	 // tabulate source radius mapping function
	  { int i;
	    double * x = p->xval, 
			   * y = p->yval, 
			   * t = (double *)malloc(p->npts * sizeof(double));
	// tabulate the radial function 
		double s = CamLens_Rmax( ps ) / (p->npts - 1);
		for( i = 0; i < p->npts; i++ ){
			x[i] = i * s;
			y[i] = CamLens_RofA( pd, CamLens_AofR( ps, x[i] ) );
		}

	// set up spline table
	  // tangents for forward spline
		i = spline_tangents_set( p->npts, x, y, t );
	  // fail if y is not monotonic
		if( i != 1 ){
			saRemap_delete( p );
			return 0;
		}
	  // coefficients for forward spline
		p->Cxy = spline_hermite_set ( p->npts, x, y, t );
	  // tangents for inverse spline
		i = spline_tangents_set( p->npts, y, x, t );
	  // coefficients for inverse spline
		p->Cyx = spline_hermite_set ( p->npts, y, x, t );

		free( t);
	  }

	}
	return p;
}


/* convert pixel coordinates
  if fn is not defined at the given input point,
  return 0 with output = (0,0)
  Note "non-lens" coordinates just shift to new origin 
*/
int saRemap_fwd ( saRemap *p,
				  double xsrc, double ysrc,
				  double *xdest, double *ydest )
{	double dx, dy, x = 0, y = 0;
	double r, R;
	*xdest = *ydest = 0;
	if( p->npts < 2 || p->Cxy == 0 ) return 0;
  // center
    dx = xsrc - p->sX0;
	dy = ysrc - p->sY0;
  // normalize
	x = dx / p->sFx; 
	y = dy / p->sFy; 
  // convert radius
	r = sqrt( x * x + y * y );
	if(!spline_hermite_val ( p->npts, p->xval, p->Cxy, r, &R, 0 )) return 0;
  // post result
	if( r ) R /= r;
	*xdest = x * R * p->dFx + p->dX0;
	*ydest = y * R * p->dFy + p->dY0;

	return 1;
}

int saRemap_inv ( saRemap *p,
				  double xdest, double ydest,
				  double *xsrc, double *ysrc  )
{	double dx, dy, x, y;
	double r, R;
	*xsrc = *ysrc = 0;
	if( p->npts < 2 || p->Cyx == 0 ) return 0;

    dx = xdest - p->dX0;
	dy = ydest - p->dY0;
	x = dx / p->dFx;
	y = dy / p->dFy;

	r = sqrt( x * x + y * y );
	if(!spline_hermite_val ( p->npts, p->yval, p->Cyx, r, &R, 0 )) return 0;
    if(r) R /= r;
	*xsrc = x * R * p->sFx + p->sX0;
	*ysrc = y * R * p->sFy + p->sY0;

	return 1;
}
