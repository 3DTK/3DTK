/* saInterp.h

  interpolation engines for SphereAlign,  
  specialized from Dersch's panotools code.

  Source may a contiguous array.or a vector of pointers
  to rows.  It may be treated as cylindrical in X.
  
  The optional mask must be a contiguous array (treated as 
  a packed array of bytes).  It may be stored with the
  source if type is compatible, or can be separate.  Only
  one byte of mask is tested per pixel, and only one bit
  of information is used: zero (out) or nonzero (in).
  
  InterpSetup() posts constant info on the source and mask.
  There are two source pointers, one for a contiguous source
  and one for a vector source -- the "wrong" one must be 0.
  Source pointers are untyped; you have to call the right
  interpolator for the actual pixel type --

	saInterp_float()  -- one float value per pixel
	saInterp_short()  -- packed signed short
	saInterp_byte()	  -- packed unsigned char

  The interpolators set one destination pixel per call.  If the given
  position (index units) is inside the (masked) source they return 1, 
  otherwise 0 with the destination pixel set to 0.

  Interpolators that can handle packed source pixels also take a sample 
  index.  However the destination pointer is always absolute.

  Uses only one interpolation kernel, the 6x6 spline 
  (you could change that at compile time).
    
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

#ifndef _SAINTERP_H
#define _SAINTERP_H

#ifdef __cplusplus
extern "C" {
#endif

int saInterpSetup( void * ps,	// -> source data or pointers
				  int vfmt,		// 0: contig 1: row vector 2: col vector
				  int wid,		// valid pixels per row
				  int hgt,		// valid rows
				  int spp,		// samples per pixel (>0)
				  int ppr,		// actual pixels per row (>=wid)
				  char wrap,	// true is X is cyclic
	 		      char * pmask,	// -> mask array, 0 if no mask
				  int mspp,		// mask samples per pixel
			      int mppr,		// actual pixels per row (>= wid)
				  int midx		// index of mask byte in pixel
			     );

int saInterp_float( float * pdest,
					double Xpos, double Ypos
				  );

#ifdef __cplusplus
}
#endif

#endif  //ndef _SAINTERP_H

