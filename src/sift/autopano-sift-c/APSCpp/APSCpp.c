/* APSCpp.cpp  04 Mar 2008 TKS
  various routines for enhanced autopano-sift-c
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
#include "AutoPanoSift.h"
#include "sphereAlign.h"
#include "saInterp.h"
#include <math.h>
#include <tiffio.h>

// create empty
pDIinfo DIinfo_new0(){
	pDIinfo p = (pDIinfo)malloc(sizeof(DIinfo));
	if( p ) memset( p, 0, sizeof(DIinfo) );
	return p;
}

// delete
void DIinfo_delete( pDIinfo p ){
	free( p );
}


/*
  Get source image info from a panotools project file.

  Adapted from "optimizer script parser" in panotools\parser.c
  Should work with panotools, ptasmblr and hugin projects.
  Ignores everything except 'i' lines (unless there are none,
  when it scans the file again looking for 'o' lines).

  input 
     project file name
  output 
     adds a DisplayImage struct to DIlist for each "i" or "o" line
	 with only the following fields set
	   name
	   width
	   height
	   hfov
	   format
	   yaw
	   pitch
	   roll
  returns
     0 on error else number of image lines found

 Notes
   ignores all optimizable params except hfov, y, p, r; 
   handles back references ("v=n", etc) by copying the value from 
   the referenced image.

   accepts all format codes verbatim (original checks them)
*/
#define LINE_LENGTH 512
#define READ_VAR( fmt, ref )  \
	sscanf( ++li, fmt, ref ); \
	if( *li == '"' ) do ++li; while(*li != '"'); \
	while(*li > ' ') li++;

int LoadProjectImages( char* script, ArrayList * DIlist )
{
    DIinfo  *im, *ir;  // DIlist items
    
    // Variables used by parser
    
    char                *li, line[LINE_LENGTH], *ch;
    int                 lineNum = 0;
    int                 k;
    int                 numIm = 0;
	int   baseIdx = ArrayList_Count( DIlist );	// 1st image goes here

	int keychar;

//    setlocale(LC_ALL, "C");
// loop over 'i', 'o' linestart
	for( keychar = 'i';; ){
		FILE * fp = fopen( script, "r" );
		if( !fp ) return 0;

		// Parse script 
	     
		while( (ch = fgets(line, LINE_LENGTH, fp)) )
		{
			lineNum++;
	   
			if( line[0] == keychar 
			   && isspace( line[1] ) )
			{ 
			// create new Image descriptor
				im  = DIinfo_new0(); 
			// fill it from line      
				li = &(line[1]);
				while( *li != 0 && *li != '\n')
				{
					switch(*li)
					{
						case 'w':   READ_VAR( "%d", &(im->width) )
									break;
						case 'h':   READ_VAR( "%d", &(im->height))
									break;
						case 'v':   if( *(li+1) == '=' ){
									// copy referenced value
									   ++li;
									   READ_VAR( "%d", &k )
									   ir = (DIinfo *)ArrayList_GetItem( DIlist, baseIdx + k );
									   im->hfov = ir->hfov;
									}else{
									   READ_VAR( "%lf", &(im->hfov));
									}
									break;
						case 'f':
									READ_VAR( "%d", &(im->format) );
									break;
						case 'y':   if( *(li+1) == '=' ){
									// copy referenced value
									   ++li;
									   READ_VAR( "%d", &k )
									   ir = (DIinfo *)ArrayList_GetItem( DIlist, baseIdx + k );
									   im->yaw = ir->yaw;
									}else{
										READ_VAR( "%lf", &(im->yaw));
									}
									break;
						case 'p':   if( *(li+1) == '=' ){
									// copy referenced value
									   ++li;
									   READ_VAR( "%d", &k )
									   ir = (DIinfo *)ArrayList_GetItem( DIlist, baseIdx + k );
									   im->pitch = ir->pitch;
									}else{
										READ_VAR("%lf", &(im->pitch));
									}
									break;
						case 'r':   if( *(li+1) == '=' ){
									// copy referenced value
									   ++li;
									   READ_VAR( "%d", &k )
									   ir = (DIinfo *)ArrayList_GetItem( DIlist, baseIdx + k );
									   im->roll = ir->roll;
									}else{
										READ_VAR("%lf", &(im->roll));
									}
									break;
						case 'n':           // Set filename
								{	char *p = im->name;
									// assume always quoted
									READ_VAR("\"%[^\"]\"", p);
								}
									break;
						default:
							break;
					}
					while( *li > ' ' ){	// skip next "word"
						if( *li == '"' ) do ++li; while( *li != '"' ); // quoted strings
						++li;
					}
					if(*li) li++;	// skip one nonnull

				} // EOL

			 // add this image to list
				numIm++;
				ArrayList_AddItem( DIlist, im );

		 } // end of line
	  } // end of file

	  fclose( fp );

	  if( numIm > 0 || keychar == 'o' ) break;
	  keychar = 'o';  // try again...
  }


  return numIm;

}

/* Dump an ImagMap to a tiff file
  return success or failure
*/
int ImageMap_toTIFF( ImageMap * p, char * name )
{
	int i, ok;

// create tiff file
    TIFF * tp = TIFFOpen( name, "w" );
	if( !tp ) return 0;

// set tiff tags  
	TIFFSetField( tp, TIFFTAG_IMAGEWIDTH, p->yDim ); 
	TIFFSetField( tp, TIFFTAG_IMAGELENGTH, p->xDim ); 
	TIFFSetField( tp, TIFFTAG_PLANARCONFIG, 1 ); // packed pixels
	TIFFSetField( tp, TIFFTAG_ROWSPERSTRIP, 1 );  // separate rows
	TIFFSetField( tp, TIFFTAG_COMPRESSION, 1 );  // uncompressed
	TIFFSetField( tp, TIFFTAG_SAMPLESPERPIXEL, 1 );	// monochrome
	TIFFSetField( tp, TIFFTAG_PHOTOMETRIC, 1 );		// greyscale
	TIFFSetField( tp, TIFFTAG_BITSPERSAMPLE, 32 ); 
	TIFFSetField( tp, TIFFTAG_SAMPLEFORMAT, 3 );	// IEEE float

// write the data rowwise
	for( i = 0; i < p->xDim; i++ ){
		if((ok = TIFFWriteScanline( tp, p->values[i], i, 0 )) != 1 ) break;
	}

// done
	TIFFClose( tp );
	return ok;

}

/*  subroutine equivalent to GenerateKeys.c 

  for APSC++, new argumnent pdi -> DIinfo struct with the input 
  projection format and angular width into needed to set up a
  remapping to stereographic format.  If pdi is 0, behaves like 
  GenerateKeys() with these exceptions.

  1) it sets the Lowe routine's prefilter sigma to 0.5 if the image stays
  at original size, or to 0 (no further smoothing) if it is reduced here.
  
  2) it always reduces image size (when maxdim < largest input dimension) 
  by a smooth interpolation rather than simple decimation.

  3) it never invokes the Lowe detector's option to expand the image 2x.

  returns 0 for failure

  06 Mar 2008 new mode argument.
    0 for legacy (KDTree) mode -- returns list of KeypointN
    1 for ANN mode -- returns list of Keypoint

*/

KeypointXMLList * GenerateKeyspp( char * imgname, int maxdim, DIinfo * pdi, int mode )
{  
#ifdef _DEBUG_INTERP
	char tmp[262];
#endif
	int nKeys;
	int pW, pH;
	double Xcen, Ycen;
	saRemap *prm = 0;
    ImageMap* picMap = 0;
	LoweFeatureDetector* lf;
	KeypointXMLList* kpp;

	double Scale = 1.0;


// reduce Lowe prefilter smoothing
	double savesigma = LoweFeatureDetector_SetPreprocSigma( 0.5 );

/* read the image */

	DisplayImage* pic = DisplayImage_new(imgname);
  // (if we get here the image was read OK, and its name printed)
	if(!pdi) {
		WriteLine("  width %d  height %d", pic->width, pic->height );
	} else {
		WriteLine("  width %d  height %d  format %d  hfov %g", 
			pic->width, pic->height, pdi->format, pdi->hfov );
	// put actual image dimensions in the DIinfo, just in case...
		pdi->width = pic->width; pdi->height = pic->height;
	}

	pW = pic->width;
	pH = pic->height;
	Xcen = 0.5 * pW;
	Ycen = 0.5 * pH;


// convert to monochrome float and discard input image
	picMap = DisplayImage_ConvertToImageMap( pic );
	DisplayImage_delete(pic);

#ifdef _DEBUG_INTERP
	strcpy(tmp, imgname);
	strcat(tmp,"-MAP.TIF");
	ImageMap_toTIFF( picMap, tmp );
#endif

/* reduce size if required
  by interpolating in smoothed map
  also convert to stereographic if pdi is valid
*/
  // prescale factor
	if( maxdim > 0) Scale = max( pW, pH ) / (double) maxdim;
	if( Scale <= 1 ) Scale = 1;

 // interpolate if necessary
	if( pdi || Scale > 1 ){
		ImageMap * pn;
		int x, y;
		double X, Y;
		int dwid = pW, dhgt = pH;
	// reduced dimensions
		if( Scale > 1 ){
			dwid = (int)(0.5 + pW / Scale );
			dhgt = (int)(0.5 + pH / Scale );
			Scale = (double) pW / (double) dwid;	// exact
		WriteLine("  reduce size to %d x %d", dwid, dhgt );
		}
	// create result map
		pn = ImageMap_new( dwid, dhgt );
	// smooth the source
		//ImageMap_GaussianConvolution( picMap, 0.75 * Scale ); // NO EFFECT.
		LoweFeatureDetector_SetPreprocSigma( 0 );
	// set up coordinate mapping
		if( pdi ){			// ++ mode
		  CamLens * psp, * pdp;
	    // create the projections
		  psp = CamLens_new1( pW, pH, pdi->format, pdi->hfov, 16.0 );
		  pdp = CamLens_new1( dwid, dhgt, _stereographic, pdi->hfov, 0 );
	    // create the mapping
		  prm = saRemap_new( psp, pdp );
		  saRemap_inv ( prm, 0.5 * dwid, 0.5 * dhgt, &Xcen, &Ycen );
		  WriteLine("  use stereographic projection");
		}
	// interpolate
		saInterpSetup((void *)picMap->values,	// void * pdata
						 2,					// format is column vectors
						 picMap->xDim,		// int wid,
						 picMap->yDim,		// int hgt,
						 1,					// samples per pixel
						 picMap->xDim,		// pixels per row
						 0,					// char wrap, // required
	 					 0,					// char * pmask,	
						 0,	0, 0			// mask params
					  );
		for( x = 0; x < pn->xDim; x++ ){
			float * pd = pn->values[x];
			for( y = 0; y < pn->yDim; y++ ){
				X = (double) x; Y = (double) y;		
				if( prm ) saRemap_inv ( prm, X, Y, &X, &Y );	// ++
				else { X *= Scale; Y *= Scale; }				// legacy
				saInterp_float( &(pd[y]), X, Y );
			}
		}
	// swap
		ImageMap_delete( picMap );
		picMap = pn;
		saRemap_delete( prm ); prm = 0;
#ifdef _DEBUG_INTERP
	strcpy(tmp, imgname);
	strcat(tmp,"-MAP-PROJ.TIF");
	ImageMap_toTIFF( picMap, tmp );
	WriteLine("ImageMap dumped to %s", tmp );
	exit(0);
#endif
	}


/* find the features 
*/

	lf = LoweFeatureDetector_new0();
	nKeys = LoweFeatureDetector_DetectFeaturesDownscaled (lf, picMap, 0, Scale);
	WriteLine("  %d keypoints found\n", nKeys );

 /* build the return value
     For KDTree mode, a list of KeypointN; for ANN mode a list of Keypoint, 
	 that will be converted to keypointN after matching
 */
  {
    
	ArrayList * globalNaturalKeypoints = 0;
	int i;

	if( mode == 0 ) globalNaturalKeypoints = ArrayList_new0 ((void *) KeypointN_delete);

	if( pdi ){ // build remapping function at input scale
	  CamLens * psp, * pdp;
	  psp = CamLens_new1( pW, pH, pdi->format, pdi->hfov, 16.0 );
	  pdp = CamLens_new1( pW, pH, _stereographic, pdi->hfov, 0 );
	  prm = saRemap_new( psp, pdp );	// dest=>src
	} else prm = 0;

	for(i=0; i < ArrayList_Count(lf->globalKeypoints); i++) {
		Keypoint* kp = (Keypoint *) ArrayList_GetItem( lf->globalKeypoints, i );
		if( prm ){		// remap coordinates to image projection
			saRemap_inv ( prm,
						  kp->x , kp->y ,	// scale to image size
						  &kp->x, &kp->y		// remapped coords
						);
		}
		if( mode == 0 ) ArrayList_AddItem ( globalNaturalKeypoints, KeypointN_new( kp ));
	}

  // package the list
	kpp = KeypointXMLList_new0();
	kpp->imageFile = strdup(imgname);
	kpp->xDim = pW;
	kpp->yDim = pH;
	if( mode == 0 ){
	// return the KeypointN list
		kpp->array = globalNaturalKeypoints;
	} else {
	// return the Keypoint list
		kpp->array = lf->globalKeypoints;
		lf->globalKeypoints = 0;	// don't delete the list!
	}
  }
  // delete the LoweDetector
	LoweFeatureDetector_delete(lf);
  // restore the default Lowe prefilter
	LoweFeatureDetector_SetPreprocSigma( savesigma );
  // disable the patent warning message for any subsequent images
    LoweFeatureDetector_SetPrintWarning(false);	

	return kpp;
}


/* Match keypoints with ANN kd-tree
  
  argument is a MultiMatch loaded with Keypoints
  returns with matchlist filled in and the Keypoints
  replaced by KeypointN n both keySets and globalKeys
  (caution these point to same KeypointN's)

  returns number of matches, or 0 on error

*/
#ifdef __cplusplus
extern "C" {
#endif
// wrapper fns in ANNkd_wrap.cpp
void ANNkd_new( int sd, double **pts, int n, int d );
void ANNkd_search( double * pt, int nnabe, int *indx, double *dist, double nneps );
void ANNkd_delete( void );
#ifdef __cplusplus
}
#endif

int MultiMatch_ANN ( MultiMatch* self ){

// nearest neighbor search parameters
#define nnabe 2			// only need 2, self-match is disabled in ANN
#define nneps 0.0
#define mindepth 180	// Nowozin tree uses 130
// search results
    double dist[nnabe];		// an array of squared distances 
    int    indx[nnabe];		// an array of point indices

	int m = 0;	// return value
	int i, j, k, n, d;
	Keypoint * kp;

  // count keypoints
	n = 0;		// number of keypoints
	for(i=0; i<ArrayList_Count(self->keySets); i++) {
		KeypointXMLList* list = (KeypointXMLList*) ArrayList_GetItem(self->keySets, i);
		n += ArrayList_Count( list->array );
	}
  /* make array of pointers to coordinates rq'd by ANN
     and array of pointers to corresponding keypointN.s
  */
	double ** pts = (double **)malloc( n * sizeof(double **));
	KeypointN ** kpn = (KeypointN **)malloc( n * sizeof(KeypointN **));
  // fill pts with pointers to Keypoint feature vectors 
  // and kpn with pointers to equivalent KeypointNs
	k = 0;
	for(i=0; i<ArrayList_Count(self->keySets); i++) {
		KeypointXMLList* list = (KeypointXMLList*) ArrayList_GetItem(self->keySets, i);
		for(j=0; j<ArrayList_Count(list->array); j++) {
			kp = (Keypoint*) ArrayList_GetItem(list->array, j);
			pts[k] = kp->featureVector;	
		// Make the KeypointN 
			kpn[k++] = KeypointN_new( kp );
		}
	}

	d = kp->featureVectorLength; // # of dimensions

// search cutoff depth 
	int sd = (int)( mindepth * max( 1.0, log ((double) n) / log (1000.0) ));

  /* build the kd-tree
  */

	ANNkd_new( sd, pts, n, d );
	
	if (self->verbose){
		WriteLine ("ANN kd-tree: %d keypoints, cutoff depth %d", n, sd);
	}

  // find near neighbors of each key
	k = 0;
	for( i = 0; i < n; i++ ) {
	// report progress
		if (self->verbose) {
			if ((i % 500) == 0)
				Write ("\r%2.2f%%, %d/%d        ",
					   (100 * ((double) i)) / ((double) n),
					   i, n);
		}

	// find neighbors

		ANNkd_search( pts[i], nnabe, indx, dist, nneps );


	/* add valid neighbor pairs to global matches list
		NOTE Matches point to KeypointN's made above
		NOTE ANN returns squared distances

		Nowozin accepts only the first neighbor as a match, and only if it is 
		significantly nearer than the 2nd neighbor.  His Match structure includes 
		both distances.

		The code below disallows matches to points that have already been tested.
		That prevents all duplicate matches, at the cost of losing possible control 
		points shared by 3 or more images -- but Nowozin's method loses those too.
	*/
		if( indx[0] > i && dist[0] / dist[1] < 0.36 ){
			ArrayList_AddItem( self->globalMatches, 
			              Match_new (kpn[i], kpn[indx[0]], 
						  sqrt(dist[0]), sqrt(dist[1]))
							 );
			m++;
		}
	}
  
	if (self->verbose) {
		Write ("\r %2.2f%%, %d/%d        ",
		       (100 * ((double) i)) / ((double) n),
		       i, n );
		
		WriteLine ("\nGlobal match search yielded %d matches", m );
	}

  /* Replace Keypoints with KeypointNs in keySets
	 NOTE globalKeys list is not needed as only KDTree code uses it
  */
	k = 0;
	for(i=0; i<ArrayList_Count(self->keySets); i++) {
		KeypointXMLList* list = (KeypointXMLList*) ArrayList_GetItem(self->keySets, i);
		for(j=0; j<ArrayList_Count(list->array); j++) {
			kp = (Keypoint*) ArrayList_GetItem(list->array, j);
			ArrayList_SetItem( list->array, j, (void *)kpn[k] );
//// Not needed			ArrayList_AddItem( self->globalKeys, (void *)kpn[k] );
			k++;
			Keypoint_delete( kp );
		}
	}

  // tidy
	free( pts );
	free( kpn );
	ANNkd_delete();

	return m;
}
