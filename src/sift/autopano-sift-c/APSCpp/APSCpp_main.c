
/* APSCpp_main.c  18 Feb2008 TKSharpless

 enhanced version of APSC for Hugin 0.7 that can transform images to
 stereographic projection for better keypoint matching.

 The enhanced mode is invoked by giving it a project file in place
 of the image files.  The project file 'i' lines supply information
 about the images' projections, as well as their file names.  This
 info is stored in DIlist.  The keypoint generator (and the refine
 keypoints step if used) converts intensity maps to stereographic 
 projection using info from DIlist. At tne final output-to-pto stage 
 the control point coordinates get projected back to image space.

 The projections are "ideal" (no lens or center point corrections), but
 the reprojection machinery could handle those if available.

 * Keypoint file correlation and hugin panorama file creation utility.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 *
 * "The University of British Columbia has applied for a patent on the SIFT
 * algorithm in the United States. Commercial applications of this software
 * may require a license from the University of British Columbia."
 * For more information, see the LICENSE file supplied with the distribution.
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
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

#include <time.h>

#include "AutoPanoSift.h"
#include "sphereAlign.h"

// APSCpp.cpp
int LoadProjectImages( char* script, ArrayList * DIlist );
KeypointXMLList * GenerateKeyspp( char * imgname, int maxdim, DIinfo * pdi, int mode );
int MultiMatch_ANN ( MultiMatch* );

void Usage ()
{
    WriteLine ("autopano-sift-c: Find control points giving a Hugin project file\n");
    WriteLine ("   Version %s for hugin 0.7\n", PACKAGE_VERSION);
	WriteLine ("autopano-sift-c [options] output.pto image1 image2 [..]");
	WriteLine ("                hugin project file output.pto gets the results ");
	WriteLine ("                input images can be jpeg, tiff, or other formats.\n");
	WriteLine ("autopano-sift-c [options] output.pto projectfile");
	WriteLine ("                projectfile: a hugin project or other PT compatible");
    WriteLine ("                script with image file names, projections and angular");
    WriteLine ("                widths.  Enables stereographic projection.\n");

	WriteLine ("Options");
	WriteLine ("  --ransac <on|off|1|0>   Switch RANSAC filtration on or off (default: off).");
	WriteLine ("  --maxmatches <matches>  Output no more that this many control points per");
	WriteLine ("                          image pair (default: 25, zero means unlimited)");
	WriteLine ("  --maxdim <n>            Make largest image dimension <= n (default: 1600).");
	WriteLine ("  --projection <n>,<d>    n = PT format code, d = hfov in degrees.  These");
	WriteLine ("                          apply to all images, reprojection is enabled.");
	WriteLine ("  --ANNmatch <on|off|1|0> Use fast keypoint matching tree (default: on).");
	WriteLine ("  --disable-areafilter    Do not use max-area filtration, which is default.");
	WriteLine ("                          See manpage for details.");
	WriteLine ("  --integer-coordinates   Truncate match coordinates to integer numbers.");
	WriteLine ("  --absolute-pathnames <on|off|1|0>   Use the absolute pathname of the image");
	WriteLine ("                          file in the PTO output file. Disabled by default.");
    WriteLine ("");
	
	WriteLine ("Alignment options");
	WriteLine ("  --align                 Automatically pre-align images in PTO file.");
	WriteLine ("  --bottom-is-left");
	WriteLine ("  --bottom-is-right       Use in case the automatic algorithm fails.");
	WriteLine ("  --generate-horizon <c>  Generate up to 'c' horizon lines.");
	WriteLine ("");
	
	WriteLine ("Refinement options");
	WriteLine ("  --refine                Refine the found control points using the");
	WriteLine ("                          original images.");
	WriteLine ("  --refine-by-middle      Use the best middle point to refine (default).");
	WriteLine ("  --refine-by-mean        Use the mean of the patches control points.");
	WriteLine ("  --keep-unrefinable <on|off|1|0>");
	WriteLine ("                          Keep unrefinable matches (default: on).");
										
	WriteLine ("output.pto: The output PTO panorama project file.");
	WriteLine ("    The filename can be \"-\", then stdout is used");
	WriteLine ("image<n>: input image files (any common format: JPEG, PNG, TIFF, ..)");
	WriteLine ("Notice: for the aligning to work, the input images shall be");
	WriteLine ("  1. All of the same dimension and scale");
	WriteLine ("  2. The first images must be an ordered row. See manpage.");
	WriteLine ("");
}


typedef struct Resolution Resolution;
struct Resolution
{
	int x, y;
};

Resolution* Resolution_new0()
{
	Resolution* self = (Resolution*)malloc(sizeof(Resolution));
	return self;
}

Resolution* Resolution_new(int x, int y) 
{
	Resolution* self = Resolution_new0();
	self->x = x;
	self->y = y;
	return self;
}

void Resolution_delete(Resolution* self) 
{
	if (self) {
		free(self);
	}
}

int Resolution_CompareTo (Resolution* self, int x, int y)
{
	
	if (self->x == x && self->y == y)
		return (0);
	
	return (1);
}




// The maximum radius to consider around a keypoint that is refined.  That
// is, at most a patch of a maximum size of twice this value in both
// horizontal and vertical direction is extracted.
int RefinementRadiusMaximum = 96;
int (*refineHandler)(int index, int total);


void RefineKeypoints (ArrayList* msList,
		      bool selectMiddlePoint, bool neverLosePoints);
DisplayImage* ExtractPatch (DisplayImage* large,
			    int px, int py, double scale, int* radius);
ArrayList* ExtractKeypoints (DisplayImage* pic);
bool YesNoOption (char* optionName, char* val, int * nerr);

// selectMiddlePoint: if true, select the middle point in the patch,
//   otherwise build the mean
// neverLosePoints: if true, and if we cannot do the refinement, still use
//   the control point.
void RefineKeypoints (ArrayList* msList,
		bool selectMiddlePoint, bool neverLosePoints)
{
	DisplayImage* pic1 = NULL;
	DisplayImage* pic2 = NULL;
	char* pic1Name = NULL;
	char* pic2Name = NULL;

	/* Keep stats for the refineHandler delegate
	 */
	int totalRefines = 0;
	int doneRefines = 0;
	int i;
	for(i=0; i<ArrayList_Count(msList); i++) {
		MatchSet* ms = (MatchSet*) ArrayList_GetItem(msList, i);
		int j;
		for(j=0; j<ArrayList_Count(ms->matches); j++) {
			ArrayList_GetItem(ms->matches, j);
			totalRefines += 1;
		}
	}

 
	for(i=0; i<ArrayList_Count(msList); i++) {
		MatchSet* ms = (MatchSet*) ArrayList_GetItem(msList, i);
		WriteLine ("  between \"%s\" and \"%s\"",
			   ms->file1, ms->file2);
		
		if (pic1Name != ms->file1) {
			pic1Name = ms->file1;
			pic1 = DisplayImage_new (ms->file1);
		}
		if (pic2Name != ms->file2) {
			pic2Name = ms->file2;
			pic2 = DisplayImage_new (ms->file2);
		}
		/*WriteLine ("pair: %s, %s, %d keypoint matches",
		  ms->file1, ms->file2, ArrayList_Count(ms->Matches));*/
		
		ArrayList* refinedMatches = ArrayList_new0 (NULL);

		int j;
		for(j=0; j<ArrayList_Count(ms->matches); j++) {
			Match* m = (Match*) ArrayList_GetItem(ms->matches, j);

			int p1x = (int) (m->kp1->x + 0.5);
			int p1y = (int) (m->kp1->y + 0.5);
			int p1radius;
			DisplayImage* patch1 = ExtractPatch (pic1, p1x, p1y,
							    m->kp1->scale, &p1radius);

			int p2x = (int) (m->kp2->x + 0.5);
			int p2y = (int) (m->kp2->y + 0.5);
			int p2radius;
			DisplayImage* patch2 = ExtractPatch (pic2, p2x, p2y,
							     m->kp2->scale, &p2radius);

			/* Call the refine handler delegate in case there is one to
			 * inform the callee of a single refining step (for progress
			 * bar displays and such).
			 */
			doneRefines += 1;
			if (refineHandler != NULL)
				refineHandler (doneRefines, totalRefines);

			// Skip over keypoint matches we cannot refine as part of the
			// image lies outside.
			if (patch1 == NULL || patch2 == NULL) {
				if (neverLosePoints)
					ArrayList_AddItem(refinedMatches, m);
                                DisplayImage_delete(patch1);
                                DisplayImage_delete(patch2);
				continue;
			}

			// Otherwise, run the SIFT algorithm on both small patches.
			ArrayList* p1kp = ExtractKeypoints (patch1);
			ArrayList* p2kp = ExtractKeypoints (patch2);
			/*WriteLine ("p1kp = %d, p2kp = %d", ArrayList_Count(p1kp),
			  ArrayList_Count(p2kp));*/
			
			// Apply the matching, RANSAC enabled.
			MultiMatch* mm = MultiMatch_new0 ();
			mm->verbose = false;

			ArrayList* matches = NULL;
			matches = MultiMatch_TwoPatchMatch (mm, p1kp,
							    patch1->width, patch1->height, p2kp, patch2->width,
							    patch2->height, true);
                        DisplayImage_delete(patch1);
                        DisplayImage_delete(patch2);

			/* In case there are less than three keypoints in the
			 * two patches, we ignore them all.
			 */
			if (0 /*was exception ???*/ ) { 
				matches = NULL;
			}

			if (matches == NULL || ArrayList_Count(matches) != 1) {
				if (neverLosePoints)
					ArrayList_AddItem(refinedMatches, m);

				continue;
			}

			MatchSet* pSet = (MatchSet*) ArrayList_GetItem(matches, 0);

			// Now get the real new control point coordinates from the
			// patches.  We have two options and assume all points are
			// equal quality-wise:
			//   a) Select the one that is most in the middle
			//      (selectMiddlePoint == true)
			//   b) Build the mean of all the control point matches in the
			//      patches (selectMiddlePoint == false).
			double kp1X = 0.0;
			double kp1Y = 0.0;
			double kp2X = 0.0;
			double kp2Y = 0.0;
			double kpMidDist = Double_PositiveInfinity;

			int k;
			for(k=0; k<ArrayList_Count(pSet->matches); k++) {
				Match* pM = (Match*) ArrayList_GetItem(pSet->matches, k);
				if (selectMiddlePoint) {
					double dist = sqrt (
						pow (pM->kp1->x - p1radius, 2.0) +
						pow (pM->kp1->y - p1radius, 2.0));
					
					if (dist < kpMidDist) {
						kpMidDist = dist;
						
						kp1X = pM->kp1->x;
						kp1Y = pM->kp1->y;
						
						kp2X = pM->kp2->x;
						kp2Y = pM->kp2->y;
					}
				} else {
					kp1X += pM->kp1->x;
					kp1Y += pM->kp1->y;
					
					kp2X += pM->kp2->x;
					kp2Y += pM->kp2->y;
				}

				/*WriteLine ("(%g, %g) matches (%g, %g)",
				  pM->kp1->x, pM->kp1->y, pM->kp2->x, pM->kp2->y);*/
			}

			if (selectMiddlePoint == false) {
				kp1X /= (double) ArrayList_Count(pSet->matches);
				kp1Y /= (double) ArrayList_Count(pSet->matches);
				kp2X /= (double) ArrayList_Count(pSet->matches);
				kp2Y /= (double) ArrayList_Count(pSet->matches);
			}

			kp1X += p1x - p1radius;
			kp1Y += p1y - p1radius;

			kp2X += p2x - p2radius;
			kp2Y += p2y - p2radius;

			Match* mn = Match_clone (m);

			// Adjust the original keypoints location to be the mean of
			// all the highly precise superresolution points.
			mn->kp1->x = kp1X;
			mn->kp1->y = kp1Y;

			mn->kp2->x = kp2X;
			mn->kp2->y = kp2Y;

			/*WriteLine ("MASTER POINT MATCH: (%g,%g) to (%g,%g)",
			  kp1X, kp1Y, kp2X, kp2Y);*/

			ArrayList_AddItem(refinedMatches, mn);
			/*
			  DisplayImage_Save (patch1, "patch-1.jpg");
			  DisplayImage_Save (patch2, "patch-2.jpg");
			  exit (0);
			*/
		}

		ms->matches = refinedMatches;
	}
}

/** Extract a small image patch from a larger image, centered at the given
 * coordinates.
 */
DisplayImage* ExtractPatch (DisplayImage* large,
			    int px, int py, double scale, int* radius)
{
	*radius = (int) (9.0 * scale + 0.5);
	if (*radius > RefinementRadiusMaximum)
		*radius = RefinementRadiusMaximum;

	/*WriteLine ("patch centered at (%d,%d), scale %g, radius = %d",
	  px, py, scale, *radius);*/

	int pxe = px + *radius;
	int pye = py + *radius;
	px -= *radius;
	py -= *radius;

	if (px < 0 || py < 0 || pxe >= large->width || pye >= large->height) {
		/*WriteLine ("   (%d,%d)-(%d,%d) out of (0,0)-(%d,%d)",
		  px, py, pxe, pye, large->width, large->height);*/

		return (NULL);
	} else {
		//WriteLine ("   extracting patch");
	}
	DisplayImage* patch = DisplayImage_Carve (large, px, py, *radius*2, *radius*2);

	return (patch);
}

/** Produce keypoints for a small image patch.
 */
ArrayList* ExtractKeypoints (DisplayImage* pic)
{
	ImageMap* picMap = DisplayImage_ConvertToImageMap (pic);
	
	LoweFeatureDetector* lf = LoweFeatureDetector_new0 ();
	LoweFeatureDetector_SetPrintWarning(false);
	LoweFeatureDetector_SetVerbose(false);
	LoweFeatureDetector_DetectFeatures (lf, picMap);
	
        ArrayList* res = LoweFeatureDetector_GlobalNaturalKeypoints(lf);
        lf->globalNaturalKeypoints = NULL; // Make sure res won't get deleted.
        LoweFeatureDetector_delete(lf);
        return res;
}

// check a commandline truth value
// return true or false; increment *nerr if invalid
bool YesNoOption (char* optionName, char* val, int * nerr)
{
    if (strcmp (val, "1") == 0 || strcmp (val, "on") == 0)
	return true;
    else if (strcmp (val, "0") == 0 || strcmp (val, "off") == 0)
	return false;
    
    WriteLine ("Error '%s' is no valid value for option '%s'.",
		val, optionName);
	(*nerr)++;
    return false;
}

/* mods for APSCpp
  new arg DIlist gets * the image info list (++mode) or == 0 (legacy mode)
  if ++ mode, copies this info to the output 'i' lines

*/
void WritePTOFile (FILE* pto, MultiMatch* mm, 
		   ArrayList* msList, ArrayList * DIlist,
		   BondBall* bb, int generateHorizon, bool integerCoordinates,
		   bool useAbsolutePathnames)
{
	DIinfo * pdi = 0;
	char buffer[262];

	fprintf(pto, "# Hugin project file generated by APSCpp\n\n");
	fprintf(pto, "p f2 w3000 h1500 v360  n\"JPEG q90\"\n");
	fprintf(pto, "m g1 i0\n\n");
	
	int imageIndex = 0;
	HashTable* imageNameTab = HashTable_new0 (NULL, NULL);
	ArrayList* resolutions = ArrayList_new0 ( (void *) Resolution_delete);
	int i;
	for(i=0; i<ArrayList_Count(mm->keySets); i++) {
		KeypointXMLList* kx = (KeypointXMLList*) ArrayList_GetItem(mm->keySets, i);
		HashTable_AddItem(imageNameTab, kx->imageFile, (void*)imageIndex);
		ArrayList_AddItem(resolutions, Resolution_new (kx->xDim, kx->yDim));
		
		char*  imageFile = kx->imageFile;
	// get corresponding DIlist record
		if( DIlist ){
			for( i = 0; i < ArrayList_Count( DIlist ); i++ ){
				pdi = (pDIinfo)ArrayList_GetItem( DIlist, i );
				if( !strcmp( pdi->name, imageFile ) ) break;
			}
			if( i >= ArrayList_Count( DIlist ) ) 
				pdi = 0;	// NOT FOUND!!
		}
		
		// If the resolution was already there, use the first image with
		// the exact same resolution as reference for camera-related
		// values.
		
		int refIdx;
		for (refIdx = 0 ; refIdx < (ArrayList_Count(resolutions) - 1) ; ++refIdx) {
			if (Resolution_CompareTo((Resolution*) ArrayList_GetItem(resolutions, refIdx), kx->xDim, kx->yDim) == 0)
				break;
		}
		if (refIdx == (ArrayList_Count(resolutions) - 1))
			refIdx = -1;
		
		Position* pos = bb == NULL ? NULL :
			(Position*) HashTable_GetItem(bb->positions, imageFile);
		/*
		  if (pos != NULL) {
		  WriteLine ("yaw %g, pitch %g, rotation %g",
		  pos->yaw, pos->pitch, pos->rotation);
		  }*/

		double yaw = 0.0, pitch = 0.0, rotation = 0.0, hfov = 180.0;
		int format = 0;
		if (pos != NULL) {
			yaw = pos->yaw;
			pitch = pos->pitch;
			rotation = pos->rotation;
		} else if( pdi ){
			yaw = pdi->yaw;
			pitch = pdi->pitch;
			rotation = pdi->roll;
			format = pdi->format;
			hfov = pdi->hfov;
		}
		
		if (imageIndex == 0 || refIdx == -1) {
			sprintf(buffer, "i w%d h%d f%d a0 b-0.01 c0 d0 e0 p%g r%g v%g y%g  u10 n\"%s\"",
				kx->xDim, kx->yDim, format, pitch, rotation, hfov, yaw, imageFile);
		} else {
			sprintf(buffer, "i w%d h%d f%d a=%d b=%d c=%d d0 e0 p%g r%g v=%d y%g  u10 n\"%s\"",
				kx->xDim, kx->yDim, format, refIdx, refIdx, refIdx, pitch, rotation, refIdx, yaw, imageFile);
		}
		fprintf( pto, "%s\n", buffer );
////		WriteLine("%s", buffer );
		imageIndex += 1;
	}
	
	fprintf(pto, "\nv p1 r1 y1\n\n");
	
	fprintf(pto, "# automatically generated control points\n");
	int j;
	for(j=0; j<ArrayList_Count(msList); j++) {
		MatchSet* ms = (MatchSet*) ArrayList_GetItem(msList, j);
		int k;
		for(k=0; k<ArrayList_Count(ms->matches); k++) {
			Match* m = (Match*) ArrayList_GetItem(ms->matches, k);
			if (integerCoordinates == false) {
				fprintf(pto, "c n%d N%d x%.6f y%.6f X%.6f Y%.6f t0\n",
					(int)HashTable_GetItem(imageNameTab, ms->file1), (int)HashTable_GetItem(imageNameTab, ms->file2),
					m->kp1->x, m->kp1->y, m->kp2->x, m->kp2->y);
			} else {
				fprintf(pto, "c n%d N%d x%d y%d X%d Y%d t0\n",
					(int)HashTable_GetItem(imageNameTab, ms->file1), (int)HashTable_GetItem(imageNameTab, ms->file2),
					(int) (m->kp1->x + 0.5), (int) (m->kp1->y + 0.5),
					(int) (m->kp2->x + 0.5), (int) (m->kp2->y + 0.5));
			}
		}
	}
	
	// Generate horizon if we should
	if (bb != NULL && generateHorizon > 0) {
		WriteLine ("Creating horizon...");
		
		int kMain = 2;
		int hPoints = generateHorizon;
		int horizonPointsMade = 0;

		bool hasGood = true;
		while (hPoints > 0 && hasGood) {
			hasGood = false;
			int kStep = 2 * kMain;

			int p;
			for (p = 0 ; hPoints > 0 && p < kMain ; ++p) {
				double stepSize = ((double) ArrayList_Count(bb->firstRow)) / ((double) kStep);
				double beginIndex = p * stepSize;
				double endIndex = (((double) ArrayList_Count(bb->firstRow)) / (double) kMain) +
					p * stepSize;

// Round to next integer and check if their image distance
// is larger than 1. If its not, we skip over this useless
// horizon point.
				int bi = (int) (beginIndex + 0.5);
				int ei = (int) (endIndex + 0.5);
				if ((ei - bi) <= 1)
					continue;

				hasGood = true;

				bi %= ArrayList_Count(bb->firstRow);
				ei %= ArrayList_Count(bb->firstRow);
				fprintf(pto, "c n%s N%s x%d y%d X%d Y%d t2\n",
					(char*)HashTable_GetItem(imageNameTab, ArrayList_GetItem(bb->firstRow, bi)),
					(char*)HashTable_GetItem(imageNameTab, ArrayList_GetItem(bb->firstRow, ei)),
					((Resolution*) ArrayList_GetItem(resolutions,bi))->x / 2,
					((Resolution*) ArrayList_GetItem(resolutions,bi))->y / 2,
					((Resolution*) ArrayList_GetItem(resolutions,ei))->x / 2,
					((Resolution*) ArrayList_GetItem(resolutions,ei))->y / 2);

				horizonPointsMade += 1;
				hPoints -= 1;
			}

// Increase density for next generation lines
			kMain *= 2;
		}
		WriteLine ("  made %d horizon lines.\n", horizonPointsMade);
	}

	fprintf(pto, "\n# :-)\n\n");
	
	WriteLine ("\nYou can now load the output file into hugin.");
	if( DIlist == 0 ){
	  WriteLine ("Notice: guessed image format and field-of-view, please check and adjust.");
	} 

        ArrayList_delete(resolutions);
        HashTable_delete(imageNameTab);
}


int main (int argc, char* argv[])
{
	clock_t start,finish;
    double time;

	// list used in ++ mode only -- image info from pto file
	ArrayList * DIlist = 0;	// item = DIinfo struct.
	char *projectfile = 0;
    // "--projection" option sets these for all images
	int globFmt = -1;	// PT format code
	double globFov = 0;	// width in degrees
	int Nerr = 0;	// counts commandline errors

	// downscale option = largest allowed image dimension (0: none)
	int maxdim = 1600;

	// output to stdout flag
	bool streamout = false;

	// Automatic pre-aligning of images
	bool preAlign = false;
	int bottomDefault = -1;
	int generateHorizon = 0;

	// Use ANN kd-tree for keypoint matching
	bool useANN = true;
  
	// Use RANSAC algorithm match filtration.
	bool useRansac = false;
  
	// Use area based weighting for final match selection.
	bool useAreaFiltration = true;
  
	// Truncate match coordinates to integer numbers.
	bool useIntegerCoordinates = false;
  
	// Use the absolute pathname of the image files in the output PTO
	// file.
	bool useAbsolutePathnames = false;
  
	// Use "keep-best" filtration, keep the maxMatches best.
	int maxMatches = 25;	// default
  
	// Refinement options
	bool refine = false;
	bool refineMiddle = true;
	bool keepUnrefinable = true;
  
	int optionCount = 0;
	int optionN = 1;
	int i;

	WriteLine ("\nAPSCpp, enhanced Autopano-sift-c");

	LoweFeatureDetector_SetVerbose(false); // default low chatter
  
	if (argc+1 < 3) { // can't be legal commandline
		Usage ();
		exit (1);
	}

	while (optionN < argc &&
	       strlen(argv[optionN]) >= 2 
		   && argv[optionN][0] == '-'
		   && argv[optionN][1] == '-')
		{
			char* optionStr = argv[optionN];
			int joptN = 1;  // number of args processed

			if (strcmp (optionStr, "--maxdim") == 0) {
				joptN = 2;
				maxdim = atoi(argv[optionN + 1]);
			} else if( strcmp (optionStr, "--projection") == 0) {
				joptN = 2;
				char *p = argv[optionN + 1];
				if(sscanf( p, "%d", &globFmt ) != 1 ){
					WriteLine ("Parameter to projection option invalid. See the usage help.");
					++Nerr;
				}
				p = strchr(p,',');
				if( p ) globFov = atof(p+1);
				else {
					WriteLine ("Parameter to projection option invalid. See the usage help.");
					++Nerr;
				}

			} else if (strcmp (optionStr, "--ransac") == 0) {
				joptN = 2;
				useRansac = YesNoOption ("--ransac", argv[optionN + 1], &Nerr );
			} else if (strcmp (optionStr, "--ANNmatch") == 0) {
				joptN = 2;
				useANN = YesNoOption ("--ANNmatch", argv[optionN + 1], &Nerr );
			} else if (strcmp (optionStr, "--maxmatches") == 0) {
				joptN = 2;
				if (sscanf(argv[optionN + 1], "%d",  &maxMatches) != 1) {
					WriteLine ("Parameter to maxmatches option invalid. See the usage help.");
					++Nerr;
				}
				if (maxMatches < 0) {
					WriteLine ("Maximum number of matches must be positive or zero (unlimited).");
					++Nerr;
				}
			} else if (strcmp (optionStr, "--disable-areafilter") == 0) {
				useAreaFiltration = false;
			} else if (strcmp (optionStr, "--integer-coordinates") == 0) {
				useIntegerCoordinates = true;
			} else if (strcmp (optionStr, "--absolute-pathnames") == 0) {
				joptN = 2;
				useAbsolutePathnames = YesNoOption ("--absolute-pathnames", argv[optionN + 1], &Nerr);
			} else if (strcmp (optionStr, "--align") == 0) {
				preAlign = true;
			} else if (strcmp (optionStr, "--bottom-is-left") == 0) {
				bottomDefault = 0;
			} else if (strcmp (optionStr, "--bottom-is-right") == 0) {
				bottomDefault = 1;
			} else if (strcmp (optionStr, "--generate-horizon") == 0) {
				joptN = 2;
				if (sscanf(argv[optionN + 1], "%d", &generateHorizon) != 1) {
					WriteLine ("Parameter to generate-horizon option invalid. See the usage help.");
					++Nerr;
				}
				if (generateHorizon < 0) {
					WriteLine ("The number of horizon lines to generate must be positive.");
					++Nerr;
				}
			} else if (strcmp (optionStr, "--refine") == 0) {
				refine = true;
			} else if (strcmp (optionStr, "--refine-by-middle") == 0) {
				refineMiddle = true;
			} else if (strcmp (optionStr, "--refine-by-mean") == 0) {
				refineMiddle = false;
			} else if (strcmp (optionStr, "--keep-unrefinable") == 0) {
				joptN = 2;
				keepUnrefinable = YesNoOption ("--keep-unrefinable", argv[optionN + 1], &Nerr);
			} else {
				WriteLine ("Bad option \"%s\"", optionStr);
				++Nerr;
				break;		// let's not take chances....
			}
			optionN += joptN;
		}
	optionCount = optionN;
	// is there an output name and at least one input name?
	if( argc - optionN < 2 ){
		WriteLine ("Error. Output name and at least one more name required.");
		++Nerr;
	}

	if (bottomDefault != -1 && preAlign == false) {
		WriteLine ("Please enable automatic alignment (\"--align\") before using the");
		WriteLine ("--bottom-is-* options.");
		++Nerr;
	}

	if (generateHorizon > 0 && preAlign == false) {
		WriteLine ("Please enable automatic alignment (\"--align\") before using the");
		WriteLine ("--generate-horizon option. Run without arguments for help.");
		++Nerr;
	}

// show usage and quit if commandline errors
	if (Nerr){
		WriteLine ("%d commandline errors.\n", Nerr );
		Usage();
		exit( 1 );
	}

	// next arg is either output file name or "-" for stdout
	// anything else beginning with '-' is an error
	if( argv[optionCount][0] == '-' ){
		if( strcmp( argv[optionCount], "-" ) == 0 )streamout = true;
		else {
			WriteLine ("Misplaced option '%s'.\n", argv[optionCount] );
			Usage();
			exit (1);
		}
	}
  /* next arg is either a script name or the first of 2 or more image file names 
  */
	if( argc - optionCount == 2 ){
		projectfile = argv[optionCount + 1];
		WriteLine("Reading project file %s", projectfile );
	// enable ++ mode by creating DIlist
		DIlist = ArrayList_new0( (void *) DIinfo_delete );
	// read the proect file image info into DIlist, or die
		if( LoadProjectImages( argv[optionCount + 1], DIlist ) ==0 ){
		  WriteLine ("Error reading project file.");
		  exit(1);
		}
	} else if( globFmt >= 0 && globFov > 0 ){
	// enable ++ mode 
		DIlist = ArrayList_new0( (void *) DIinfo_delete );
	}

	if( DIlist ) WriteLine("  Stereographic projection enabled.");



/** Find keypoints **/
	start = clock();

	ArrayList* keylists = ArrayList_new0(NULL);

	if( DIlist ){
	// ++ mode: image info from DIlist
		pDIinfo pd;
	// build it first if only a global projection was given
		if( globFmt >= 0 ){
			for( i=0; i<argc - 1 - optionCount; i++) {
			  pd = DIinfo_new0();
			  pd->format = globFmt;
			  pd->hfov = globFov;
			  strcpy(pd->name, argv[i+optionCount+1] );
			  // note dimensions are still zero
			  ArrayList_AddItem( DIlist, pd );
			}
		}
	// process the images
		for( i = 0; i < ArrayList_Count( DIlist ); i++ ){
		  pd = (pDIinfo) ArrayList_GetItem( DIlist, i);
		  ArrayList_AddItem(keylists, GenerateKeyspp( pd->name, maxdim, pd, useANN ) );
		}
	} else {
	// std mode: image names only from commandline
		for( i=0; i<argc - 1 - optionCount; i++) {
		  ArrayList_AddItem(keylists, GenerateKeyspp( argv[i+optionCount+1], maxdim, 0, useANN ) );
		}
	}

    finish = clock();
    time = ((double)finish - (double)start ) / CLOCKS_PER_SEC;
    WriteLine("%.1f seconds to find keypoints", time);

/** Find control points **/

	start = clock();

	MultiMatch* mm = MultiMatch_new0 ();
	MultiMatch_LoadKeysetsFromMemory (mm, keylists);
	// note mm now owns keylists and will delete it when destroyed

	WriteLine ("\nMatching; ransac %s, ANNmatch %s...", 
		        useRansac == true ? "on" : "off",
		        useANN == true ? "on" : "off"
			  );

	if( useANN ) MultiMatch_ANN ( mm );

	ArrayList* msList = MultiMatch_LocateMatchSets (mm, 3, maxMatches,
							useRansac, useAreaFiltration);

    finish = clock();
    time = ((double)finish - (double)start ) / CLOCKS_PER_SEC;
    WriteLine("\n%.1f seconds to match keypoints", time);


	// Connected component check
	WriteLine ("\nConnected component check...");
	ArrayList* components = MultiMatch_ComponentCheck (mm, msList);
	WriteLine ("Connected component identification resulted in %d component%s:",
		   ArrayList_Count(components), ArrayList_Count(components) > 1 ? "s" : "");

	int compN = 1;
	int j;
	for(j=0; j<ArrayList_Count(components); j++) {
		Component* comp = (Component*) ArrayList_GetItem(components, j);
                char* compstr = Component_ToString(comp);
                WriteLine ("component %d: %s", compN++, compstr);
                free(compstr);
	}

	if (ArrayList_Count(components) > 1) {
		WriteLine ("");
		WriteLine ("Warning: There is one or more components that are not connected through control");
		WriteLine ("         points. An optimization of the resulting PTO will not be possible");
		WriteLine ("         without prior adding of control points between the components listed");
		WriteLine ("         above. Please see the manual page for autopano(1) for details.");
		WriteLine ("");
	} else
		WriteLine ("");

	// BondBall algorithm
	BondBall* bb = NULL;
	if (preAlign) {
		bb = MultiMatch_BuildBondBall (mm, msList, bottomDefault);

		if (bb == NULL) {
			WriteLine ("WARNING: Failed to build bondball as requested. No pre-aligning of images");
			WriteLine ("         takes place.\n");
		}
	}

	if (refine) {
		WriteLine ("Refining keypoints");
		RefineKeypoints (msList, refineMiddle, keepUnrefinable);
	}

	FILE* pto;
	if ( streamout ) {
		pto = stdout;
	} else {
		WriteLine ("Creating output file \"%s\"", argv[optionCount]);
		pto = fopen(argv[optionCount], "w");
	}

	WritePTOFile (pto, mm, msList, DIlist,
		      bb, generateHorizon, useIntegerCoordinates,
		      useAbsolutePathnames);

	ArrayList_delete(components);
	BondBall_delete(bb);
	MultiMatch_delete(mm);
	if ( !streamout )
		fclose(pto);


	return 0;
}


