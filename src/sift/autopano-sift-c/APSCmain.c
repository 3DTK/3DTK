
/* APSCmain.c  06Feb2008 TKSharpless
 Single-executable form of AutoPano-sift-C for Hugin 0.7
 Here, GenerateKeys is a subroutine

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

#include "AutoPanoSift.h"

void Usage ()
{
    WriteLine ("APSC: Generate and Match Keypoints giving Hugin project file\n");
    WriteLine ("  Version %s\n", PACKAGE_VERSION);
	WriteLine ("usage: APSC.exe [options] output.pto image1 image2 [..]\n");
	WriteLine ("Options");
	WriteLine ("  --ransac <on|off|1|0>   Switch RANSAC filtration on or off (default: on)");
	WriteLine ("  --maxmatches <matches>  Use no more than the given number of matches");
	WriteLine ("                          (default: 16, use zero for unlimited)");
						       
	WriteLine ("  --disable-areafilter    Do not use max-area filtration, which is default.");
	WriteLine ("                          See manpage for details.");
	WriteLine ("  --integer-coordinates   Truncate match coordinates to integer numbers.");
	WriteLine ("  --absolute-pathnames <on|off|1|0>   Use the absolute pathname of the image");
	WriteLine ("                          file in the PTO output file. Disabled by default.");
	WriteLine ("  --maxdim <integer>      input images are repeatedly halfed in size until");
	WriteLine ("                          both width and height are below 'maxdim' (default: 800).");
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

// subroutine equivalent to GenerateKeys.c
KeypointXMLList * GenerateKeys( char* imgname, int downRes )
{    
	// 1. load the image file
	DisplayImage* pic = DisplayImage_new(imgname);
	int pW = pic->width;
	int pH = pic->height;
    ImageMap* picMap;
	LoweFeatureDetector* lf;
	KeypointXMLList* kpp;
	double startScale = 1.0;
		
	if (downRes > 0) {
		startScale = DisplayImage_ScaleWithin(pic, downRes);
//		WriteLine ("Scaled picture, starting with scale %0.04f", startScale);
	}
 	picMap = DisplayImage_ConvertToImageMap(pic);
	DisplayImage_delete(pic);

	// 2. find the features
	lf = LoweFeatureDetector_new0();

	if (downRes > 0) {
		LoweFeatureDetector_DetectFeaturesDownscaled (lf, picMap, 0, 1.0 / startScale);
	} else
		LoweFeatureDetector_DetectFeatures (lf, picMap);

  /* build the return value
     We need a new copy of the list held in lf because lf is about to be deleted.
	 The following code adapted from LoweFeatureDetector_GlobalNaturalKeypoints() 
	 does that, and also a needed format conversion (lucky this is not C++!)
 */
	ArrayList * globalNaturalKeypoints = ArrayList_new0 (KeypointN_delete);
	int i;
	for(i=0; i < ArrayList_Count(lf->globalKeypoints); i++) {
		Keypoint* kp = (Keypoint *) ArrayList_GetItem( lf->globalKeypoints, i );
		ArrayList_AddItem ( globalNaturalKeypoints, KeypointN_new( kp ));
	}
    // package result as a KeypointXMLList
	kpp = KeypointXMLList_new ( imgname, pW, pH, globalNaturalKeypoints );

	LoweFeatureDetector_delete(lf);

  // disable the patent warning message (for any subsequent images)
    LoweFeatureDetector_SetPrintWarning(false);	

	return kpp;
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
bool YesNoOption (char* optionName, char* val);

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

bool YesNoOption (char* optionName, char* val)
{
    if (strcmp (val, "1") == 0 || strcmp (val, "on") == 0)
	return (true);
    else if (strcmp (val, "0") == 0 || strcmp (val, "off") == 0)
	return (false);
    
    FatalError ("'%s' is no valid truth value for option '%s'. Please see manpage for help.",
		val, optionName);
    return false;
}

void WritePTOFile (FILE* pto, MultiMatch* mm,
		   ArrayList* msList, BondBall* bb, int generateHorizon, bool integerCoordinates,
		   bool useAbsolutePathnames)
{
	fprintf(pto, "# Hugin project file\n");
	fprintf(pto, "# automatically generated by autopano-sift, available at\n");
	fprintf(pto, "# http://cs.tu-berlin.de/~nowozin/autopano-sift/\n\n");
	fprintf(pto, "p f2 w3000 h1500 v360  n\"JPEG q90\"\n");
	fprintf(pto, "m g1 i0\n\n");
	
	int imageIndex = 0;
	HashTable* imageNameTab = HashTable_new0 (NULL, NULL);
	ArrayList* resolutions = ArrayList_new0 (Resolution_delete);
	int i;
	for(i=0; i<ArrayList_Count(mm->keySets); i++) {
		KeypointXMLList* kx = (KeypointXMLList*) ArrayList_GetItem(mm->keySets, i);
		HashTable_AddItem(imageNameTab, kx->imageFile, (void*)imageIndex);
		ArrayList_AddItem(resolutions, Resolution_new (kx->xDim, kx->yDim));
		
		char*  imageFile = kx->imageFile;
		
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

		double yaw = 0.0, pitch = 0.0, rotation = 0.0;
		if (pos != NULL) {
			yaw = pos->yaw;
			pitch = pos->pitch;
			rotation = pos->rotation;
		}
		
		if (imageIndex == 0 || refIdx == -1) {
			fprintf(pto, "i w%d h%d f0 a0 b-0.01 c0 d0 e0 p%g r%g v180 y%g  u10 n\"%s\"\n",
				kx->xDim, kx->yDim, pitch, rotation, yaw, imageFile);
		} else {
			fprintf(pto, "i w%d h%d f0 a=%d b=%d c=%d d0 e0 p%g r%g v=%d y%g  u10 n\"%s\"\n",
				kx->xDim, kx->yDim, refIdx, refIdx, refIdx, pitch, rotation, refIdx, yaw, imageFile);
		}
		imageIndex += 1;
	}
	
	fprintf(pto, "\nv p1 r1 y1\n\n");
	
	fprintf(pto, "# match list automatically generated\n");
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
	WriteLine ("Notice: You absolutely must adjust the field-of-view value for the images");

        ArrayList_delete(resolutions);
        HashTable_delete(imageNameTab);
}


int main (int argc, char* argv[])
{
	WriteLine ("autopano-sift, Automatic panorama generation program\n");
  
	if (argc+1 < 3) {
		Usage ();
		exit (1);
	}

	// downscale option for GenerteKeys
	int mindim = 800; // default maxdim = 800

	// output to stdout flag
	bool streamout = false;

	// Automatic pre-aligning of images
	bool preAlign = false;
	int bottomDefault = -1;
	int generateHorizon = 0;
  
	// Use RANSAC algorithm match filtration.
	bool useRansac = true;
  
	// Use area based weighting for final match selection.
	bool useAreaFiltration = true;
  
	// Truncate match coordinates to integer numbers.
	bool useIntegerCoordinates = false;
  
	// Use the absolute pathname of the image files in the output PTO
	// file.
	bool useAbsolutePathnames = false;
  
	// Use "keep-best" filtration, keep the maxMatches best.
	int maxMatches = 16;	// default: 16
  
	// Refinement options
	bool refine = false;
	bool refineMiddle = true;
	bool keepUnrefinable = true;
  
	int optionCount = 0;
	int optionN = 1;
	while (optionN < argc &&
	       strlen(argv[optionN]) >= 2 
		   && argv[optionN][0] == '-'
		   && argv[optionN][1] == '-')
		{
			char* optionStr = argv[optionN];

			if (strcmp (optionStr, "--maxdim") == 0) {
				mindim = atoi(argv[optionN + 1]);
				optionN += 2;
			} else if (strcmp (optionStr, "--ransac") == 0) {
				useRansac = YesNoOption ("--ransac", argv[optionN + 1]);
				optionN += 2;
			} else if (strcmp (optionStr, "--maxmatches") == 0) {
				if (sscanf(argv[optionN + 1], "%d",  &maxMatches) != 1) {
					WriteLine ("Parameter to maxmatches option invalid. See the usage help.");
					exit (1);
				}
				if (maxMatches < 0) {
					WriteLine ("Maximum number of matches must be positive or zero (unlimited).");
					exit (1);
				}
				optionN += 2;
			} else if (strcmp (optionStr, "--disable-areafilter") == 0) {
				useAreaFiltration = false;
				optionN += 1;
			} else if (strcmp (optionStr, "--integer-coordinates") == 0) {
				useIntegerCoordinates = true;
				optionN += 1;
			} else if (strcmp (optionStr, "--absolute-pathnames") == 0) {
				useAbsolutePathnames = YesNoOption ("--absolute-pathnames", argv[optionN + 1]);
				optionN += 2;
			} else if (strcmp (optionStr, "--align") == 0) {
				preAlign = true;
				optionN += 1;
			} else if (strcmp (optionStr, "--bottom-is-left") == 0) {
				bottomDefault = 0;
				optionN += 1;
			} else if (strcmp (optionStr, "--bottom-is-right") == 0) {
				bottomDefault = 1;
				optionN += 1;
			} else if (strcmp (optionStr, "--generate-horizon") == 0) {
				if (sscanf(argv[optionN + 1], "%d", &generateHorizon) != 1) {
					WriteLine ("Parameter to generate-horizon option invalid. See the usage help.");
					exit (1);
				}
				if (generateHorizon < 0) {
					WriteLine ("The number of horizon lines to generate must be positive.");
					exit (1);
				}

				optionN += 2;
			} else if (strcmp (optionStr, "--refine") == 0) {
				refine = true;
				optionN += 1;
			} else if (strcmp (optionStr, "--refine-by-middle") == 0) {
				refineMiddle = true;
				optionN += 1;
			} else if (strcmp (optionStr, "--refine-by-mean") == 0) {
				refineMiddle = false;
				optionN += 1;
			} else if (strcmp (optionStr, "--keep-unrefinable") == 0) {
				keepUnrefinable = YesNoOption ("--keep-unrefinable", argv[optionN + 1]);
				optionN += 2;
			} else {
				WriteLine ("Option error. Run without arguments for help.");
				exit (1);
			}
		}
	optionCount = optionN;
	// is there an output name and at least one input name?
	if( argc - optionN < 2 ){
		WriteLine ("Error. Output name and at least one image name required.");
		Usage();
		exit(1);
	}
	// next arg is either output file name or "-" for stdout
	// anything else beginning with '-' is an error
	if( argv[optionCount][0] == '-' ){
		if( strcmp( argv[optionCount], "-" ) == 0 )streamout = true;
		else {
			WriteLine ("Option error. Run without arguments for help.");
			exit (1);
		}
	}
	
	if (bottomDefault != -1 && preAlign == false) {
		WriteLine ("Please enable automatic alignment (\"--align\") before using the");
		WriteLine ("--bottom-is-* options. Run without arguments for help.");

		exit (1);
	}

	if (generateHorizon > 0 && preAlign == false) {
		WriteLine ("Please enable automatic alignment (\"--align\") before using the");
		WriteLine ("--generate-horizon option. Run without arguments for help.");

		exit (1);
	}

	MultiMatch* mm = MultiMatch_new0 ();
//------------
	ArrayList* keylists = ArrayList_new0(NULL);
	int i;
	for( i=0; i<argc - 1 - optionCount; i++) {
		ArrayList_AddItem(keylists, GenerateKeys( argv[i+optionCount+1], mindim ) );
	}
	MultiMatch_LoadKeysetsFromMemory (mm, keylists);
	// note mm now owns keylists and will delete it when destroyed
//-------------

	WriteLine ("\nMatching...%s", useRansac == true ? " RANSAC enabled" : "");
	ArrayList* msList = MultiMatch_LocateMatchSets (mm, 3, maxMatches,
							useRansac, useAreaFiltration);

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

	WritePTOFile (pto, mm, msList, bb, generateHorizon, useIntegerCoordinates,
		      useAbsolutePathnames);

	ArrayList_delete(components);
	BondBall_delete(bb);
	MultiMatch_delete(mm);
	if ( !streamout )
		fclose(pto);
	return 0;
}

