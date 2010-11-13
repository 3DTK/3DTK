
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* LoweDetector.cs
 *
 * Lowe scale-invariant keypoint feature detector (SIFT) interface.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 *
 * Implementation of the SIFT algorithm as specified in this research paper by
 * David Lowe: http://www.cs.ubc.ca/~lowe/papers/ijcv03-abs.html
 *
 * "The University of British Columbia has applied for a patent on the SIFT
 * algorithm in the United States. Commercial applications of this software
 * may require a license from the University of British Columbia."
 * For more information, see the LICENSE file supplied with the distribution.
 */

#include "AutoPanoSift.h"

// Detection parameters, suggested by Lowe's research paper.
// Initial parameters
double octaveSigma = 1.6;

// Sigma for gaussian filter applied to double-scaled input image.
double preprocSigma = 1.5;
double LoweFeatureDetector_SetPreprocSigma( double newsigma ){
	double d = preprocSigma;
	if( newsigma < 0 ) newsigma = 0;
	preprocSigma = newsigma;
	return d;
}

// Once one of the downscaled image's dimension falls below this,
// downscaling is stopped.
int minimumRequiredPixelsize = 32;

// How many DoG levels for each octave.
int scaleSpaceLevels = 3;

bool printWarning = true;
// Disable the patent warning.  Used so far only in autopano's subcalls to
// the SIFT algorithm in the refining step.
void LoweFeatureDetector_SetPrintWarning(bool value) {
	printWarning = value;
}

bool verbose = true;
void LoweFeatureDetector_SetVerbose(bool value) {
	verbose = value;
}

// *** Peak related parameters
// Tweak here to reduce/increase keypoint density

// Minimum absolute DoG value of a pixel to be allowed as minimum/maximum
// peak. This control how much general non-differing areas, such as the
// sky is filtered. Higher value = less peaks, lower value = more peaks.
// Good values from 0.005 to 0.01. Note this is related to
// 'dValueLowThresh', which should be a bit larger, factor 1.0 to 1.5.
double dogThresh = 0.0075;

// D-value filter highcap value, higher = less keypoints, lower = more.
// Lower: only keep keypoints with good localization properties, i.e.
//    those that are precisely and easily to localize (high contrast, see
//    Lowe, page 11. He recommends 0.03, but this seems way too high to
//    me.)
double dValueLowThresh = 0.008;

// Required cornerness ratio level, higher = more keypoints, lower = less.
double maximumEdgeRatio = 20.0;

// The exact sub-pixel localization is done on just one DoG plane. Even
// when the scale adjustment exceeds +/- 0.5, the plane is not changed.
// With this value you can discard peaks that are localized to be too far
// from the plane. A high value will allow for peaks to be used that are
// more far away from the plane used for localization, while a low value
// will sort out more peaks, that drifted too far away.
//
// Be very careful with this value, as a too large value will lead to a
// high number of keypoints in hard to localize areas such as in photos of
// the sky.
//
// Good values seem to lie between 0.30 and 0.6.
double scaleAdjustThresh = 0.50;

// Number of maximum steps a single keypoint can make in its space.
int relocationMaximum = 4;

// Results
ArrayList* LoweFeatureDetector_GlobalKeypoints(LoweFeatureDetector* self) {
	return (self->globalKeypoints);
}

// The Integer-normalized version of the globalKeypoints.
ArrayList* LoweFeatureDetector_GlobalNaturalKeypoints(LoweFeatureDetector* self) {
  	int i;

	if (self->globalNaturalKeypoints != NULL)
		return (self->globalNaturalKeypoints);
	
	if (self->globalKeypoints == NULL)
		WriteError ("No keypoints generated yet.");
	
	self->globalNaturalKeypoints = ArrayList_new0 (KeypointN_delete);

	for(i=0; i<ArrayList_Count(self->globalKeypoints); i++) {
		Keypoint* kp = (Keypoint *) ArrayList_GetItem(self->globalKeypoints, i);
		ArrayList_AddItem (self->globalNaturalKeypoints, KeypointN_new(kp));
	}
	
	return (self->globalNaturalKeypoints);
}

LoweFeatureDetector* LoweFeatureDetector_new0() 
{
	LoweFeatureDetector* self = (LoweFeatureDetector*)malloc(sizeof(LoweFeatureDetector));
	self->globalKeypoints = NULL;
	self->globalNaturalKeypoints = NULL;
	self->pyr = NULL;
	return self;
}

void LoweFeatureDetector_delete(LoweFeatureDetector* self) 
{
	if (self) {
		ArrayList_delete(self->globalKeypoints);
		ArrayList_delete(self->globalNaturalKeypoints);
		OctavePyramid_delete(self->pyr);
		free( self);
	}
}

// Return the number of detected features.
int LoweFeatureDetector_DetectFeatures (LoweFeatureDetector* self, ImageMap* img)
{
	return (LoweFeatureDetector_DetectFeaturesDownscaled (self, img, -1, 1.0));
}

// Scale down the images down so that both dimensions are smaller than
// 'bothDimHi'. If 'bothDimHi' is < 0, the image is doubled before
// processing, if it is zero, nothing is done to the image.
int LoweFeatureDetector_DetectFeaturesDownscaled (LoweFeatureDetector* self, ImageMap* img, int bothDimHi,
						  double startScale)
{
	if (printWarning) {
		// Print license restriction
		WriteLine ("");
		WriteLine ("===============================================================================");
		WriteLine ("The use of this software is restricted by certain conditions.");
		WriteLine ("See the \"LICENSE\" file distributed with the program for details.");
		WriteLine ("");
		WriteLine ("The University of British Columbia has applied for a patent on the SIFT");
		WriteLine ("algorithm in the United States.  Commercial applications of this software may");
		WriteLine ("require a license from the University of British Columbia.");
		WriteLine ("===============================================================================");
		WriteLine ("");
	}

	// Double the image size, as this way more features are detected. The
	// scale is reduced to 0.5.
	if (bothDimHi < 0) {
		ImageMap* tmp = ImageMap_ScaleDouble (img);
		ImageMap_delete(img);
		img = tmp;
		startScale *= 0.5;
	} else if (bothDimHi > 0) {
		while (img->xDim > bothDimHi || img->yDim > bothDimHi) {
			ImageMap* tmp = ImageMap_ScaleHalf (img);
			ImageMap_delete(img);
			img = tmp;
			startScale *= 2.0;
		}
	}


	// XXX: Maybe the blurring has to be before double-sizing?
	// better not, if we would lose more information then?

	// (Lowe03, p10, "We assume that the original image has a blur of at
	// least \sigma = 0.5 ...")
	// So, do one initial image smoothing pass.
	if (preprocSigma > 0.0) {
		ImageMap* tmp = ImageMap_GaussianConvolution (img, preprocSigma);
		ImageMap_delete(img);
		img = tmp;
	}

	self->pyr = OctavePyramid_new0 ();
	self->pyr->Verbose = verbose;
	OctavePyramid_BuildOctaves (self->pyr, img, startScale, scaleSpaceLevels,
			  octaveSigma, minimumRequiredPixelsize);

	self->globalKeypoints = ArrayList_new0 (Keypoint_delete);

	// Generate keypoints from each scalespace.
	int on;
	for (on = 0 ; on < OctavePyramid_Count(self->pyr) ; ++on) {
		DScaleSpace* dsp = OctavePyramid_GetScaleSpace(self->pyr, on);

		ArrayList* peaks = DScaleSpace_FindPeaks (dsp, dogThresh);
		if (verbose)
			WriteLine ("Octave %d has %d raw peaks",
				    on, ArrayList_Count(peaks));

		int oldCount = ArrayList_Count(peaks);
		ArrayList* peaksFilt = DScaleSpace_FilterAndLocalizePeaks (dsp, peaks,
								  maximumEdgeRatio, dValueLowThresh, scaleAdjustThresh,
								  relocationMaximum);

		if (verbose) {
			WriteLine ("  filtered: %d remaining from %d, thats %2.2f%%",
				   ArrayList_Count(peaksFilt), oldCount, (100.0 * ArrayList_Count(peaksFilt)) / oldCount);

			WriteLine ("generating keypoints from peaks");
		}

		// Generate the actual keypoint descriptors, using pre-computed
		// values for the gradient magnitude and direction.
		DScaleSpace_GenerateMagnitudeAndDirectionMaps (dsp);
		ArrayList* keypoints = DScaleSpace_GenerateKeypoints (dsp, peaksFilt,
							     scaleSpaceLevels, octaveSigma);
        // dangelo
		if( verbose ){
        WriteLine ("  %d keypoints generated",
                   ArrayList_Count(keypoints));
		}

		ArrayList_delete(peaksFilt);
		ArrayList_delete(peaks);
		DScaleSpace_ClearMagnitudeAndDirectionMaps (dsp);

		ArrayList_AddRange (self->globalKeypoints, keypoints);
		ArrayList_delete(keypoints);
	}

	return (ArrayList_Count(self->globalKeypoints));
}
