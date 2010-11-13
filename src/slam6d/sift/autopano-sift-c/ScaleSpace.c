
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004-2005 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* ScaleSpace.cs
 *
 * Key feature identification functionality, octave and scale-space handling.
 *
 * (C) Copyright 2004-2005 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 *
 * "The University of British Columbia has applied for a patent on the SIFT
 * algorithm in the United States. Commercial applications of this software
 * may require a license from the University of British Columbia."
 * For more information, see the LICENSE file supplied with the distribution.
 */

#include "AutoPanoSift.h"

OctavePyramid* OctavePyramid_new0 ()
{
	OctavePyramid* self = (OctavePyramid*)malloc(sizeof(OctavePyramid));
	self->octaves = NULL;
	self->Verbose = true;
	return self;
}

void OctavePyramid_delete(OctavePyramid* self)
{
	if (self) {
		ArrayList_delete(self->octaves);
		memset(self, 0, sizeof(OctavePyramid));
		free(self);
	}
}

int OctavePyramid_Count(OctavePyramid* self)
{
	return ArrayList_Count(self->octaves);
}

DScaleSpace* OctavePyramid_GetScaleSpace(OctavePyramid* self, int idx)
{
	return (DScaleSpace*) ArrayList_GetItem(self->octaves, idx);
}

// Build the largest possible number of octaves, each holding
// 'levelsPerOctave' scales in scale-space. Each octave is downscaled by
// 0.5 and the scales in each octave represent a sigma change of
// 'octaveSigma' to 2.0 * 'octaveSigma'. If minSize is greater zero, every
// scale-space with less than minSize effective pixels in x-dimension will
// be discarded.
//
// Return the number of octaves build
int OctavePyramid_BuildOctaves (OctavePyramid* self, 
				ImageMap* source, double scale,
				int levelsPerOctave, double octaveSigma, int minSize)
{
	self->octaves = ArrayList_new0 (DScaleSpace_delete);
	
	DScaleSpace* downSpace = NULL;
	ImageMap* prev = source;
	
	while (prev != NULL && prev->xDim >= minSize && prev->yDim >= minSize) {
		DScaleSpace* dsp = DScaleSpace_new0 ();
		dsp->Verbose = self->Verbose;
		
		if (self->Verbose)
			WriteLine ("Building octave, (%d, %d)", prev->xDim, prev->yDim);
		
		// Create both the gaussian filtered images and the DoG maps
		DScaleSpace_BuildGaussianMaps (dsp, prev, scale, levelsPerOctave, octaveSigma);
		DScaleSpace_BuildDiffMaps (dsp);

		ArrayList_AddItem (self->octaves , dsp);

		prev = ImageMap_ScaleHalf(DScaleSpace_LastGaussianMap(dsp));

		if (downSpace != NULL)
			downSpace->Up = dsp;

		dsp->Down = downSpace;
		downSpace = dsp;
		
		scale *= 2.0;
	}
	if (prev != NULL) {
		ImageMap_delete(prev);
	}
	
	return ArrayList_Count(self->octaves);
}

DScaleSpace* DScaleSpace_new0()
{
	DScaleSpace* self = (DScaleSpace*)malloc(sizeof(DScaleSpace));
	self->Verbose = true;
	self->Down = NULL;
	self->Up = NULL;
	self->baseImg = NULL;
	self->magnitudes = NULL;
	self->directions = NULL;
	self->imgScaled = NULL;
	self->spaces = NULL;
	return self;
}

void DScaleSpace_delete(DScaleSpace* self)
{
	ArrayList_delete(self->magnitudes);
	ArrayList_delete(self->directions);
	ArrayList_delete(self->imgScaled);
	ArrayList_delete(self->spaces);
	free(self);
}

ImageMap* DScaleSpace_GetGaussianMap (DScaleSpace* self, int idx)
{
	return (ImageMap*) ArrayList_GetItem(self->imgScaled, idx);
}

// The last created gaussian map, with 2 \sigma blur. (For use with next
// octave).
ImageMap* DScaleSpace_LastGaussianMap(DScaleSpace* self)
{
	// position s = Length - 2 has: D(k^s * sigma) = D(2 * sigma)
	if (ArrayList_Count(self->imgScaled) < 2)
		FatalError ("bu keneng: too few gaussian maps");
	
	return (ImageMap*) ArrayList_GetItem(self->imgScaled, ArrayList_Count(self->imgScaled) - 2);
}

int DScaleSpace_Count(DScaleSpace* self)
{
	return ArrayList_Count(self->spaces);
}

// Return a single DoG map
ImageMap* DScaleSpace_GetMap(DScaleSpace* self, int idx) 
{
	return (ImageMap*) ArrayList_GetItem(self->spaces, idx);
}

// Generate keypoints from localized peak list.
// TODO: description
// scaleCount: number of scales (3)
ArrayList* DScaleSpace_GenerateKeypoints(DScaleSpace* self,
					 ArrayList* localizedPeaks,
					 int scaleCount, double octaveSigma)
{
	ArrayList* keypoints = ArrayList_new0 (NULL);
	
	int i;
	for (i=0; i<ArrayList_Count(localizedPeaks); i++) {
		ScalePoint* sp = (ScalePoint*) ArrayList_GetItem(localizedPeaks, i);

		// Generate zero or more keypoints from the scale point locations.
		// TODO: make the values configurable
		ArrayList* selfPointKeys = DScaleSpace_GenerateKeypointSingle (self, self->basePixScale,
									       sp, 36, 0.8, scaleCount, octaveSigma);

		// Generate the feature descriptor.
		selfPointKeys = DScaleSpace_CreateDescriptors (self, selfPointKeys,
							       (ImageMap*) ArrayList_GetItem(self->magnitudes, sp->level), 
							       (ImageMap*) ArrayList_GetItem(self->directions, sp->level), 2.0, 4, 8, 0.2);

		// Only copy over those keypoints that have been successfully
		// assigned a descriptor (feature vector).
		int j;
		for(j=0; j<ArrayList_Count(selfPointKeys); j++) {
			Keypoint* kp = (Keypoint*) ArrayList_GetItem(selfPointKeys, j);
			
			if (kp->hasFV == false)
				FatalError ("should not happen");

			// Transform the this level image relative coordinate system
			// to the original image coordinates by multiplying with the
			// current img scale (which starts with either 0.5 or 1.0 and
			// then always doubles: 2.0, 4.0, ..)
			// Note that the kp coordinates are not used for processing by
			// the detection methods and this has to be the last step.
			// Also transform the relative-to-image scale to an
			// absolute-to-original-image scale.
			kp->x *= kp->imgScale;
			kp->y *= kp->imgScale;
			kp->scale *= kp->imgScale;
			
			ArrayList_AddItem (keypoints, kp);
		}
		ArrayList_delete(selfPointKeys);
	}
	
	return (keypoints);
}

// Assign each feature point one or more standardized orientations.
// (section 5 in Lowe's paper)
//
// We use an orientation histogram with 36 bins, 10 degrees each. For
// this, every pixel (x,y) lieing in a circle of 'squareDim' diameter
// within in a 'squareDim' sized field within the image L ('gaussImg') is
// examined and two measures calculated:
//
//    m = \sqrt{ (L_{x+1,y} - L_{x-1,y})^2 + (L_{x,y+1} - L_{x,y-1})^2 }
//    theta = tan^{-1} ( \frac{ L_{x,y+1} - L_{x,y-1} }
//                { L_{x+1,y} - L_{x-1,y} } )
//
// Where m is the gradient magnitude around the pixel and theta is the
// gradient orientation. The 'imgScale' value is the octave scale,
// starting with 1.0 at the finest-detail octave, and doubling every
// octave. The gradient orientations are discreetized to 'binCount'
// directions (should be: 36). For every peak orientation that lies within
// 'peakRelThresh' of the maximum peak value, a keypoint location is
// added (should be: 0.8).
//
// Note that 'space' is the gaussian smoothed original image, not the
// difference-of-gaussian one used for peak-search.
ArrayList* DScaleSpace_GenerateKeypointSingle (DScaleSpace* self,
					       double imgScale, ScalePoint* point,
					       int binCount, double peakRelThresh, int scaleCount,
					       double octaveSigma)
{
	// The relative estimated keypoint scale. The actual absolute keypoint
	// scale to the original image is yielded by the product of imgScale.
	// But as we operate in the current octave, the size relative to the
	// anchoring images is missing the imgScale factor.
	double kpScale = octaveSigma *
		pow (2.0, (point->level + point->local->scaleAdjust) / scaleCount);
	
	// Lowe03, "A gaussian-weighted circular window with a \sigma three
	// times that of the scale of the keypoint".
	//
	// With \sigma = 3.0 * kpScale, the square dimension we have to
	// consider is (3 * \sigma) (until the weight becomes very small).
	double sigma = 3.0 * kpScale;
	int radius = (int) (3.0 * sigma / 2.0 + 0.5);
	int radiusSq = radius * radius;

	ImageMap* magnitude = (ImageMap*) ArrayList_GetItem(self->magnitudes, point->level);
	ImageMap* direction = (ImageMap*) ArrayList_GetItem(self->directions, point->level);

	// As the point may lie near the border, build the rectangle
	// coordinates we can still reach, minus the border pixels, for which
	// we do not have gradient information available.
	int xMin = max (point->x - radius, 1);
	int xMax = min (point->x + radius, magnitude->xDim - 1);
	int yMin = max (point->y - radius, 1);
	int yMax = min (point->y + radius, magnitude->yDim - 1);

	// Precompute 1D gaussian divisor (2 \sigma^2) in:
	// G(r) = e^{-\frac{r^2}{2 \sigma^2}}
	double gaussianSigmaFactor = 2.0 * sigma * sigma;

	double* bins = (double*)calloc(binCount, sizeof(double));

	// Build the direction histogram
	int y;
	for (y = yMin ; y < yMax ; ++y) {
		int x;
		for ( x = xMin ; x < xMax ; ++x) {
			// Only consider pixels in the circle, else we might skew the
			// orientation histogram by considering more pixels into the
			// corner directions
			int relX = x - point->x;
			int relY = y - point->y;
			if (DScaleSpace_IsInCircle (relX, relY, radiusSq) == false)
				continue;

			// The gaussian weight factor.
			double gaussianWeight = exp
				(- ((relX * relX + relY * relY) / gaussianSigmaFactor));

			// find the closest bin and add the direction
			int binIdx = DScaleSpace_FindClosestRotationBin (self, binCount, ImageMap_GetPixel(direction, x, y));
			bins[binIdx] += ImageMap_GetPixel(magnitude, x, y) * gaussianWeight;
		}
	}

	// As there may be succeeding histogram entries like this:
	// ( ..., 0.4, 0.3, 0.4, ... ) where the real peak is located at the
	// middle of this three entries, we can improve the distinctiveness of
	// the bins by applying an averaging pass.
	//
	// TODO: is this really the best method? (we also loose a bit of
	// information. Maybe there is a one-step method that conserves more)
	DScaleSpace_AverageWeakBins (self, bins, binCount);

	// find the maximum peak in gradient orientation
	double maxGrad = 0.0;
	int maxBin = 0;
	int b;
	for (b = 0 ; b < binCount ; ++b) {
		if (bins[b] > maxGrad) {
			maxGrad = bins[b];
			maxBin = b;
		}
	}

	// First determine the real interpolated peak high at the maximum bin
	// position, which is guaranteed to be an absolute peak.
	//
	// XXX: should we use the estimated peak value as reference for the
	//   0.8 check or the original bin-value?
	double maxPeakValue, maxDegreeCorrection;
	DScaleSpace_InterpolateOrientation (self,
					    bins[maxBin == 0 ? (binCount - 1) : (maxBin - 1)],
					    bins[maxBin], bins[(maxBin + 1) % binCount],
					    &maxDegreeCorrection, &maxPeakValue);

	// Now that we know the maximum peak value, we can find other keypoint
	// orientations, which have to fulfill two criterias:
	//
	//  1. They must be a local peak themselves. Else we might add a very
	//     similar keypoint orientation twice (imagine for example the
	//     values: 0.4 1.0 0.8, if 1.0 is maximum peak, 0.8 is still added
	//     with the default threshhold, but the maximum peak orientation
	//     was already added).
	//  2. They must have at least peakRelThresh times the maximum peak
	//     value.
	bool* binIsKeypoint = (bool*)malloc(binCount*sizeof(bool));
	for (b = 0 ; b < binCount ; ++b) {
		binIsKeypoint[b] = false;
			
		// The maximum peak of course is
		if (b == maxBin) {
			binIsKeypoint[b] = true;
			continue;
		}
			
		// Local peaks are, too, in case they fulfill the threshhold
		if (bins[b] < (peakRelThresh * maxPeakValue))
			continue;

		int leftI = (b == 0) ? (binCount - 1) : (b - 1);
		int rightI = (b + 1) % binCount;
		if (bins[b] <= bins[leftI] || bins[b] <= bins[rightI])
			continue;	// no local peak

		binIsKeypoint[b] = true;
	}

	// All the valid keypoint bins are now marked in binIsKeypoint, now
	// build them.
	ArrayList* keypoints = ArrayList_new0 (NULL);

	// find other possible locations
	double oneBinRad = (2.0 * M_PI) / binCount;

	for (b = 0 ; b < binCount ; ++b) {
		if (binIsKeypoint[b] == false)
			continue;

		int bLeft = (b == 0) ? (binCount - 1) : (b - 1);
		int bRight = (b + 1) % binCount;

		// Get an interpolated peak direction and value guess.
		double peakValue;
		double degreeCorrection;

		if (DScaleSpace_InterpolateOrientation (self, bins[bLeft], bins[b], bins[bRight],
							&degreeCorrection, &peakValue) == false)
		{
			FatalError("BUG: Parabola fitting broken");
		}

		// [-1.0 ; 1.0] -> [0 ; binrange], and add the fixed absolute bin
		// position.
		// We subtract PI because bin 0 refers to 0, binCount-1 bin refers
		// to a bin just below 2PI, so -> [-PI ; PI]. Note that at this
		// point we determine the canonical descriptor anchor angle. It
		// does not matter where we set it relative to the peak degree,
		// but it has to be constant. Also, if the output of this
		// implementation is to be matched with other implementations it
		// must be the same constant angle (here: -PI).
		double degree = (b + degreeCorrection) * oneBinRad - M_PI;

		if (degree < -M_PI)
			degree += 2.0 * M_PI;
		else if (degree > M_PI)
			degree -= 2.0 * M_PI;

		Keypoint* kp = Keypoint_new ((ImageMap*)ArrayList_GetItem(self->imgScaled, point->level),
					     point->x + point->local->fineX,
					     point->y + point->local->fineY,
					     imgScale, kpScale, degree);
		ArrayList_AddItem (keypoints, kp);
	}

	free(binIsKeypoint);
	free(bins);
	return (keypoints);
}

// Fit a parabol to the three points (-1.0 ; left), (0.0 ; middle) and
// (1.0 ; right).
//
// Formulas:
// f(x) = a (x - c)^2 + b
//
// c is the peak offset (where f'(x) is zero), b is the peak value.
//
// In case there is an error false is returned, otherwise a correction
// value between [-1 ; 1] is returned in 'degreeCorrection', where -1
// means the peak is located completely at the left vector, and -0.5 just
// in the middle between left and middle and > 0 to the right side. In
// 'peakValue' the maximum estimated peak value is stored.
bool DScaleSpace_InterpolateOrientation (DScaleSpace* self,
					 double left, double middle,
					 double right, double* degreeCorrection, double* peakValue)
{
	double a = ((left + right) - 2.0 * middle) / 2.0;
	*degreeCorrection = *peakValue = -1;
	
	// Not a parabol
       if (a == 0.0) {
               *degreeCorrection = 0;
               return (true);
       }
	
	double c = (((left - middle) / a) - 1.0) / 2.0;
	double b = middle - c * c * a;
	
	if (c < -0.5 || c > 0.5)
		FatalError
		       ("InterpolateOrientation: off peak ]-0.5 ; 0.5[");
	
	*degreeCorrection = c;
	*peakValue = b;
	
	return (true);
}

// Find the bin out of 'binCount' bins that matches the 'angle' closest.
// 'angle' fulfills -PI <= angle <= PI. Bin 0 is assigned to -PI, the
// binCount-1 bin refers to just below PI.
//
// Return the index of the closest bin.
int DScaleSpace_FindClosestRotationBin (DScaleSpace* self, int binCount, double angle)
{
	angle += M_PI;
	angle /= 2.0 * M_PI;
	
	// calculate the aligned bin
	angle *= binCount;
	
	int idx = (int) angle;
	if (idx == binCount)
		idx = 0;

	return (idx);
}

// Average the content of the direction bins.
void DScaleSpace_AverageWeakBins (DScaleSpace* self, double* bins, int binCount)
{
	// TODO: make some tests what number of passes is the best. (its clear
	// one is not enough, as we may have something like
	// ( 0.4, 0.4, 0.3, 0.4, 0.4 ))
	int sn;
	for ( sn = 0 ; sn < 4 ; ++sn) {
		double firstE = bins[0];
		double last = bins[binCount - 1];
		
		int sw;
		for ( sw = 0 ; sw < binCount ; ++sw) {
			double cur = bins[sw];
			double next = (sw == (binCount - 1)) ?
				firstE : bins[(sw + 1) % binCount];
			
			bins[sw] = (last + cur + next) / 3.0;
			last = cur;
		}
	}
}

// Create the descriptor vector for a list of keypoints.
//
// keypoints: The list of keypoints to be processed. Everything but the
//     descriptor must be filled in already.
// magnitude/direction: The precomputed gradient magnitude and direction
//     maps.
// considerScaleFactor: The downscale factor, which describes the amount
//     of pixels in the circular region relative to the keypoint scale.
//     Low values means few pixels considered, large values extend the
//     range. (Use values between 1.0 and 6.0)
// descDim: The dimension size of the output descriptor. There will be
//     descDim * descDim * directionCount elements in the feature vector.
// directionCount: The dimensionality of the low level gradient vectors.
// fvGradHicap: The feature vector gradient length hi-cap threshhold.
//     (Should be: 0.2)
//
// Some parts modelled after Alexandre Jenny's Matlab implementation.
//
// Return a list of survivors, which a descriptor was created for
// successfully.
ArrayList* DScaleSpace_CreateDescriptors (DScaleSpace* self,
					  ArrayList* keypoints,
					  ImageMap* magnitude, ImageMap* direction,
					  double considerScaleFactor, int descDim, int directionCount,
					  double fvGradHicap)
{
	if (ArrayList_Count(keypoints) <= 0)
		return (keypoints);
	
	considerScaleFactor *= ((Keypoint*) ArrayList_GetItem(keypoints, 0))->scale;
	double dDim05 = ((double) descDim) / 2.0;
	
	// Now calculate the radius: We consider pixels in a square with
	// dimension 'descDim' plus 0.5 in each direction. As the feature
	// vector elements at the diagonal borders are most distant from the
	// center pixel we have scale up with sqrt(2).
	int radius = (int) (((descDim + 1.0) / 2) *
			    sqrt (2.0) * considerScaleFactor + 0.5);
	
	// Instead of modifying the original list, we just copy the keypoints
	// that received a descriptor.
	ArrayList* survivors = ArrayList_new0 (NULL);
	
	// Precompute the sigma for the "center-most, border-less" gaussian
	// weighting.
	// (We are operating to dDim05, CV book tells us G(x), x > 3 \sigma
	//  negligible, but this range seems much shorter!?)
	//
	// In Lowe03, page 15 it says "A Gaussian weighting function with
	// \sigma equal to one half the width of the descriptor window is
	// used", so we just use his advice.
	double sigma2Sq = 2.0 * dDim05 * dDim05;
	
	int i;
	for (i=0; i<ArrayList_Count(keypoints); i++) {
		Keypoint* kp = (Keypoint*) ArrayList_GetItem(keypoints,i);
		// The angle to rotate with: negate the orientation.
		double angle = -kp->orientation;
		
		Keypoint_CreateVector (kp, descDim, descDim, directionCount);
		//Console.WriteLine ("  FV allocated");
		
		int y;
		for (y = -radius ; y < radius ; ++y) {
			int x;
			for (x = -radius ; x < radius ; ++x) {
				// Rotate and scale
				double yR = sin (angle) * x +
					cos (angle) * y;
				double xR = cos (angle) * x -
					sin (angle) * y;
				
				yR /= considerScaleFactor;
				xR /= considerScaleFactor;
				
				// Now consider all (xR, yR) that are anchored within
				// (- descDim/2 - 0.5 ; -descDim/2 - 0.5) to
				//    (descDim/2 + 0.5 ; descDim/2 + 0.5),
				// as only those can influence the FV.
				if (yR >= (dDim05 + 0.5) || xR >= (dDim05 + 0.5) ||
				    xR <= -(dDim05 + 0.5) || yR <= -(dDim05 + 0.5))
					continue;
				
				int currentX = (int) (x + kp->x + 0.5);
				int currentY = (int) (y + kp->y + 0.5);
				if (currentX < 1 || currentX >= (magnitude->xDim - 1) ||
				    currentY < 1 || currentY >= (magnitude->yDim - 1))
					continue;
				
				/*
				  WriteLine ("    (%d,%d) by angle {%lf} -> (%lf,%lf)",
				  x, y, angle, xR, yR);
				  */
				
				// Weight the magnitude relative to the center of the
				// whole FV. We do not need a normalizing factor now, as
				// we normalize the whole FV later anyway (see below).
				// xR, yR are each in -(dDim05 + 0.5) to (dDim05 + 0.5)
				// range
				double magW = exp (-(xR * xR + yR * yR) / sigma2Sq) *
					ImageMap_GetPixel(magnitude, currentX, currentY);
				
				// Anchor to (-1.0, -1.0)-(dDim + 1.0, dDim + 1.0), where
				// the FV points are located at (x, y)
				yR += dDim05 - 0.5;
				xR += dDim05 - 0.5;
				
				// Build linear interpolation weights:
				// A B
				// C D
				//
				// The keypoint is located between A, B, C and D.
				int xIdx[2] = {0,0};
				int yIdx[2] = {0,0};
				int dirIdx[2] ={0,0};
				double xWeight[2] = {0.0,0.0};
				double yWeight[2] = {0.0,0.0};
				double dirWeight[2] = {0.0,0.0};

				if (xR >= 0) {
					xIdx[0] = (int) xR;
					xWeight[0] = (1.0 - (xR - xIdx[0]));
				}
				if (yR >= 0) {
					yIdx[0] = (int) yR;
					yWeight[0] = (1.0 - (yR - yIdx[0]));
				}
				
				if (xR < (descDim - 1)) {
					xIdx[1] = (int) (xR + 1.0);
					xWeight[1] = xR - xIdx[1] + 1.0;
				}
				if (yR < (descDim - 1)) {
					yIdx[1] = (int) (yR + 1.0);
					yWeight[1] = yR - yIdx[1] + 1.0;
				}
				
				// Rotate the gradient direction by the keypoint
				// orientation, then normalize to [-pi ; pi] range.
				double dir = ImageMap_GetPixel(direction, currentX, currentY) - kp->orientation;
				if (dir <= -M_PI)
					dir += M_PI;
				if (dir > M_PI)
					dir -= M_PI;
				
				double idxDir = (dir * directionCount) /
					(2.0 * M_PI);
				if (idxDir < 0.0)
					idxDir += directionCount;
				
				dirIdx[0] = (int) idxDir;
				dirIdx[1] = (dirIdx[0] + 1) % directionCount;
				dirWeight[0] = 1.0 - (idxDir - dirIdx[0]);
				dirWeight[1] = idxDir - dirIdx[0];

				/*
				  WriteLine ("    (%lf,%lf) yields:", xR, yR);
				  WriteLine ("      x<%d,%d>*(%lf,%lf)",
				  xIdx[0], xIdx[1], xWeight[0], xWeight[1]);
				  WriteLine ("      y<%d,%d>*(%lf,%lf)",
				  yIdx[0], yIdx[1], yWeight[0], yWeight[1]);
				  WriteLine ("      dir<%d,%d>*(%lf,%lf)",
				  dirIdx[0], dirIdx[1], dirWeight[0], dirWeight[1]);
				  WriteLine ("    weighting m * w: %lf * %lf",
				  ImageMap_GetPixel(magnitude,currentX, currentY), exp (-(xR * xR +
				  yR * yR) / sigma2Sq));
				  */
				int iy;
				for (iy = 0 ; iy < 2 ; ++iy) {
					int ix;
					for (ix = 0 ; ix < 2 ; ++ix) {
						int id;
						for (id = 0 ; id < 2 ; ++id) {
							Keypoint_FVSet (kp, xIdx[ix], yIdx[iy], dirIdx[id],
									Keypoint_FVGet (kp, xIdx[ix], yIdx[iy], dirIdx[id]) +
									xWeight[ix] * yWeight[iy] * dirWeight[id] * magW);
						}
					}
				}
			}
		}
		
		// Normalize and hicap the feature vector, as recommended on page
		// 16 in Lowe03.
		DScaleSpace_CapAndNormalizeFV (self, kp, fvGradHicap);
		
		ArrayList_AddItem (survivors, kp);
	}

	ArrayList_delete(keypoints);
	return (survivors);
}


// Threshhold and normalize feature vector.
// Note that the feature vector as a whole is normalized (Lowe's paper is
// a bit unclear at that point).
void DScaleSpace_CapAndNormalizeFV (DScaleSpace* self, Keypoint* kp, double fvGradHicap)
{
	// Straight normalization
	double norm = 0.0;
	int n;
	for (n = 0 ; n < Keypoint_FVLinearDim(kp) ; ++n)
		norm += pow (Keypoint_FVLinearGet (kp, n), 2.0);
	
	norm = sqrt (norm);
	if (norm == 0.0)
               return;
	
	for (n = 0 ; n < Keypoint_FVLinearDim(kp) ; ++n)
		Keypoint_FVLinearSet (kp, n, Keypoint_FVLinearGet (kp, n) / norm);
	
	// Hicap after normalization
	for (n = 0 ; n < Keypoint_FVLinearDim(kp) ; ++n) {
		if (Keypoint_FVLinearGet (kp, n) > fvGradHicap) {
			Keypoint_FVLinearSet (kp, n, fvGradHicap);
		}
	}
	
	// Renormalize again
	norm = 0.0;
	for ( n = 0 ; n < Keypoint_FVLinearDim(kp) ; ++n)
		norm += pow (Keypoint_FVLinearGet (kp, n), 2.0);
	
	norm = sqrt (norm);

	for ( n = 0 ; n < Keypoint_FVLinearDim(kp) ; ++n)
		Keypoint_FVLinearSet (kp, n, Keypoint_FVLinearGet (kp, n) / norm);
}

// Simple helper predicate to tell if (rX, rY) is within a circle of
// \sqrt{radiusSq} radius, assuming the circle center is (0, 0).
bool DScaleSpace_IsInCircle (int rX, int rY, int radiusSq)
{
	rX *= rX;
	rY *= rY;
	if ((rX + rY) <= radiusSq)
		return (true);
	
	return (false);
}

// Remove peaks by peak magnitude and peak edge response. Find the
// sub-pixel local offset by interpolation.
//
// Sub-pixel localization and peak magnitude:
// After this method returns, every peak has a relative localization
// offset and its peak magnitude available in 'peak.Local'. The peak
// magnitude value must be above 'dValueLoThresh' for the point to
// survive. Usual values might lie in the range 0.0 (no filtering) to
// 0.03 (Lowe/Brown's recommendation). We normally use a value around
// 0.0001 to 0.00025 (and Brown's values seem quite large to me). The
// scaleAdjustThresh value is explained in LoweDetector.cs.
//
// Edge filtering:
// 'edgeRatio' denotes the required hi-threshhold for the ratio between
// the principle curvatures. Small values (1.5 to 3.0) will filter most
// points, leaving only the most corner-like points. Larger values (3.0 to
// 10.0) will remove the points which lie on a straight edge, whose
// position might be more vulnerable to noise.
//
// Return a filtered list of ScalePoint elements, with only the remaining
// survivors.
ArrayList* DScaleSpace_FilterAndLocalizePeaks (DScaleSpace* self, ArrayList* peaks, double edgeRatio,
		double dValueLoThresh, double scaleAdjustThresh, int relocationMaximum)
{
	ArrayList* filtered = ArrayList_new0 (NULL);
	
	ImageMap* space0 = (ImageMap*) ArrayList_GetItem(self->spaces, 0);
	int** processed = IntMap_new(space0->xDim, space0->yDim);
	
	int i;
	for(i=0; i<ArrayList_Count(peaks); i++) {
		ScalePoint* peak = (ScalePoint*)ArrayList_GetItem(peaks, i);
		
		if (DScaleSpace_IsTooEdgelike (self, (ImageMap*) ArrayList_GetItem(self->spaces, peak->level), peak->x, peak->y, edgeRatio))
			continue;
		
		// When the localization hits some problem, i.e. while moving the
		// point a border is reached, then skip this point.
		if (DScaleSpace_LocalizeIsWeak (self, peak, relocationMaximum, processed))
			continue;
		
		// Now we approximated the exact sub-pixel peak position.
		// Comment the following line out to get a number of image files
		// which show the located peak in the closest DoG scale.
		/*DEBUGSaveRectangle (spaces[peak.Level], peak.X, peak.Y,
		  String.Format ("rect_{0}.png", peak.Local.DValue);
		*/
		
		/*WriteLine ("peak.Local.ScaleAdjust = %f",
		  peak->local->scaleAdjust);*/
		if (abs (peak->local->scaleAdjust) > scaleAdjustThresh)
			continue;
		
		// Additional local pixel information is now available, threshhold
		// the D(^x)
		/*WriteLine ("%d %d %f # == DVALUE", peak->y, peak->x, peak->local->dValue);*/
		if (abs (peak->local->dValue) <= dValueLoThresh)
			continue;
		
		/*WriteLine ("%d %d %f %f # FILTERLOCALIZE",
		  peak->y, peak->x, peak->local->scaleAdjust, peak->local->dValue);*/
		
		// its edgy enough, add it
		ArrayList_AddItem (filtered, peak);
	}

	IntMap_delete(processed);
	return (filtered);
}

// Return true if the point is not suitable, either because it lies on a
// border pixel or the Hessian matrix cannot be inverted.
// If false is returned, the pixel is suitable for localization and
// additional localization information has been put into 'point.Local'.
// No more than 'steps' corrections are made.
bool DScaleSpace_LocalizeIsWeak (DScaleSpace* self, ScalePoint* point, int steps, int** processed)
{
	bool needToAdjust = true;
	int adjusted = steps;
	
	while (needToAdjust) {
		int x = point->x;
		int y = point->y;
		
		// Points we cannot say anything about, as they lie on the border
		// of the scale space
		if (point->level <= 0 || point->level >= (ArrayList_Count(self->spaces) - 1))
			return (true);
		
		ImageMap* space = (ImageMap*) ArrayList_GetItem(self->spaces,point->level);
		if (x <= 0 || x >= (space->xDim - 1))
			return (true);
		if (y <= 0 || y >= (space->yDim - 1))
			return (true);

		double dp;
		SimpleMatrix* adj = DScaleSpace_GetAdjustment (self, point, point->level, x, y, &dp);
		
		// Get adjustments and check if we require further adjustments due
		// to pixel level moves. If so, turn the adjustments into real
		// changes and continue the loop. Do not adjust the plane, as we
		// are usually quite low on planes in thie space and could not do
		// further adjustments from the top/bottom planes.
		double adjS = SimpleMatrix_GetValue(adj, 0, 0);
		double adjY = SimpleMatrix_GetValue(adj, 1, 0);
		double adjX = SimpleMatrix_GetValue(adj, 2, 0);
		SimpleMatrix_delete(adj);
		if (abs (adjX) > 0.5 || abs (adjY) > 0.5) {
			// Already adjusted the last time, give up
			if (adjusted == 0) {
				//WriteLine ("too many adjustments, returning");
				return (true);
			}
			
			adjusted -= 1;
			
			// Check that just one pixel step is needed, otherwise discard
			// the point
			double distSq = adjX * adjX + adjY * adjY;
			if (distSq > 2.0)
				return (true);
			
			point->x = (int) (point->x + adjX + 0.5);
			point->y = (int) (point->y + adjY + 0.5);
			//point->level = (int) (point->level + adjS + 0.5);
			
			/*WriteLine ("moved point by ({0},{1}: {2}) to ({3},{4}: {5})",
			  adjX, adjY, adjS, point.X, point.Y, point.Level);*/
			continue;
		}
		
		/* for processing with gnuplot
		 *
		 Console.WriteLine ("{0} {1} # POINT LEVEL {2}", point.X,
		 point.Y, basePixScale);
		 Console.WriteLine ("{0} {1} {2} # ADJ POINT LEVEL {3}",
		 adjS, adjX, adjY, basePixScale);
		*/
		
		// Check if we already have a keypoint within this octave for this
		// pixel position in order to avoid dupes. (Maybe we can move this
		// check earlier after any adjustment, so we catch dupes earlier).
		// If its not in there, mark it for later searches.
		//
		// FIXME: check why there does not seem to be a dupe at all
		if (processed[point->x][point->y] != 0)
			return (true);
		
		processed[point->x][point->y] = 1;
		
		// Save final sub-pixel adjustments.
		PointLocalInformation* local = PointLocalInformation_new3 (adjS, adjX, adjY);
		//local.DValue = dp;
		local->dValue = ImageMap_GetPixel(space, point->x, point->y) + 0.5 * dp;
		point->local = local;
		
		needToAdjust = false;
	}
	
	return (false);
}

bool DScaleSpace_IsTooEdgelike (DScaleSpace* self, ImageMap* space, int x, int y, double r)
{
	double D_xx, D_yy, D_xy;
	
	// Calculate the Hessian H elements [ D_xx, D_xy ; D_xy , D_yy ]
	D_xx = space->values[x + 1][y] + space->values[x - 1][y] - 2.0 * space->values[x][y];
	D_yy = space->values[x][y + 1] + space->values[x][y - 1] - 2.0 * space->values[x][y];
	D_xy = 0.25 * ((space->values[x + 1][y + 1] - space->values[x + 1][y - 1]) -
		       (space->values[x - 1][y + 1] - space->values[x - 1][y - 1]));
	
	// page 13 in Lowe's paper
	double TrHsq = D_xx + D_yy;
	TrHsq *= TrHsq;
	double DetH = D_xx * D_yy - (D_xy * D_xy);
	
	double r1sq = (r + 1.0);
	r1sq *= r1sq;
	
	// BUG: this can invert < to >, uhh: if ((TrHsq * r) < (DetH * r1sq))
	if ((TrHsq / DetH) < (r1sq / r)) {
		/*Console.WriteLine ("{0} {1} {2} {3} {4} # EDGETEST",
		  y, x, (TrHsq * r), (DetH * r1sq),
		  (TrHsq / DetH) / (r1sq / r));*/
		return (false);
	}
	
	return (true);
}

// Return adjustment (scale, y, x) on success,
// return null on failure
// TODO: integrate this
SimpleMatrix* DScaleSpace_GetAdjustment (DScaleSpace* self, ScalePoint* point,
					 int level, int x, int y, double* dp)
{
    /*WriteLine ("GetAdjustment (point, %d, %d, %d, out double dp)",
      level, x, y);*/
	*dp = 0.0;
	if (point->level <= 0 || point->level >= (ArrayList_Count(self->spaces) - 1))
		FatalError ("point.Level is not within [bottom-1;top-1] range");
	
	ImageMap* below = (ImageMap*) ArrayList_GetItem(self->spaces, level - 1);
	ImageMap* current = (ImageMap*) ArrayList_GetItem(self->spaces, level);
	ImageMap* above = (ImageMap*) ArrayList_GetItem(self->spaces, level + 1);
	
	SimpleMatrix* H = SimpleMatrix_new (3, 3);
	H->values[0][0] = below->values[x][y] - 2 * current->values[x][y] + above->values[x][y];
	H->values[0][1] = H->values[1][0] = 0.25 * (above->values[x][y + 1] - above->values[x][y - 1] -
						    (below->values[x][y + 1] - below->values[x][y - 1]));
	H->values[0][2] = H->values[2][0] = 0.25 * (above->values[x + 1][y] - above->values[x - 1][y] -
						    (below->values[x + 1][y] - below->values[x - 1][y]));
	H->values[1][1] = current->values[x][y - 1] - 2 * current->values[x][y] + current->values[x][y + 1];
	H->values[1][2] = H->values[2][1] = 0.25 * (current->values[x + 1][y + 1] - current->values[x - 1][y + 1] -
						    (current->values[x + 1][y - 1] - current->values[x - 1][y - 1]));
	H->values[2][2] = current->values[x - 1][y] - 2 * current->values[x][y] + current->values[x + 1][y];
	
	SimpleMatrix* d = SimpleMatrix_new (3, 1);
	d->values[0][0] = 0.5 * (above->values[x][y] - below->values[x][y]);
	d->values[1][0] = 0.5 * (current->values[x][y + 1] - current->values[x][y - 1]);
	d->values[2][0] = 0.5 * (current->values[x + 1][y] - current->values[x - 1][y]);
	
	SimpleMatrix* b = SimpleMatrix_clone (d);
	SimpleMatrix_Negate (b);
	
	// Solve: A x = b
	SimpleMatrix_SolveLinear (H, b);
	
	SimpleMatrix_delete(H);
	*dp = SimpleMatrix_Dot (b, d);

	SimpleMatrix_delete(d);
	
	return (b);
}


// Peak localization in scale-space.
//
// From Lowe's paper its not really clear whether we always need three
// neighbourhood spaces or should also search only one or two spaces. As
// figure 3 might suggest the later, we do it like this.
//
// Return an arraylist holding ScalePoint elements.
ArrayList* DScaleSpace_FindPeaks (DScaleSpace* self, double dogThresh)
{
	if (self->Verbose)
		WriteLine ("  FindPeaks: scale %02.2f, testing %d levels",
			   self->basePixScale, DScaleSpace_Count(self) - 2);
	
	ArrayList* peaks = ArrayList_new0 (ScalePoint_delete);

	ImageMap *current, *above, *below;

	// Search the D(k * sigma) to D(2 * sigma) spaces
	int level;
	for (level = 1 ; level < (DScaleSpace_Count(self) - 1) ; ++level)
	{
		current = DScaleSpace_GetMap(self, level);
		below = DScaleSpace_GetMap(self, level - 1);
		above = DScaleSpace_GetMap(self, level + 1);

		//// WriteLine ("peak-search at level %d", level);
		/*
		  WriteLine ("below/current/above: %s %s %s",
		  below == NULL ? "-" : "X",
		  current == NULL ? "-" : "X",
		  above == NULL ? "-" : "X");
		  WriteLine ("peak-search at level %d", level);
		*/
		ArrayList* newPeaks;
		ArrayList_AddRange (peaks, newPeaks = DScaleSpace_FindPeaksThreeLevel (self, below, current, above,
										       level, dogThresh));
		ArrayList_delete(newPeaks);

		below = current;
	}
		
	return (peaks);
}

ArrayList* DScaleSpace_FindPeaksThreeLevel (DScaleSpace* self, ImageMap* below, ImageMap* current,
					    ImageMap* above, int curLev, double dogThresh)
{
	ArrayList* peaks = ArrayList_new0 (NULL);

	int x, y;
	for ( y = 1 ; y < (current->yDim - 1) ; ++y) {
		for ( x = 1 ; x < (current->xDim - 1) ; ++x) {
			bool cIsMax = true;
			bool cIsMin = true;
			
			double c = current->values[x][y];	// Center value
			
			/* If the magnitude is below the threshhold, skip it early.
			 */
			if (abs (c) <= dogThresh)
				continue;
				
			DScaleSpace_CheckMinMax (self, current, c, x, y, &cIsMin, &cIsMax, true);
			DScaleSpace_CheckMinMax (self, below, c, x, y, &cIsMin, &cIsMax, false);
			DScaleSpace_CheckMinMax (self, above, c, x, y, &cIsMin, &cIsMax, false);
			if (cIsMin == false && cIsMax == false)
				continue;
				
			//WriteLine ("%d %d %f # DOG", y, x, c);
				
			/* Add the peak that survived the first checks, to the peak
			 * list.
			 */
			ArrayList_AddItem (peaks,  ScalePoint_new3 (x, y, curLev));
		}
	}
	
	return (peaks);
}

// Check if a pixel ('x', 'y') with value 'c' is minimum or maximum in the
// 'layer' image map. Except for the center, and its above/below planes
// corresponding pixels, use a strong > and < check (because the if is
// inverted it looks like >= and <=).
void DScaleSpace_CheckMinMax (DScaleSpace* self, ImageMap* layer, double c, int x, int y,
			      bool* IsMin, bool* IsMax, bool cLayer)
{
	if (layer == NULL)
		return;
	
	if (*IsMin == true) {
		if (layer->values[x - 1][y - 1] <= c ||
		    layer->values[x][y - 1] <= c ||
		    layer->values[x + 1][y - 1] <= c ||
		    layer->values[x - 1][y] <= c ||
		    // note here its just < instead of <=
		    (cLayer ? false : (layer->values[x][y] < c)) ||
				layer->values[x + 1][y] <= c ||
				layer->values[x - 1][y + 1] <= c ||
				layer->values[x][y + 1] <= c ||
				layer->values[x + 1][y + 1] <= c)
				*IsMin = false;
		}
		if (*IsMax == true) {
			if (layer->values[x - 1][y - 1] >= c ||
				layer->values[x][y - 1] >= c ||
				layer->values[x + 1][y - 1] >= c ||
				layer->values[x - 1][y] >= c ||
				// note here its just > instead of >=
				(cLayer ? false : (layer->values[x][y] > c)) ||
				layer->values[x + 1][y] >= c ||
				layer->values[x - 1][y + 1] >= c ||
				layer->values[x][y + 1] >= c ||
				layer->values[x + 1][y + 1] >= c)
				*IsMax = false;
		}
	}

double DScaleSpace_SToK (int s)
{
	return (pow (2.0, 1.0 / s));
}

// Precompute all gradient magnitude and direction planes for one octave.
void DScaleSpace_GenerateMagnitudeAndDirectionMaps (DScaleSpace* self)
{
	// We leave the first entry to null, and ommit the last. This way, the
	// magnitudes and directions maps have the same index as the
	// imgScaled maps they below to.
	int Count = DScaleSpace_Count(self);
	self->magnitudes = ArrayList_new(Count - 1, ImageMap_delete);
	self->directions = ArrayList_new(Count - 1, ImageMap_delete);

	// Build the maps, omitting the border pixels, as we cannot build
	// gradient information there.
	int s;
	for ( s = 1 ; s < (Count - 1) ; ++s) {
		ImageMap* imgScaledAtS = (ImageMap*) ArrayList_GetItem(self->imgScaled,s);

		ImageMap* magnitudeAtS = ImageMap_new (imgScaledAtS->xDim, imgScaledAtS->yDim);
		ImageMap* directionsAtS = ImageMap_new (imgScaledAtS->xDim, imgScaledAtS->yDim);

		ArrayList_SetItem(self->magnitudes, s, magnitudeAtS);
		ArrayList_SetItem(self->directions, s, directionsAtS);
		
		int x, y;
		for ( y = 1 ; y < (imgScaledAtS->yDim - 1) ; ++y) {
			for ( x = 1 ; x < (imgScaledAtS->xDim - 1) ; ++x) {
				// gradient magnitude m
				magnitudeAtS->values[x][y] = sqrt (
					pow ((double)(imgScaledAtS->values[x + 1][y] -
						  imgScaledAtS->values[x - 1][y]), 2.0) +
					pow ((double)(imgScaledAtS->values[x][y + 1] -
						  imgScaledAtS->values[x][y - 1]), 2.0));

				// gradient direction theta
				directionsAtS->values[x][y] = atan2
					(imgScaledAtS->values[x][y + 1] - imgScaledAtS->values[x][y - 1],
					 imgScaledAtS->values[x + 1][y] - imgScaledAtS->values[x - 1][y]);
			}
		}
	}
}

void DScaleSpace_ClearMagnitudeAndDirectionMaps (DScaleSpace* self)
{
	// TODO: this should free up temporary memory, but this results in a heap corruption ???

	//ArrayList_delete(self->magnitudes, NULL/*ImageMap_delete*/);
	//ArrayList_delete(self->directions, NULL/*ImageMap_delete*/);
	//self->magnitudes = self->directions = NULL;
}

// Build a set of Difference-of-Gaussian (DoG) maps from the gaussian
// blurred images.
// This method has to be called after BuildGaussianMaps has completed.
void DScaleSpace_BuildDiffMaps (DScaleSpace* self)
{
	// Generate DoG maps. The maps are organized like this:
	//    0: D(sigma)
	//    1: D(k * sigma)
	//    2: D(k^2 * sigma)
	//   ...
	//    s: D(k^s * sigma) = D(2 * sigma)
	//  s+1: D(k * 2 * sigma)
	//
	// So, we can start peak searching at 1 to s, and have a DoG map into
	// each direction.
	self->spaces = ArrayList_new(ArrayList_Count(self->imgScaled) - 1, ImageMap_delete);
	
	// After the loop completes, we have used (s + 1) maps, yielding s
	// D-gaussian maps in the range of sigma to 2*sigma, as k^s = 2, which
	// is defined as one octave.
	int sn;
	for ( sn = 0 ; sn < ArrayList_Count(self->spaces) ; ++sn) {
		// XXX: order correct? It should not really matter as we search
		// for both minimums and maximums, but still, what is recommended?
		// (otherwise maybe the gradient directions are inverted?)
		ImageMap* imgScaledAtSnPlus1 = (ImageMap*) ArrayList_GetItem(self->imgScaled, sn+1);
		ImageMap* imgScaledAtSn = (ImageMap*) ArrayList_GetItem(self->imgScaled, sn);
		ImageMap* imgDiff = (ImageMap*) ImageMap_Sub(imgScaledAtSnPlus1, imgScaledAtSn);
		ArrayList_SetItem(self->spaces, sn, imgDiff);
	}
}

// Incrementally blur the input image first so it reaches the next octave.
void DScaleSpace_BuildGaussianMaps (DScaleSpace* self, ImageMap* first, double firstScale,
				    int scales, double sigma)
{
	// We need one more gaussian blurred image than the number of DoG
	// maps. But for the minima/maxima pixel search, we need two more. See
	// BuildDiffMaps.
	self->imgScaled = ArrayList_new(scales + 1 + 1 + 1, ImageMap_delete);
	self->basePixScale = firstScale;
	
	// Ln1(x, y, k^{p+1}) = G(x, y, \sqrt{k^2-1}) * Ln0(x, y, k^p).
	ImageMap* prev = first;
	ArrayList_SetItem(self->imgScaled, 0, first);

	/* Many thanks to Liu for this explanation and fix:
	 *
	 * Gaussian G(sigma), with relation
	 * G(sigma_1) * G(sigma_2) = G(sqrt(sigma_1^2 + * sigma_2^2))
	 *
	 * Then, we have:
	 *
	 * G(k^{p+1}) = G(k^p) * G(sigma),
	 * and our goal is to compute every iterations sigma value so self
	 * equation iteratively produces the next level.  Hence:
	 *
	 * sigma = \sqrt{\left(k^{p+1}\right)^2 - \left(k^p\right)^2}
	 *   = \sqrt{k^{2p+2} - k^{2p}}
	 *   = \sqrt{k^2p (k^2 - 1)}
	 *   = k^p \sqrt{k^2 - 1}
	 *
	 * In the code below, 'w' is the running k^p, where p increases by one
	 * each iteration.  kTerm is the constant \sqrt{k^2 - 1} term.
	 */
	double w = sigma;
	double kTerm = sqrt (pow (DScaleSpace_SToK (scales), 2.0) - 1.0);
	int scI;
	for (scI = 1 ; scI < ArrayList_Count(self->imgScaled) ; ++scI) {
		GaussianConvolution* gauss = GaussianConvolution_new1 (w * kTerm);
		prev = GaussianConvolution_Convolve (gauss, prev);
		GaussianConvolution_delete(gauss);
		ArrayList_SetItem(self->imgScaled, scI, prev);
		w *= DScaleSpace_SToK (scales);
	}

}


ScalePoint* ScalePoint_new0 ()
{
	ScalePoint * self = (ScalePoint*)malloc(sizeof(ScalePoint));
	self->local = NULL;
	return self;
}

void ScalePoint_delete (ScalePoint* self)
{
	if (self) {
		PointLocalInformation_delete(self->local);
		free(self);
	}
}

ScalePoint* ScalePoint_new3 (int x, int y, int level)
{
	ScalePoint* self = ScalePoint_new0();
	self->x = x;
	self->y = y;
	self->level = level;
	self->local = NULL;
	return self;
}

PointLocalInformation* PointLocalInformation_new0 ()
{
	PointLocalInformation* self = (PointLocalInformation*)malloc(sizeof(PointLocalInformation));
	return self;
}

void PointLocalInformation_delete (PointLocalInformation* self)
{
	if (self) {
		free( self );
	}
}

PointLocalInformation* PointLocalInformation_new3 (double fineS, double fineX, double fineY)
{
	PointLocalInformation* self = PointLocalInformation_new0();
	self->fineX = fineX;
	self->fineY = fineY;
	self->scaleAdjust = fineS;
	self->dValue = 0;
	return self;
}




double Keypoint_FVGet (Keypoint* self, int xI, int yI, int oI)
{
	return (self->featureVector[(xI * self->yDim * self->oDim) + (yI * self->oDim) + oI]);
}
void Keypoint_FVSet (Keypoint* self, int xI, int yI, int oI, double value)
{
	self->featureVector[(xI * self->yDim * self->oDim) + (yI * self->oDim) + oI] = value;
}

int Keypoint_FVLinearDim(Keypoint* self) {
	return (self->featureVectorLength);
}

double Keypoint_FVLinearGet (Keypoint* self, int idx)
{
	return self->featureVector[idx];
}

void Keypoint_FVLinearSet (Keypoint* self, int idx, double value)
{
	self->featureVector[idx] = value;
}

void Keypoint_CreateLinearVector (Keypoint* self, int dim)
{
	self->featureVector = (double*)malloc( sizeof(double)*dim);
	self->featureVectorLength = dim;
}

void Keypoint_CreateVector (Keypoint* self, int xDim, int yDim, int oDim)
{
	self->hasFV = true;
	self->xDim = xDim;
	self->yDim = yDim;
	self->oDim = oDim;
	self->featureVectorLength = yDim * xDim * oDim;
	self->featureVector = (double*)calloc(self->featureVectorLength, sizeof(double));
}

Keypoint*  Keypoint_new0() {
	Keypoint* self = (Keypoint*)malloc(sizeof(Keypoint));
	self->featureVector = NULL;
	self->hasFV = false;
	self->featureVectorLength = 0;
	return self;
}

void Keypoint_delete(Keypoint* self) {
	if (self) {
		if (self->featureVector) {
			free(self->featureVector);
		}
		free(self);
	}
}

// Keypoint constructor.
//
// image: The smoothed gaussian image the keypoint was located in.
// x, y: The subpixel level coordinates of the keypoint.
// imgScale: The scale of the gaussian image, with 1.0 being the original
//    detail scale (source image), and doubling at each octave.
// kpScale: The scale of the keypoint.
// orientation: Orientation degree in the range of [-PI ; PI] of the
//    keypoint.
//
// First add a keypoint, then use 'MakeDescriptor' to generate the local
// image descriptor for this keypoint.
Keypoint*  Keypoint_new (ImageMap* image, double x, double y, double imgScale,
		 double kpScale, double orientation)
{
	Keypoint* self = Keypoint_new0();
	self->image = image;
	self->x = x;
	self->y = y;
	self->imgScale = imgScale;
	self->scale = kpScale;
	self->orientation = orientation;
	self->hasFV = false;
	self->featureVector = NULL;
	return self;
}
