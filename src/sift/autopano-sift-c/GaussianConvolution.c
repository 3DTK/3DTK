
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* GaussianConvolution.cs
 *
 * Gaussian convolution filter, separated passes.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 */

// TODO: There might be a way to do this faster by creating a different mask
// and using just one pass from left to right per row or up to down from
// column, instead of (filterSize * dim) passes.

#include "AutoPanoSift.h"

ConvLinearMask* ConvLinearMask_new0()
{
	ConvLinearMask* self = (ConvLinearMask*)malloc(sizeof(ConvLinearMask));
	self->Dim = 0;
	self->Middle = 0;
	self->items = NULL;
	self->MaskSum = 0;
	return self;
}

ConvLinearMask* ConvLinearMask_new1(int dim)
{
	ConvLinearMask* self = ConvLinearMask_new0();
	self->Dim = dim;
	self->Middle = dim / 2;
	self->items = (double*)calloc(sizeof(double),dim);
	self->MaskSum = 0;
	return self;
}

void ConvLinearMask_delete(ConvLinearMask* self) 
{
	if (self->items != NULL) {
		free(self->items);
		self->items = NULL;
	}
	free(self);
}

GaussianConvolution* GaussianConvolution_new0()
{
	GaussianConvolution* self = (GaussianConvolution*)malloc(sizeof(GaussianConvolution));
	return self;
}

void GaussianConvolution_delete (GaussianConvolution* self)
{
	if (self) {
		if (self->mask != NULL) {
			ConvLinearMask_delete(self->mask);
			self->mask = NULL;
		}
		free(self);
	}
}

// From "Image Processing, Analysis and Machine Vision", pp. 84:
// 'Pixels more distant from the center of the operator have smaller
// influence, and pixels farther than 3 \sigma from the center have
// neglible influence.'
//
// So, find the kernel size by rounding twice 3 \sigma up.
GaussianConvolution* GaussianConvolution_new1(double sigma) 
{
	return GaussianConvolution_new2(sigma, 1 + 2 * ((int) (3.0 * sigma)));
}

// Like Generate, but manually specifying the kernel size.
GaussianConvolution* GaussianConvolution_new2 (double sigma, int dim)
{
	// Assert the kernel size is odd, so we have a clear center pixel.
	dim |= 1;
	GaussianConvolution* self = GaussianConvolution_new0();
	self->mask = ConvLinearMask_new1(dim);
    
	double sigma2sq = 2 * sigma * sigma;
	double normalizeFactor = 1.0 / (sqrt (2.0 * M_PI) * sigma);
    
	int n;
	for ( n = 0 ; n < dim ; ++n) {
		int relPos = n - self->mask->Middle;
	
		double G = (relPos * relPos) / sigma2sq;
		G = exp (-G);
		G *= normalizeFactor;
		self->mask->items[n] = G;
		self->mask->MaskSum += G;
	}
	return self;
}

// Apply the gaussian filter.
ImageMap* GaussianConvolution_Convolve (GaussianConvolution* self, ImageMap* img)
{
	return ConvolutionFilter_Convolve (img, self->mask);
}

int Direction_Vertical = 0;
int Direction_Horizontal = 1;

// Static utility functions for convolution
//
ImageMap* ConvolutionFilter_Convolve (ImageMap* img, ConvLinearMask* mask)
{
    ImageMap* res = ImageMap_new (img->xDim, img->yDim);
    ImageMap* res2 = ImageMap_new (img->xDim, img->yDim);
    
    ConvolutionFilter_Convolve1D (res, mask, img, Direction_Vertical);
    ConvolutionFilter_Convolve1D (res2, mask, res, Direction_Horizontal);

    ImageMap_delete(res);

    return (res2);
}

void ConvolutionFilter_Convolve1D (ImageMap* dest, ConvLinearMask* mask,
				   ImageMap* src, int dir)
{
	int maxN=0;	// outer loop maximum index
	int maxP=0;	// inner loop maximum index

	if (dir == Direction_Vertical) {
		maxN = src->xDim;
		maxP = src->yDim;
	} else if (dir == Direction_Horizontal) {
		maxN = src->yDim;
		maxP = src->xDim;
	} else
		FatalError ("TODO: invalid direction");

	int n;
	for ( n = 0 ; n < maxN ; ++n) {
		int p;
		for ( p = 0 ; p < maxP ; ++p) {
			double val = ConvolutionFilter_CalculateConvolutionValue1D (src, mask,
										    n, p, maxN, maxP, dir);

			if (dir == Direction_Vertical)
				ImageMap_SetPixel(dest, n, p, val);
			else
				ImageMap_SetPixel(dest, p, n, val);
		}
	}
}

double ConvolutionFilter_CalculateConvolutionValue1D (ImageMap* src,
		ConvLinearMask* mask, int n, int p, int maxN, int maxP, int dir)
{
	double sum = 0.0;
	
	bool isOut = false;
	double outBound = 0.0;	// values that are out of bound
	
	int xw;
	for (xw = 0 ; xw < mask->Dim ; ++xw) {
		int curAbsP = xw - mask->Middle + p;
		
		if (curAbsP < 0 || curAbsP >= maxP) {
			isOut = true;
			outBound += mask->items[xw];
			
			continue;
		}
		
		if (dir == Direction_Vertical)
			sum += mask->items[xw] * ImageMap_GetPixel(src, n, curAbsP);
		else
			sum += mask->items[xw] * ImageMap_GetPixel(src, curAbsP, n);
	}
	
	// if part of the mask was outside, correct the resulting value by the
	// in/out ratio.
	if (isOut)
		sum *= 1.0 / (1.0 - outBound);
	
	return (sum);
}

