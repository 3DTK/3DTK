
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* ImageMap.cs
 *
 * Abstract floating point image map processing functionality.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 */

#include "AutoPanoSift.h"

ImageMap* ImageMap_new0() 
{
	ImageMap* self = (ImageMap*)malloc(sizeof(ImageMap));
	return self;
}

ImageMap* ImageMap_new(int xDim, int yDim) 
{
	ImageMap* self = ImageMap_new0();
	self->xDim = xDim;
	self->yDim = yDim;
	self->values = FloatMap_new(xDim, yDim);
	return self;
}

void ImageMap_delete(ImageMap* self) 
{
	if (self) {
		if (self->values) {
			FloatMap_delete(self->values);
		}
		free(self);
	}
}

ImageMap* ImageMap_clone(ImageMap* self) 
{
	ImageMap* result = ImageMap_new(self->xDim, self->yDim);
	int x;
	for(x=0; x<self->xDim; x++) {
		memcpy(result->values[x], self->values[x], self->yDim*sizeof(double));
	}
	return result;
}

void ImageMap_Save(ImageMap* self, char* filename, char* comment) 
{
	FILE* fp = fopen(filename, "w");
	fprintf(fp, "# %d %d\n", self->xDim, self->yDim);
	fprintf(fp, "# %s\n", comment);
	int x;
	int y;
	for (y=0; y<self->yDim; y++) {
		for (x=0; x<self->xDim; x++) {
			fprintf(fp, "%d %d %.15lf\n", y, x, self->values[x][y]);
		}
		fprintf(fp, "\n");
	}
	fclose(fp);
}

// Double the size of an imagemap using linear interpolation.
// It is not a real doubling as the last line is omitted (the image size
// would always be odd otherwise and we have no second line for
// interpolation).
ImageMap* ImageMap_ScaleDouble(ImageMap* self) 
{
	// Doubling an image with x/y dimensions less or equal than 2 will
	// result in an image with just (2, 2) dims, so its useless.
	if (self->xDim <= 2 || self->yDim <= 2)
		return NULL;

	ImageMap* result = ImageMap_new(self->xDim * 2 - 2, self->yDim * 2 - 2);

	int x; 
	int y;
	for(y=0; y < (self->yDim-1); y++) {
		for(x=0; x < (self->xDim-1); x++) {	    
			result->values[x*2+0][y*2+0]=(self->values[x][y]);
			result->values[x*2+1][y*2+0]=(self->values[x][y]+self->values[x+1][y])/2.0;
			result->values[x*2+0][y*2+1]=(self->values[x][y]+self->values[x][y+1])/2.0;
			result->values[x*2+1][y*2+1]=(self->values[x][y]+self->values[x+1][y]+self->values[x][y+1]+self->values[x+1][y+1])/4.0;
		}
	}
	return result;
}

ImageMap* ImageMap_ScaleHalf(ImageMap* self) 
{
	ImageMap* result = ImageMap_new(self->xDim/2, self->yDim/2);
	int x; 
	int y;
	for(y=0; y<result->yDim; y++) {
		for(x=0; x<result->xDim; x++) {
#if 1
			result->values[x][y]=self->values[2*x  ][2*y  ];
#else
			// 2*x+1 is out of bound!
			result->values[x][y]=( self->values[2*x  ][2*y  ]
					      +self->values[2*x+1][2*y  ]
					      +self->values[2*x  ][2*y+1]
					      +self->values[2*x+1][2*y+1])/4.0;
#endif
		}
	}
	return result;
}

ImageMap* ImageMap_Add(ImageMap* f1, ImageMap* f2) 
{
	if (f1->xDim != f2->xDim || f1->yDim != f2->yDim) {
		FatalError("Mismatching dimensions");
	}
	ImageMap* result = ImageMap_new(f1->xDim, f1->yDim);
	int x; 
	int y;
	for(y=0; y<f1->yDim; y++) {
		for(x=0; x<f1->xDim; x++) { 
			result->values[x][y]=
				f1->values[x][y]+f2->values[x][y];
		}
	}
	return result;
}

ImageMap* ImageMap_Sub(ImageMap* f1, ImageMap* f2) 
{
	if (f1->xDim != f2->xDim || f1->yDim != f2->yDim) {
		FatalError("Mismatching dimensions");
	}
	ImageMap* result = ImageMap_new(f1->xDim, f1->yDim);
	int x; 
	int y;
	for(y=0; y<f1->yDim; y++) {
		for(x=0; x<f1->xDim; x++) {
			result->values[x][y]=
				f1->values[x][y]-f2->values[x][y];
		}
	}
	return result;
}

ImageMap* ImageMap_Mul(ImageMap* f1, ImageMap* f2) 
{
	if (f1->xDim != f2->xDim || f1->yDim != f2->yDim) {
		FatalError("Mismatching dimensions");
	}
	ImageMap* result = ImageMap_new(f1->xDim, f1->yDim);
	int x; 
	int y;
	for(y=0; y<f1->yDim; y++) {
		for(x=0; x<f1->xDim; x++) {
			result->values[x][y]=
				f1->values[x][y]*f2->values[x][y];
		}
	}
	return result;
}

// Normalize: Find the minimum to maximum range, then stretch and limit
// those to exactly 0.0 to 1.0. If both the minimum and maximum values are
// equal, no normalization takes place.
void ImageMap_Normalize(ImageMap* self) 
{
	int x; 
	int y;
	double min=self->values[0][0];
	double max=self->values[0][0];
	for(y=0; y<self->yDim; y++) {
		for(x=0; x<self->xDim; x++) {
			if (min>self->values[x][y]) min = self->values[x][y];
			if (max<self->values[x][y]) max = self->values[x][y];
		}
	}
	if (min == max) return;
	double diff = max-min;
	for(y=0; y<self->yDim; y++) {
		for(x=0; x<self->xDim; x++) {
			self->values[x][y] = (self->values[x][y]-min) / diff;
		}
	}    
}

ImageMap* ImageMap_GaussianConvolution(ImageMap* self, double sigma) 
{
	GaussianConvolution* conv = GaussianConvolution_new1(sigma);
	ImageMap* result =  GaussianConvolution_Convolve(conv, self);
	GaussianConvolution_delete(conv);
	return result;
}

