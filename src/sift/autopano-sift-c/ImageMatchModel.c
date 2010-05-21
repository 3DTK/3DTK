
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* ImageMatchModel.cs
 *
 * Two image align model for use with RANSAC filtering. Will provide a
 * viewport transformation from one image coordinates to another images, given
 * some keypoint matches between the images. Then, provides information about
 * the likely correctness of further matches.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 */

#include "AutoPanoSift.h"

ImageMatchModel* ImageMatchModel_clone (ImageMatchModel* self)
{
	ImageMatchModel* mod = ImageMatchModel_new (self->fitThresh,
						    self->distanceFactor, self->width, self->height);
	
	mod->trans = AffineTransform2D_clone(self->trans);
	
	return mod;
}

// IComparable
int ImageMatchModel_CompareTo (IComparator* cmp, ImageMatchModel* self, ImageMatchModel* mod)
{
	if (self->fittingErrorSum < mod->fittingErrorSum)
		return (-1);
	else if (self->fittingErrorSum > mod->fittingErrorSum)
		return (1);
	
	return (0);
}

	// IRANSACModel
bool ImageMatchModel_FitModel (ImageMatchModel* self, ArrayList* matches)
{
	if (ArrayList_Count(matches) < 2) {
		//WriteLine ("ImageMatchModel.FitModel: Need at least two matches to fit.");
		
		return (false);
	}
	
	// TODO: least-square match if more than two points are given.
	// For now, just ignore any further matches.
	self->m1 = (Match *) ArrayList_GetItem(matches, 0);
	self->m2 = (Match *) ArrayList_GetItem(matches, 1);
	
	//WriteLine ("Doing transform building...");
	self->trans = AffineTransform2D_BuildTransformFromTwoPairs
		(self->m1->kp1->x, self->m1->kp1->y, self->m2->kp1->x, self->m2->kp1->y,
		 self->m1->kp2->x, self->m1->kp2->y, self->m2->kp2->x, self->m2->kp2->y,
		 self->width / 2, self->height / 2);
	//WriteLine ("   resulting angle: %f", self->trans->centerAngle);
	if (self->trans == NULL) {
		return (false);
	}
	
	return (true);
}

double ImageMatchModel_FittingErrorSingle (ImageMatchModel* self, Match* m)
{
	// Build homegenous coordinates for the point in the second (right)
	// image.
	SimpleMatrix* X = SimpleMatrix_new (3, 1);
	X->values[0][0] = m->kp2->x;
	X->values[1][0] = m->kp2->y;
	X->values[2][0] = 1.0;

	// Calculate the points expected position in the first (left) image.
	SimpleMatrix* Xexpected = SimpleMatrix_Mul((SimpleMatrix*)self->trans, X);

	// Now calculate the difference/distance between the expected and the
	// real point position.
	/*WriteLine ("exp, real: ({0} ; {1}) - ({2} ; {3}) # RANSACREALEXP",
	  Xexpected[0, 0], Xexpected[1, 0], m->kp1->x, m->kp1->y);*/
	double pixDistance = sqrt (pow (m->kp1->x - Xexpected->values[0][0], 2.0) +
				   pow (m->kp1->y - Xexpected->values[1][0], 2.0));

	SimpleMatrix_delete(X);
	SimpleMatrix_delete(Xexpected);

	// Now, we cheat a little. As we cannot associate information with the
	// value we return, but want to adjust for distance based skews in the
	// model, we will subtract the pixel distance with a distance term. If
	// we go below zero, we restore it to zero.
	// The term to subtract is, in LaTeX notation:
	// d_{f} \cdot \left(\frac{d (A, B, X)}{\sqrt{w^2 + h^2}}\right) \cdot t
	//
	// Where d_{f} is the distance factor, d the triangular distance
	// function and t the fitting threshhold. The d(A,B,X) function is
	// defined as:
	//
	// d (A, B, X) := |X - A| + |X - B| - |A - B|
	//
	// and ensures a smooth eliptical weighting around what we assume to
	// be good model predictions.
	double distXA = sqrt (pow (m->kp2->x - self->m1->kp2->x, 2) +
				   pow (m->kp2->y - self->m1->kp2->y, 2));
	double distXB = sqrt (pow (m->kp2->x - self->m2->kp2->x, 2) +
				   pow (m->kp2->y - self->m2->kp2->y, 2));
	double distAB = sqrt (pow (self->m1->kp2->x - self->m2->kp2->x, 2) +
				   pow (self->m1->kp2->y - self->m2->kp2->y, 2));

	double distanceTerm = distXA + distXB - distAB;
	distanceTerm /= sqrt (pow (self->width, 2.0) + pow (self->height, 2.0));
	double distanceReduce = self->distanceFactor * distanceTerm * self->fitThresh;
	/*WriteLine ("reducing distance %f by %f due to distance term %f, thresh %f",
	  pixDistance, distanceReduce, distanceTerm, fitThresh);*/
	pixDistance -= distanceReduce;
	
	if (pixDistance < 0.0)
		pixDistance = 0.0;

	return (pixDistance);
}

bool ImageMatchModel_ThreshholdPoint (ImageMatchModel* self, double fitError)
{
	/*WriteLine ("TreshholdPoint: %f to %f # RANSACTHRESH",
			fitError, self->fitThresh);*/
	if (fitError > self->fitThresh)
		return (false);
	
	return (true);
}

void ImageMatchModel_SetFittingErrorSum(ImageMatchModel* self, double value)
{
	self->fittingErrorSum = value;
}

double ImageMatchModel_GetFittingErrorSum(ImageMatchModel* self)
{
	return self->fittingErrorSum;
}
void ImageMatchModel_SetFittingGround (ImageMatchModel* self, ArrayList* value)
{
	self->fittingGround = value;
}

ArrayList* ImageMatchModel_GetFittingGround (ImageMatchModel* self)
{
	return self->fittingGround;
}


IRANSACModel ImageMatchModel_vtbl = {
	(IRANSACModel *( *)(IRANSACModel *)) ImageMatchModel_clone,
	(void ( *)(IRANSACModel *))ImageMatchModel_delete,
	(int ( *)(IRANSACModel *,ArrayList *)) ImageMatchModel_FitModel,
	(double ( *)(IRANSACModel *,void *))ImageMatchModel_FittingErrorSingle,
	(int ( *)(IRANSACModel *,double)) ImageMatchModel_ThreshholdPoint,
	(double ( *)(IRANSACModel *)) ImageMatchModel_GetFittingErrorSum,
	(void ( *)(IRANSACModel *,double)) ImageMatchModel_SetFittingErrorSum,
	(ArrayList *( *)(IRANSACModel *)) ImageMatchModel_GetFittingGround,
	(void ( *)(IRANSACModel *,ArrayList *)) ImageMatchModel_SetFittingGround,
	(int ( *)(IComparator *,IRANSACModel *,IRANSACModel *)) ImageMatchModel_CompareTo
};


ImageMatchModel* ImageMatchModel_new0()
{
	ImageMatchModel* self = (ImageMatchModel*)malloc(sizeof(ImageMatchModel));
	self->base = ImageMatchModel_vtbl;
	return self;
}

void ImageMatchModel_delete(ImageMatchModel* self)
{
	if (self) {
		AffineTransform2D_delete(self->trans);
		free(self);
	}
}

	// Constructor
	// fitThresh: Threshhold value for further matching of a single match. If
	// the position of the transformed pixel is more than fitThresh pixels
	// away, its considered invalid.
	// distanceFactor: The distance gratifying factor. Use zero to nullify the
	// effect of distance based relaxing.
	// width, heigth: Image format.
ImageMatchModel* ImageMatchModel_new (double fitThresh, double distanceFactor,
		int width, int height)
{
	ImageMatchModel* self = ImageMatchModel_new0();
	self->fitThresh = fitThresh;
	self->distanceFactor = distanceFactor;
	self->width = width;
	self->height = height;
	self->trans = NULL;
	return self;
}

char* ImageMatchModel_ToString (ImageMatchModel* self)
{
	char* str = (char*)malloc(32);
	sprintf(str, "Model: sumError = %f", self->fittingErrorSum);
	return str;
}


// Driver class for the RANSAC processing.

ImageMatchModel* MatchDriver_FilterMatchSet (ArrayList* matches,
		double distanceFactor, int width, int height)
{
	// Need at least one match pair plus one for verification to use
	// RANSAC. If we only have two we could apply RANSAC, but it would
	// always fit perfectly, so its of no use.
	if (ArrayList_Count(matches) <= 2)
		return (NULL);

	// Create a generic RANSAC algorithm processor, which requires at
	// least two points to be able to fit the model, with an expected 50%
	// correct matchrate and an additional safe-margin of five times the
	// standard-deviation of the correct match probabilities.
	RANSAC* ran = RANSAC_new (2,
				 RANSAC_GetKFromGoodfraction (2, 0.5, 10));

	// Create a template model with a threshhold distance of 10 pixels.
	ImageMatchModel* mod = ImageMatchModel_new
		(4.0, distanceFactor, width, height);

	// Find appropiate models with two additional matching points
	// required. (FIXME: Maybe change to 1?)
	ArrayList* models = (ArrayList*)RANSAC_FindModels (ran, (IRANSACModel*)mod, matches, 2);
	/*foreach (ImageMatchModel model in models)
	  Console.WriteLine (model.ToString ());*/

	ImageMatchModel_delete(mod);

	RANSAC_delete(ran);

	if (ArrayList_Count(models) == 0) {
		ArrayList_delete(models);
		return (NULL);
	}

	ImageMatchModel* best = (ImageMatchModel*) ArrayList_GetItem(models, 0);
	// Make sure best doesn't get erased by ArrayList_delete below.
	ArrayList_SetItem(models, 0, NULL);

	ArrayList_delete(models);
	return (best);
}
