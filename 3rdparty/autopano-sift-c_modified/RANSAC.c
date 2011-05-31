
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* RANSAC - RANdom SAmple Consensus
 *
 * Generic RANSAC fitting functionality.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 *
 * Based on "Computer Vision - a modern approach", Forsyth & Ponce, pp. 346
 */

#include "AutoPanoSift.h"

RANSAC* RANSAC_new0 ()
{
	return (RANSAC*)malloc(sizeof(RANSAC));
}

void RANSAC_delete(RANSAC* self)
{
	free(self);
}

// n: Smallest number of points to be able to fit the model.
// k: The number of iterations required.
RANSAC* RANSAC_new (int n, int k)
{
	RANSAC* self = RANSAC_new0();
	self->n = n;
	self->k = k;
	return self;
}

// ArrayList of Model's, sorted by summed fitting error.
// model: Model to fit
// points: List of point data to fit
// d: Number of nearby points required for a model to be accepted
ArrayList* RANSAC_FindModels (RANSAC* self, IRANSACModel* model, ArrayList* points, int d)
{
	Random* rand = Random_new0 ();
	ArrayList* result = ArrayList_new0 (IRANSACModel_delete);
	
	if (ArrayList_Count(points) < self->n)
		FatalError("List of data is smaller than minimum fit requires.");

	int ki;
	for (ki = 0 ; ki < self->k ; ++ki) {
		ArrayList* samples = ArrayList_new0 (NULL);

		// Build random samples
		int ri;
		for ( ri = 0 ; ri < self->n ; ++ri) {
			void* sampleToAdd;
			sampleToAdd = ArrayList_GetItem(points, Random_Next(rand, 0, ArrayList_Count(points)));

			if (ArrayList_Contains (samples, sampleToAdd))
				continue;

			ArrayList_AddItem (samples, sampleToAdd);
		}

		if (IRANSACModel_FitModel (model, samples) == false) {
			ArrayList_delete(samples);
			continue;
		}

		ArrayList* good = ArrayList_new0 (NULL);
		double overAllFittingError = 0.0;

		// Check all non-sample points for fit.
		int i;
		for (i=0; i<ArrayList_Count(points); i++) {
			void* point = ArrayList_GetItem(points, i);
			if (ArrayList_Contains (samples, point))
				continue;

			double fitError = IRANSACModel_FittingErrorSingle (model, point);
			if (IRANSACModel_ThreshholdPoint (model, fitError)) {
				ArrayList_AddItem (good, point);
				overAllFittingError += fitError;
			}
		}

		// good contains a list of all fitting points now. Check if there
		// are more than d points near our model.
		if (ArrayList_Count(good) >= d) {
			ArrayList_AddRange(good, samples);
			IRANSACModel* modelGood = IRANSACModel_clone ( model);

			if (IRANSACModel_FitModel (modelGood, good) == false) {
				// This is a rare case when the distance between the
				// sample points is zero. It could happen, but is very
				// rare.
				//WriteLine ("RANSAC: Fitting model from good samples failed, discarding this model.");
				ArrayList_delete(samples);
				IRANSACModel_delete(modelGood);
				continue;
			}

			IRANSACModel_SetFittingErrorSum(modelGood, overAllFittingError / ArrayList_Count(good));
			IRANSACModel_SetFittingGround(modelGood, good);

			ArrayList_AddItem (result, modelGood);
		}
		ArrayList_delete(samples);
	}

	Random_delete(rand);

	IComparator cmp;
	cmp.compareTo = (int ( *)(IComparator *,const void *,const void *)) model->compareTo;
	ArrayList_Sort( result, &cmp);
	//WriteLine ("got %d modelfits", ArrayList_Count(result));

	return (result);
}

// Calculate the expected number of draws required when a fraction of
// 'goodFraction' of the sample points is good and at least 'n' points are
// required to fit the model. Add 'sdM' times the standard deviation to be
// sure.
// n: > 0
// goodFraction: > 0.0 and <= 1.0
// sdM: >= 0
// return the guess for k, the expected number of draws.

int RANSAC_GetKFromGoodfraction (int n, double goodFraction, int sdM)
{
	double result;
	
	result = pow (goodFraction, -n);
	if (sdM > 0)
		result += sdM * sqrt (1.0 - pow (goodFraction, n));
	
	return ((int) (result + 0.5));
}

IRANSACModel* IRANSACModel_new0() {
	IRANSACModel* self = (IRANSACModel*)malloc(sizeof(IRANSACModel));
	return self;
}
void IRANSACModel_delete(IRANSACModel* self) {
	if (self->deletefn) {
		self->deletefn(self);
	} else {
		free(self);
	}
}
IRANSACModel* IRANSACModel_clone(IRANSACModel* self) {
	return self->clone(self);
}
bool IRANSACModel_FitModel(IRANSACModel* self, ArrayList* points) {
	return self->fitModel(self, points);
}
double IRANSACModel_FittingErrorSingle(IRANSACModel* self, void* point) {
	return self->fittingErrorSingle(self, point);
}
bool IRANSACModel_ThreshholdPoint (IRANSACModel* self, double value) {
	return self->threshholdPoint(self, value);
}
void IRANSACModel_SetFittingErrorSum(IRANSACModel* self, double value) {
	self->setFittingErrorSum(self, value);
}
double IRANSACModel_GetFittingErrorSum(IRANSACModel* self) {
	return self->getFittingErrorSum(self);
}
void IRANSACModel_SetFittingGround(IRANSACModel* self, ArrayList* points) {
	self->setFittingGround(self, points);
}
ArrayList* IRANSACModel_GetFittingGround(IRANSACModel* self) {
	return self->getFittingGround(self);
}

#ifdef TEST_MAIN
// Test Main
int main (int argc, char* argv[])
{
	WriteLine ("n = 3, goodFraction = 0.3, sdM = 0: %d",
			   RANSAC_GetKFromGoodfraction (3, 0.3, 0));
	WriteLine ("n = 3, goodFraction = 0.3, sdM = 10: %d",
			   RANSAC_GetKFromGoodfraction (3, 0.3, 10));
	return 0;
}
#endif
