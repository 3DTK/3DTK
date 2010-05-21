
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* KDTree.cs
 *
 * A vanilla k-d tree implementation.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 *
 * Based on "An introductory tutorial on kd-trees" by Andrew W. Moore,
 * available at http://www.ri.cmu.edu/pubs/pub_2818.html
 */

#include "AutoPanoSift.h"


/* SortedLimitedList replacement by Eric Engle
 *
 * Changes:
 * Modified Add(), and implemented Set to handle node setting semantics
 * not provided by the .NET library.
 *
 * Performance notes:
 * This routine uses a simple insertion sort to handle calls to Add.
 * Each element is compared from right to left. If obj is smaller than the
 * current array object, that object is slid to the right.  Otherwise the hole
 * from the last slide operation is used to hold obj.
 *   Most of the calls to Add() will return -1, i.e. in the normal case only a
 * fraction of the items will be smaller than the largest item on the list.
 * This common case is recognized in the first comparison.  Iteration occurs
 * only for those items that belong on the list, so for the normal case self
 * operation is faster than it's strictly linear performance would suggest.
 */

SortedLimitedList* SortedLimitedList_new0 ()
{
	SortedLimitedList* self = (SortedLimitedList*)malloc(sizeof(SortedLimitedList));
	return self;
}

void SortedLimitedList_delete (SortedLimitedList* self)
{
	ArrayList_delete(&self->base);
}

SortedLimitedList* SortedLimitedList_new (int maxElements, void* deletefn)
{
	SortedLimitedList* self = SortedLimitedList_new0();
	ArrayList_init((ArrayList*)self, deletefn);
	self->max = maxElements;
	self->deletefn = (void (*)(void *)) deletefn;
	return self;
}

// Sets the argument index to the argument object.
// Replaces the node if it already exists,
// adds a new node if at the end of the list,
// does nothing otherwise.
void SortedLimitedList_SetItem (SortedLimitedList* self, int idx, void* value)
{
	if (idx < ArrayList_Count(&self->base)) {
		ArrayList_SetItem(&self->base, idx, value);
	} else if (idx == ArrayList_Count(&self->base)) { // TODO: should check for max?
		ArrayList_AddItem(&self->base, value);
	} else {
		if (self->deletefn) {
			self->deletefn(value);
		}
	}
}

int SortedLimitedList_Count (SortedLimitedList* self)
{
	return ArrayList_Count(&self->base);
}

void* SortedLimitedList_GetItem (SortedLimitedList* self, int i)
{
	return ArrayList_GetItem(&self->base, i);
}

void SortedLimitedList_RemoveAt (SortedLimitedList* self, int i)
{
	return ArrayList_RemoveAt(&self->base, i);
}


//	Processes list from right to left, sliding each node that is greater
//	than 'self' to the right.  The loop ends when either the first node is
//	reached, meaning obj is a new minimum, or it's proper sorted position
//	in the list is reached.
//	Returns position of obj or -1 if obj was not placed.

int SortedLimitedList_AddItem (SortedLimitedList* self, void* value)
{
	int pos = ArrayList_Count(&self->base);
	while (pos > 0 && self->comparator.compareTo(&self->comparator, ArrayList_GetItem(&self->base, pos-1), value) >= 0) {
		if (pos < self->max) {
			if (pos==self->max-1) {
				if (ArrayList_Count(&self->base) == self->max) {
					if (self->deletefn) {
						self->deletefn(ArrayList_GetItem(&self->base, pos));
					}
				}
			}
			SortedLimitedList_SetItem(self, pos, ArrayList_GetItem(&self->base, pos-1));
		}
		pos --;
	}
	
	if (pos < self->max) {
		if (pos==self->max-1) {
			if (ArrayList_Count(&self->base) == self->max) {
				if (self->deletefn) {
					self->deletefn(ArrayList_GetItem(&self->base, pos));
				}
			}
		}
	        SortedLimitedList_SetItem(self, pos, value);
	} else {
		if (self->deletefn) {
			self->deletefn(value);
		}
		pos = -1;
	}
	
	return pos;
}




int IKDTreeDomain_GetDimensionCount(IKDTreeDomain* self)
{
	return self->getDimensionCount(self);
}

int IKDTreeDomain_GetDimensionElement(IKDTreeDomain* self, int dim)
{
	return self->getDimensionElement(self, dim);
}


KDTree* KDTree_new0 ()
{
	KDTree* self = (KDTree*)malloc(sizeof(KDTree));
	self->left = NULL;
	self->right = NULL;
	self->dr = NULL;
	return self;
}

void KDTree_delete(KDTree* self)
{
	if (self) {
		KDTree_delete(self->left);
		KDTree_delete(self->right);
		// TODO: KDTreeDomain_delete(self->dr)?
		free(self);
	}
}

bool KDTreeBestEntry_Equals (KDTreeBestEntry* be1, KDTreeBestEntry* be2)
{
	return (be1->neighbour == be2->neighbour);
}

KDTreeBestEntry* KDTreeBestEntry_new0()
{
	KDTreeBestEntry* self = (KDTreeBestEntry*)malloc(sizeof(KDTreeBestEntry));
	return self;
}

void KDTreeBestEntry_delete(KDTreeBestEntry* self)
{
	if (self) {
		free(self);
	}
}

KDTreeBestEntry* KDTreeBestEntry_new3 (IKDTreeDomain* neighbour, int distanceSq, bool squared)
{
	KDTreeBestEntry* self= KDTreeBestEntry_new0();
	self->neighbour = neighbour;
	self->distanceSq = distanceSq;
	self->squared = true;
	return self;
}

KDTreeBestEntry* KDTreeBestEntry_new (IKDTreeDomain* neighbour, double dist)
{
	KDTreeBestEntry* self= KDTreeBestEntry_new0();
	self->neighbour = neighbour;
	self->distance = dist;
	self->squared = false;
	return self;
}

IKDTreeDomain* KDTreeBestEntry_Neighbour (KDTreeBestEntry* self)
{
	return self->neighbour;
}

int KDTreeBestEntry_CompareTo (IComparator* self, KDTreeBestEntry* be1, KDTreeBestEntry* be2)
{
	if (be1->squared) {
		if (be1->distanceSq < be2->distanceSq)
			return (-1);
		else if (be1->distanceSq > be2->distanceSq)
			return (1);
	
		return (0);
	} else {
		if (be1->distance < be2->distance)
			return (-1);
		else if (be1->distance > be2->distance)
			return (1);
	
		return (0);
	}
}

KDTreeHREntry* KDTreeHREntry_new0()
{
	KDTreeHREntry* self = (KDTreeHREntry*)malloc(sizeof(KDTreeHREntry));
	self->rect = NULL;
	self->tree = NULL;
	self->pivot = NULL;
	return self;
}

void KDTreeHREntry_delete(KDTreeHREntry* self)
{
	if (self) {
	    HyperRectangle_unref(self->rect);
	    //KDTree_delete(self->tree);
	    //IKDTreeDomain_delete(self->pivot);
	    free(self);
	}
}


KDTreeHREntry* KDTreeHREntry_new (HyperRectangle* rect, KDTree* tree, IKDTreeDomain* pivot,
				  double dist)
{
	KDTreeHREntry* self = KDTreeHREntry_new0();
	self->rect = HyperRectangle_ref(rect);
	self->tree = tree;
	self->pivot = pivot;
	self->dist = dist;
	return self;
}

int KDTreeHREntry_CompareTo (IComparator* self, KDTreeHREntry* hre1, KDTreeHREntry* hre2)
{
	if (hre1->dist < hre2->dist)
		return (-1);
	else if (hre1->dist > hre2->dist)
		return (1);
	
	return (0);
}

HyperRectangle* HyperRectangle_new0()
{
	HyperRectangle* self = (HyperRectangle*)malloc(sizeof(HyperRectangle));
	self->ref = 0;
	return self;
}

void HyperRectangle_delete (HyperRectangle* self)
{
	if (self) {
		if (self->leftTop) {
			free(self->leftTop);
			self->leftTop = NULL;
		}
		if (self->rightBottom) {
			free(self->rightBottom);
			self->rightBottom = NULL;
		}
		self->dim = 0;
		free(self);
	}
}

HyperRectangle* HyperRectangle_ref (HyperRectangle* self)
{
	self->ref++;
	return self;
}

void HyperRectangle_unref (HyperRectangle* self)
{
	self->ref--;
	if (self->ref == 0) {
		//HyperRectangle_delete(self);
	}
}


HyperRectangle* HyperRectangle_new (int dim)
{
	HyperRectangle* self = HyperRectangle_new0();
	self->dim = dim;
	self->leftTop = (int*)malloc(dim*sizeof(int));
	self->rightBottom = (int*)malloc(dim*sizeof(int));
	return self;
}

HyperRectangle* HyperRectangle_clone (HyperRectangle* self)
{
	HyperRectangle* rec = HyperRectangle_new (self->dim);
	
	int n;
	for ( n = 0 ; n < self->dim ; ++n) {
		rec->leftTop[n] = self->leftTop[n];
		rec->rightBottom[n] = self->rightBottom[n];
	}
	
	return (rec);
}

HyperRectangle* HyperRectangle_CreateUniverseRectangle (int dim)
{
	HyperRectangle* rec = HyperRectangle_new (dim);

	int n;
	for ( n = 0 ; n < dim ; ++n) {
		rec->leftTop[n] = INT_MIN;
		rec->rightBottom[n] = INT_MAX;
	}
	
	return (rec);
}

HyperRectangle* HyperRectangle_SplitAt (HyperRectangle* self, int splitDim, int splitVal)
{
	if (self->leftTop[splitDim] >= splitVal || self->rightBottom[splitDim] < splitVal)
		FatalError("SplitAt with splitpoint outside rec");

	HyperRectangle* r2 = HyperRectangle_clone(self);
	self->rightBottom[splitDim] = splitVal;
	r2->leftTop[splitDim] = splitVal;

	return (r2);
}

bool HyperRectangle_IsIn (HyperRectangle* self, IKDTreeDomain* target)
{
	if (IKDTreeDomain_GetDimensionCount(target) != self->dim)
		FatalError ("IsIn dimension mismatch");

	int n;
	for (n = 0 ; n < self->dim ; ++n) {
		int targD = IKDTreeDomain_GetDimensionElement(target, n);
		
		if (targD < self->leftTop[n] || targD >= self->rightBottom[n])
			return (false);
	}
	
	return (true);
}

// Return true if any part of this HR is reachable from target by no
// more than 'distRad', false otherwise.
// The algorithm is specified in the kd-tree paper mentioned at the
// top of this file, in section 6-7. But there is a mistake in the
// third distinct case, which should read "hrMax" instead of "hrMin".
bool HyperRectangle_IsInReach (HyperRectangle* self, IKDTreeDomain* target, double distRad)
{
	return (HyperRectangle_Distance (self, target) < distRad);
}

// Return the distance from the nearest point from within the HR to
// the target point.
double HyperRectangle_Distance (HyperRectangle* self, IKDTreeDomain* target)
{
	int closestPointN;
	int distance = 0;

	// first compute the closest point within hr to the target. if
	// this point is within reach of target, then there is an
	// intersection between the hypersphere around target with radius
	// 'dist' and this hyperrectangle.
	int n;
	for ( n = 0 ; n < self->dim ; ++n) {
		int tI = IKDTreeDomain_GetDimensionElement (target, n);
		int hrMin = self->leftTop[n];
		int hrMax = self->rightBottom[n];

		closestPointN = 0;
		if (tI <= hrMin) {
			closestPointN = hrMin;
		} else if (tI > hrMin && tI < hrMax) {
			closestPointN = tI;
		} else if (tI >= hrMax) {
			closestPointN = hrMax;
		}

		int dimDist = tI - closestPointN;
		distance += dimDist * dimDist;
	}

	return (sqrt ((double) distance));
}


// Find the nearest neighbour to the hyperspace point 'target' within the
// kd-tree. After return 'resDist' contains the absolute distance from the
// target point. The nearest neighbour is returned.
IKDTreeDomain* KDTree_NearestNeighbour (KDTree* self, IKDTreeDomain* target, double* resDist)
{
	ArrayList* hrl = ArrayList_new0(HyperRectangle_delete);

	HyperRectangle* hr =
		HyperRectangle_CreateUniverseRectangle (IKDTreeDomain_GetDimensionCount(target));
	
	ArrayList_AddItem(hrl, hr);

	IKDTreeDomain* nearest = KDTree_NearestNeighbourI (self, target, hr,
							   Double_PositiveInfinity, resDist, hrl);
	*resDist = sqrt (*resDist);

	ArrayList_delete(hrl);

	return (nearest);
}


	// Internal recursive algorithm for the kd-tree nearest neighbour search.
IKDTreeDomain* KDTree_NearestNeighbourI (KDTree* self, IKDTreeDomain* target, HyperRectangle* hr,
					 double maxDistSq, double* resDistSq, ArrayList* hrl)
{
	//WriteLine ("C NearestNeighbourI called");

	*resDistSq = Double_PositiveInfinity;

	IKDTreeDomain* pivot = self->dr;

	HyperRectangle* leftHr = hr;
	HyperRectangle* rightHr = HyperRectangle_SplitAt(leftHr, self->splitDim,
							 IKDTreeDomain_GetDimensionElement (pivot, self->splitDim));

	ArrayList_AddItem(hrl, rightHr);

	HyperRectangle* nearerHr = NULL;
	HyperRectangle* furtherHr =NULL;
	KDTree* nearerKd = NULL;
	KDTree* furtherKd = NULL;

	// step 5-7
	if (IKDTreeDomain_GetDimensionElement (target, self->splitDim) <=
	    IKDTreeDomain_GetDimensionElement (pivot, self->splitDim))
	{
		nearerKd = self->left;
		nearerHr = leftHr;
		furtherKd = self->right;
		furtherHr = rightHr;
	} else {
		nearerKd = self->right;
		nearerHr = rightHr;
		furtherKd = self->left;
		furtherHr = leftHr;
	}

	// step 8
	IKDTreeDomain* nearest = NULL;
	double distSq;
	if (nearerKd == NULL) {
		distSq = Double_PositiveInfinity;
	} else {
		nearest = KDTree_NearestNeighbourI (nearerKd, target, nearerHr,
						      maxDistSq, &distSq, hrl);
	}

	// step 9
	maxDistSq = min (maxDistSq, distSq);

	// step 10
	if (HyperRectangle_IsInReach (furtherHr, target, sqrt (maxDistSq))) {
		double ptDistSq = KDTree_DistanceSq (pivot, target);
		if (ptDistSq < distSq) {
			// steps 10.1.1 to 10.1.3
			nearest = pivot;
			distSq = ptDistSq;
			maxDistSq = distSq;
		}

		// step 10.2
		double tempDistSq;
		IKDTreeDomain* tempNearest = NULL;
		if (furtherKd == NULL) {
			tempDistSq = Double_PositiveInfinity;
		} else {
			tempNearest = KDTree_NearestNeighbourI (furtherKd, target,
								furtherHr, maxDistSq, & tempDistSq, hrl);
		}

		// step 10.3
		if (tempDistSq < distSq) {
			nearest = tempNearest;
			distSq = tempDistSq;
		}
	}

	*resDistSq = distSq;
	return (nearest);
}

SortedLimitedList* KDTree_NearestNeighbourList (KDTree* self, IKDTreeDomain* target,
					double* resDist, int q)
{
	ArrayList* hrl = ArrayList_new0(HyperRectangle_delete);

	HyperRectangle* hr =
		HyperRectangle_CreateUniverseRectangle (IKDTreeDomain_GetDimensionCount(target));

	ArrayList_AddItem(hrl, hr);

	SortedLimitedList* best = SortedLimitedList_new (q, KDTreeBestEntry_delete);
	best->comparator.compareTo = (int ( *)(IComparator *,const void *,const void *))KDTreeBestEntry_CompareTo;

	/*IKDTreeDomain* nearest = */KDTree_NearestNeighbourListI (self, best, q, target, hr,
							       Double_PositiveInfinity, resDist, hrl);
	*resDist = sqrt (*resDist);

	int i;
	for(i=0; i<SortedLimitedList_Count(best); i++) {
		KDTreeBestEntry* be = (KDTreeBestEntry*) SortedLimitedList_GetItem(best, i);
		be->distance = sqrt (be->distance);
	}
       
	ArrayList_delete(hrl);

	return (best);
}


IKDTreeDomain* KDTree_NearestNeighbourListI (KDTree* self, SortedLimitedList* best,
		int q, IKDTreeDomain* target, HyperRectangle* hr, double maxDistSq,
		double* resDistSq, ArrayList* hrl)
{
	//WriteLine ("C NearestNeighbourI called");

	*resDistSq = Double_PositiveInfinity;

	IKDTreeDomain* pivot = self->dr;

	KDTreeBestEntry* be = KDTreeBestEntry_new (self->dr, KDTree_DistanceSq (target, self->dr));
	SortedLimitedList_AddItem (best, be);

	HyperRectangle* leftHr = hr;
	HyperRectangle* rightHr = HyperRectangle_SplitAt (leftHr, self->splitDim,
							  IKDTreeDomain_GetDimensionElement (pivot, self->splitDim));

	ArrayList_AddItem(hrl, rightHr);

	HyperRectangle* nearerHr = NULL;
	HyperRectangle* furtherHr = NULL;
	KDTree* nearerKd = NULL;
	KDTree* furtherKd = NULL;

	// step 5-7
	if (IKDTreeDomain_GetDimensionElement (target, self->splitDim) <=
	    IKDTreeDomain_GetDimensionElement (pivot, self->splitDim))
	{
		nearerKd = self->left;
		nearerHr = leftHr;
		furtherKd = self->right;
		furtherHr = rightHr;
	} else {
		nearerKd = self->right;
		nearerHr = rightHr;
		furtherKd = self->left;
		furtherHr = leftHr;
	}

	// step 8
	IKDTreeDomain* nearest = NULL;
	double distSq;

	// No child, bottom reached!
	if (nearerKd == NULL) {
		distSq = Double_PositiveInfinity;
	} else {
		nearest = KDTree_NearestNeighbourListI (nearerKd, best, q, target, nearerHr,
							maxDistSq, &distSq, hrl);
	}
	
	// step 9
	//maxDistSq = Math.Min (maxDistSq, distanceSq);
	if (SortedLimitedList_Count(best) >= q)
		maxDistSq = ((KDTreeBestEntry*) SortedLimitedList_GetItem(best, q - 1))->distance;
	else
		maxDistSq = Double_PositiveInfinity;

	// step 10
	if (HyperRectangle_IsInReach (furtherHr, target, sqrt (maxDistSq))) {
		double ptDistSq = KDTree_DistanceSq (pivot, target);
		if (ptDistSq < distSq) {
			// steps 10.1.1 to 10.1.3
			nearest = pivot;
			distSq = ptDistSq;

			// TODO: use k-element list
			/*
			  best.Add (new BestEntry (pivot, ptDistSq));
			  best.Sort ();
			*/

			maxDistSq = distSq;
		}

		// step 10.2
		double tempDistSq;
		IKDTreeDomain* tempNearest = NULL;
		if (furtherKd == NULL) {
			tempDistSq = Double_PositiveInfinity;
		} else {
			tempNearest = KDTree_NearestNeighbourListI (furtherKd, best, q, target,
								    furtherHr, maxDistSq, &tempDistSq, hrl);
		}

		// step 10.3
		if (tempDistSq < distSq) {
			nearest = tempNearest;
			distSq = tempDistSq;
		}
	}

	*resDistSq = distSq;
	return (nearest);
}
	
// Limited Best-Bin-First k-d-tree nearest neighbour search.
//
// (Using the algorithm described in the paper "Shape indexing using
// approximate nearest-neighbour search in high-dimensional spaces",
// available at http://www.cs.ubc.ca/spider/lowe/papers/cvpr97-abs.html)
//
// Find the approximate nearest neighbour to the hyperspace point 'target'
// within the kd-tree using 'searchSteps' tail recursions at most (each
// recursion deciding about one neighbours' fitness).
//
// After return 'resDist' contains the absolute distance of the
// approximate nearest neighbour from the target point. The approximate
// nearest neighbour is returned.
SortedLimitedList* KDTree_NearestNeighbourListBBF (KDTree* self, IKDTreeDomain* target,
					   int q, int searchSteps)
{
	ArrayList* hrl = ArrayList_new0(HyperRectangle_delete);

	HyperRectangle* hr =
		HyperRectangle_CreateUniverseRectangle (IKDTreeDomain_GetDimensionCount(target));
		
	ArrayList_AddItem(hrl, hr);

	SortedLimitedList* best = SortedLimitedList_new (q, KDTreeBestEntry_delete);
	best->comparator.compareTo = (int ( *)(IComparator *,const void *,const void *)) KDTreeBestEntry_CompareTo;
	SortedLimitedList* searchHr = SortedLimitedList_new (searchSteps, KDTreeHREntry_delete);
	searchHr->comparator.compareTo = (int ( *)(IComparator *,const void *,const void *))KDTreeHREntry_CompareTo;

	int dummyDist;
	/*IKDTreeDomain* nearest = */KDTree_NearestNeighbourListBBFI (self, best, q, target, hr,
								      INT_MAX, &dummyDist, searchHr, &searchSteps,
								      hrl);

	SortedLimitedList_delete(searchHr);

	int i;
	for(i=0; i<SortedLimitedList_Count(best); i++) {
		KDTreeBestEntry* be = (KDTreeBestEntry*) SortedLimitedList_GetItem(best, i);
		be->distance = sqrt ((double)be->distanceSq);
	}

	ArrayList_delete(hrl);

	return (best);
}


IKDTreeDomain* KDTree_NearestNeighbourListBBFI (KDTree* self, SortedLimitedList* best,
						int q, IKDTreeDomain* target, HyperRectangle* hr, int maxDistSq,
						int* resDistSq, SortedLimitedList* searchHr, int* searchSteps, 
						ArrayList* hrl)
{
	//WriteLine ("C NearestNeighbourI called");

	*resDistSq = INT_MAX;

	IKDTreeDomain* pivot = self->dr;

	KDTreeBestEntry* be = KDTreeBestEntry_new3 (self->dr, KDTree_DistanceSq (target, self->dr), true);
	SortedLimitedList_AddItem(best, be);

	HyperRectangle* leftHr = hr;
	HyperRectangle* rightHr = HyperRectangle_SplitAt (leftHr, self->splitDim,
							  IKDTreeDomain_GetDimensionElement (pivot, self->splitDim));

	ArrayList_AddItem(hrl, rightHr);

	HyperRectangle* nearerHr = NULL;
	HyperRectangle* furtherHr = NULL;
	KDTree* nearerKd = NULL;
	KDTree* furtherKd = NULL;

	// step 5-7
	if (IKDTreeDomain_GetDimensionElement (target, self->splitDim) <=
	    IKDTreeDomain_GetDimensionElement (pivot, self->splitDim))
	{
		nearerKd = self->left;
		nearerHr = leftHr;
		furtherKd = self->right;
		furtherHr = rightHr;
	} else {
		nearerKd = self->right;
		nearerHr = rightHr;
		furtherKd = self->left;
		furtherHr = leftHr;
	}

	// step 8
	IKDTreeDomain* nearest = NULL;
	int distSq;

	KDTreeHREntry* hre = KDTreeHREntry_new (furtherHr, furtherKd, pivot,
						HyperRectangle_Distance (furtherHr, target));
	SortedLimitedList_AddItem (searchHr, hre);

	// No child, bottom reached!
	if (nearerKd == NULL) {
		distSq = INT_MAX;
	} else {
		nearest = KDTree_NearestNeighbourListBBFI (nearerKd, best, q, target, nearerHr,
							   maxDistSq, &distSq, searchHr, searchSteps, hrl);
	}

	// step 9
	if (SortedLimitedList_Count(best) >= q) {
		maxDistSq = ((KDTreeBestEntry*) SortedLimitedList_GetItem(best,q - 1))->distanceSq;
	} else
		maxDistSq = INT_MAX;

	if (SortedLimitedList_Count(searchHr) > 0) {
		KDTreeHREntry* hre = (KDTreeHREntry*) SortedLimitedList_GetItem(searchHr, 0);
		SortedLimitedList_RemoveAt (searchHr, 0);

		furtherHr = hre->rect;
		furtherKd = hre->tree;
		pivot = hre->pivot;
		KDTreeHREntry_delete(hre);
	}

	// step 10
	*searchSteps -= 1;
	if (*searchSteps > 0 &&
	    HyperRectangle_IsInReach (furtherHr, target, sqrt ((double)maxDistSq)))
	{
		int ptDistSq = KDTree_DistanceSq (pivot, target);
		if (ptDistSq < distSq) {
			// steps 10.1.1 to 10.1.3
			nearest = pivot;
			distSq = ptDistSq;

			maxDistSq = distSq;
		}

		// step 10.2
		int tempDistSq;
		IKDTreeDomain* tempNearest = NULL;
		if (furtherKd == NULL) {
			tempDistSq = INT_MAX;
		} else {
			tempNearest = KDTree_NearestNeighbourListBBFI (furtherKd, best, q,
								       target, furtherHr, maxDistSq, &tempDistSq, searchHr,
								       searchSteps, hrl);
		}

		// step 10.3
		if (tempDistSq < distSq) {
			nearest = tempNearest;
			distSq = tempDistSq;
		}
	}

	*resDistSq = distSq;
	return (nearest);
}


int KDTree_DistanceSq (IKDTreeDomain* t1, IKDTreeDomain* t2)
{
	int distance = 0;

	int n;
	for ( n = 0 ; n < IKDTreeDomain_GetDimensionCount(t1) ; ++n) {
		int dimDist = IKDTreeDomain_GetDimensionElement (t1, n) -
			IKDTreeDomain_GetDimensionElement (t2, n);
		distance += dimDist * dimDist;
	}

	return (distance);
}

IKDTreeDomain* KDTree_GoodCandidate (ArrayList* exset, int* splitDim)
{
	IKDTreeDomain* first = (IKDTreeDomain*) ArrayList_GetItem(exset, 0);
	if (first == NULL) {
		FatalError ("Not of type IKDTreeDomain (TODO: custom exception)");
	}
	
	int dim = IKDTreeDomain_GetDimensionCount(first);
	
	// initialize temporary hr search min/max values
	double* minHr = (double*)malloc(sizeof(double)* dim);
	double* maxHr = (double*)malloc(sizeof(double)* dim);
	int i;
	for (i = 0 ; i < dim ; ++i) {
		minHr[i] = Double_PositiveInfinity;
		maxHr[i] = Double_NegativeInfinity;
	}

	int j;
	for(j=0; j<ArrayList_Count(exset); j++) {
		IKDTreeDomain* dom = (IKDTreeDomain*) ArrayList_GetItem(exset, j);
		int k;
		for (k = 0 ; k < dim ; ++k) {
			double dimE = IKDTreeDomain_GetDimensionElement (dom, k);
			
			if (dimE < minHr[k])
				minHr[k] = dimE;
			if (dimE > maxHr[k])
				maxHr[k] = dimE;
		}
	}

	// find the maximum range dimension
	double* diffHr = (double*)malloc(sizeof(double)* dim);
	int maxDiffDim = 0;
	double maxDiff = 0.0;

	int k;
	for (k = 0 ; k < dim ; ++k) {
		diffHr[k] = maxHr[k] - minHr[k];
		if (diffHr[k] > maxDiff) {
			maxDiff = diffHr[k];
			maxDiffDim = k;
		}
	}
	
	free(maxHr); maxHr = NULL;

	// the splitting dimension is maxDiffDim
	// now find a exemplar as close to the arithmetic middle as possible
	double middle = (maxDiff / 2.0) + minHr[maxDiffDim];
	IKDTreeDomain* exemplar = NULL;
	double exemMinDiff = Double_PositiveInfinity;
	
	free(minHr); minHr=NULL;

	int l;
	for(l=0; l<ArrayList_Count(exset); l++) {
		IKDTreeDomain* dom = (IKDTreeDomain*) ArrayList_GetItem(exset, l);
		double curDiff = abs (IKDTreeDomain_GetDimensionElement (dom, maxDiffDim) - middle);
		if (curDiff < exemMinDiff) {
			exemMinDiff = curDiff;
			exemplar = dom;
		}
	}

	free(diffHr); diffHr = NULL;

	// return the values
	*splitDim = maxDiffDim;
	
	return (exemplar);
}

// Build a kd-tree from a list of elements. All elements must implement
// the IKDTreeDomain interface and must have the same dimensionality.
KDTree* KDTree_CreateKDTree (ArrayList* exset)
{
	if (ArrayList_Count(exset) == 0)
		return (NULL);
	
	KDTree* cur = KDTree_new0 ();
	cur->dr = KDTree_GoodCandidate (exset, &cur->splitDim);

	ArrayList* leftElems = ArrayList_new0 (NULL);
	ArrayList* rightElems = ArrayList_new0 (NULL);
	
	// split the exemplar set into left/right elements relative to the
	// splitting dimension
	double bound = IKDTreeDomain_GetDimensionElement (cur->dr, cur->splitDim);
	int i;
	for(i=0; i<ArrayList_Count(exset); i++) {
		IKDTreeDomain* dom = (IKDTreeDomain*) ArrayList_GetItem(exset, i);
		// ignore the current element
		if (dom == cur->dr)
			continue;
		
		if (IKDTreeDomain_GetDimensionElement (dom, cur->splitDim) <= bound) {
			ArrayList_AddItem(leftElems, dom);
		} else {
			ArrayList_AddItem(rightElems, dom);
		}
	}

	// recurse
	cur->left = KDTree_CreateKDTree (leftElems);
	cur->right = KDTree_CreateKDTree (rightElems);

	ArrayList_delete(leftElems);
	ArrayList_delete(rightElems);
	
	return (cur);
}


