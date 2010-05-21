
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* MatchKeys.cs
 *
 * Keypoint file correlation functionality.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 *
 * "The University of British Columbia has applied for a patent on the SIFT
 * algorithm in the United States. Commercial applications of this software
 * may require a license from the University of British Columbia."
 * For more information, see the LICENSE file supplied with the distribution.
 */

#include "AutoPanoSift.h"

ArrayList* MatchKeys_FindMatchesBBF (ArrayList* keys1, ArrayList* keys2)
{
	// TODO: swap so smaller list is searched.
    
	ArrayList* al = ArrayList_new0 (NULL);
	int i;
	for (i=0; i<ArrayList_Count(keys2); i++)
		ArrayList_AddItem (al, ArrayList_GetItem(keys2, i));
	
	KDTree* kd = KDTree_CreateKDTree (al);

	ArrayList_delete(al);

	ArrayList* matches = ArrayList_new0 (Match_delete);
	int j;
	for (j=0; j<ArrayList_Count(keys1); j++) {
		KeypointN* kp = (KeypointN*) ArrayList_GetItem(keys1, j);
		ArrayList* kpNNList = (ArrayList*)KDTree_NearestNeighbourListBBF (kd, (IKDTreeDomain*)kp, 2, 40);
		
		if (ArrayList_Count(kpNNList) < 2)
			FatalError ("BUG: less than two neighbours!");

		KDTreeBestEntry* be1 = (KDTreeBestEntry*) ArrayList_GetItem(kpNNList, 0);
		KDTreeBestEntry* be2 = (KDTreeBestEntry*) ArrayList_GetItem(kpNNList, 1);

		if ((be1->distance / be2->distance) > 0.6)
			continue;
		
		KeypointN* kpN = (KeypointN*)KDTreeBestEntry_Neighbour(be1);
		
		ArrayList_AddItem(matches, Match_new (kp, kpN, be1->distance, be2->distance));

		/*
		WriteLine ("(%d,%d) (%d,%d) %d, 2nd: %d", (int)(kp->x + 0.5),
			   (int)(kp->y + 0.5), (int)(kpN->x + 0.5),
			   (int)(kpN->y + 0.5), be1->distance, be2->distance);
		*/
	}
	
	return (matches);
}


ArrayList* MatchKeys_FilterJoins (ArrayList* matches)
{
	HashTable* ht = HashTable_new0 (NULL, NULL);
	
	// Count the references to each keypoint
	int i;
	for(i=0; i<ArrayList_Count(matches); i++) {
		Match* m = (Match*) ArrayList_GetItem(matches, i);
		int lI = (HashTable_GetItem(ht, m->kp1) == NULL) ? 0 : (int) HashTable_GetItem(ht, m->kp1);
		HashTable_SetItem(ht, m->kp1, (void*)(lI + 1));
		int rI = (HashTable_GetItem(ht, m->kp2) == NULL) ? 0 : (int) HashTable_GetItem(ht, m->kp2);
		HashTable_SetItem(ht, m->kp2, (void*)(rI + 1));
	}

	ArrayList* survivors = ArrayList_new0(NULL);
	int removed = 0;
	int j;
	for(j=0; j<ArrayList_Count(matches); j++) {
		Match* m = (Match*) ArrayList_GetItem(matches, j);
		//WriteLine ("match: %d, %d", (int) HashTable_GetItem(ht, m->kp1), (int) HashTable_GetItem(ht, m->kp2));
		
		if (((int) HashTable_GetItem(ht, m->kp1)) <= 1 && ((int) HashTable_GetItem(ht, m->kp2)) <= 1)
			ArrayList_AddItem(survivors, m);
		else
			removed += 1;
	}
	
	HashTable_delete(ht);

	return (survivors);
}

void MatchKeys_FilterNBest (ArrayList* matches, int bestQ)
{
	MatchWeighter* weighter = MatchWeighter_new0 ();
	ArrayList_Sort (matches, &weighter->comparator);
	MatchWeighter_delete(weighter);
	if (ArrayList_Count(matches) > bestQ)
		ArrayList_RemoveRange(matches, bestQ, ArrayList_Count(matches) - bestQ);
}


Match* Match_new0 ()
{
    Match* self = (Match*)malloc(sizeof(Match));
    self->kp1 = NULL;
    self->kp2 = NULL;
    return self;
}

// dist1: distance between kp1/kp2,
// dist2: distance between kp1 and kp3, where kp3 is the next closest
//   match
Match* Match_new (KeypointN* kp1, KeypointN* kp2, double dist1, double dist2)
{
	Match* self = Match_new0();
	self->kp1 = kp1;
	self->kp2 = kp2;
	self->dist1 = dist1;
	self->dist2 = dist2;
	return self;
}

void Match_delete (Match* self)
{
	if (self) {
		//KeypointN_delete(self->kp1);
		//KeypointN_delete(self->kp2);
		free(self);
	}
}

Match* Match_clone (Match* self)
{
	Match* mc = Match_new (KeypointN_clone(self->kp1), KeypointN_clone(self->kp2), self->dist1, self->dist2);
	return (mc);
}


MatchWeighter* MatchWeighter_new0()
{
	MatchWeighter* self = (MatchWeighter*)malloc(sizeof(MatchWeighter));
	self->comparator.compareTo = (int ( *)(IComparator *,const void *,const void *)) MatchWeighter_CompareTo;
	self->distExp = 1.0;
	self->quotExp = 1.0;
	return self;
}

void MatchWeighter_delete(MatchWeighter* self)
{
	if (self) {
		free(self);
	}
}

// The formula goes like this, with lowest weight being best matches:
// w(kp) = kp.dist1^{distExp} *
//     {\frac{1}{kp.dist2 - kp.dist1}}^{quotExp}
//
// This means, as both dist1 and dist2 are in [0.0 ; 1.0], that a high
// distExp exponent (and distExp > quotExp) will make the absolute
// distance for the best match more important. A high value for
// quotExp will make the difference between the best and second best
// match more important (as in "how many other candidates are likely
// matches?").
MatchWeighter* MatchWeighter_new(double distExp, double quotExp)
{
	MatchWeighter* self = MatchWeighter_new0();
	self->distExp = distExp;
	self->quotExp = quotExp;
	return self;
}

int MatchWeighter_CompareTo (MatchWeighter* self, Match* m1, Match* m2)
{
	double fit1 = MatchWeighter_OverallFitness (self, m1);
	double fit2 = MatchWeighter_OverallFitness (self, m2);
	if (fit1 < fit2)
		return (-1);
	else if (fit1 > fit2)
		return (1);
	
	return (0);
}

double MatchWeighter_OverallFitness (MatchWeighter* self, Match* m)
{
	double fitness = pow (m->dist1, self->distExp) *
		pow (1.0 / (m->dist2 - m->dist1), self->quotExp);
	
	return (fitness);
}

MultiMatch* MultiMatch_new0()
{
	MultiMatch* self = (MultiMatch*)malloc(sizeof(MultiMatch));
	self->keySets = NULL;
	self->globalKeyKD = NULL;
	self->globalKeys = ArrayList_new0(NULL);
	self->globalMatches = ArrayList_new0(Match_delete);
	self->matchSets = NULL;
	self->filteredMatchSets = NULL;
	self->verbose = true;
	return self;
}

void MultiMatch_delete(MultiMatch* self) 
{
	if (self) {
		ArrayList_delete(self->keySets);
		KDTree_delete(self->globalKeyKD);
		ArrayList_delete(self->globalKeys);
		ArrayList_delete(self->globalMatches);
		PtrMap_delete(self->matchSets, NULL);
		ArrayList_delete(self->filteredMatchSets);
		free(self);
	}
}

void MultiMatch_LoadKeysetsFromMemory (MultiMatch* self, ArrayList* memlist)
{
	self->imageCount = ArrayList_Count(memlist);
	self->keySets = memlist;
}

void MultiMatch_LoadKeysets(MultiMatch* self, ArrayList* filenames)
{
	self->imageCount = ArrayList_Count(filenames);
	
	self->keySets = ArrayList_new(self->imageCount, KeypointXMLList_delete);
	int n;
	for ( n = 0 ; n < self->imageCount ; ++n) {
	        char* fn = (char *) ArrayList_GetItem(filenames, n);
		KeypointXMLList* keys = KeypointXMLReader_ReadComplete (fn);
		if (!keys)
			FatalError("Failed to load keypoints from %s", fn);
		WriteLine ("Loaded %d keypoints from %s for image \"%s\" (%dx%d)",
			   ArrayList_Count(keys->array),
			   fn,
			   keys->imageFile,
			   keys->xDim, keys->yDim);
		
		ArrayList_SetItem(self->keySets, n, keys);
	}
}

ArrayList* MatchSet_GetOriginalKeys (MatchSet* self, int which) 
{
	if (which == 0)
		return (self->keys1->array);
	else
		return (self->keys2->array);
}


MatchSet* MatchSet_new0(MultiMatch* parent)
{
	MatchSet* self = (MatchSet*)malloc(sizeof(MatchSet));
	self->parent = parent;
	self->file1 = NULL;
	self->file2 = NULL;
	self->matches = NULL;
	self->bestMatchFit = NULL;
	self->keys1 = NULL;
	self->keys2 = NULL;
	return self;
}

void MatchSet_delete(MatchSet* self)
{
    if (self) {
	ArrayList_delete(self->matches);
	KeypointXMLList_delete(self->keys1);
	KeypointXMLList_delete(self->keys2);
	free(self);
    }
}

MatchSet* MatchSet_new(MultiMatch* parent,
		       char* file1, int xdim1, int ydim1,
		       char* file2, int xdim2, int ydim2,
		       KeypointXMLList* kp1, KeypointXMLList* kp2)
{
	MatchSet* self = MatchSet_new0(parent);
	self->file1 = file1;
	self->file2 = file2;
	self->matches = ArrayList_new0 (Match_delete);
	
	self->xDim1 = xdim1;
	self->yDim1 = ydim1;
	self->xDim2 = xdim2;
	self->yDim2 = ydim2;

	self->keys1 = kp1;
	self->keys2 = kp2;
	return self;
}

ArrayList* MultiMatch_TwoPatchMatch (MultiMatch* self, ArrayList* kp1, int kp1maxX, int kp1maxY,
				   ArrayList* kp2, int kp2maxX, int kp2maxY, bool useRANSAC)
{
	// Initialize keysets
	ArrayList* kl = ArrayList_new0 (NULL);
	ArrayList_AddItem (kl, KeypointXMLList_new2 (kp1, kp1maxX, kp1maxY));
	ArrayList_AddItem (kl, KeypointXMLList_new2 (kp2, kp2maxX, kp2maxY));
	MultiMatch_LoadKeysetsFromMemory (self, kl);

	// Build search structure
	MultiMatch_BuildGlobalKD (self);
	MultiMatch_BuildGlobalMatchList (self);
	
	MultiMatch_PartitionMatches (self);
	
	self->filteredMatchSets = ArrayList_new0 (NULL);
	
	// The only combination between two matches
	MatchSet* ms = (MatchSet*) self->matchSets[0][1];
	if (ms == NULL || ms->matches == NULL)
		return (NULL);

	if (useRANSAC) {
		ArrayList* ransacMatches = NULL;
		int maxX = ms->xDim1 >= ms->xDim2 ? ms->xDim1 : ms->xDim2;
		int maxY = ms->yDim1 >= ms->yDim2 ? ms->yDim1 : ms->yDim2;

		// TODO: 16.0 -> configurable
		ImageMatchModel* bestRansac = MatchDriver_FilterMatchSet
			(ms->matches, 16.0, maxX, maxY);

		ms->bestMatchFit = bestRansac;
		if (bestRansac != NULL)
			ransacMatches = bestRansac->fittingGround;

		if (ransacMatches == NULL) {
			if (self->verbose)
				WriteLine ("RANSAC says: no good matches from " 
					   "%d original matches", ArrayList_Count(ms->matches));

			return (NULL);
		} else {
			if (self->verbose)
				WriteLine ("RANSAC purged: %d to %d",
					   ArrayList_Count(ms->matches), ArrayList_Count(ransacMatches));
		}

		// Overwrite matches with RANSAC checked ones.
		ms->matches = ransacMatches;
	}
	ArrayList_AddItem(self->filteredMatchSets, ms);

	return (self->filteredMatchSets);
}

// Find and return a list of MatchSet objects.
//
// This is the heart of the matching process.
//
// minimumMatches: minimum number of matches required in final results.
// bestMatches: number of best matches to keep, or zero to keep all.
// useRANSAC: whether ransac filtering should be used (true recommended
//     for 2d rigid transformed images, such as panoramas).
ArrayList* MultiMatch_LocateMatchSets (MultiMatch* self, 
				     int minimumMatches, int bestMatches,
				     bool useRANSAC, bool useAreaFiltration)
{
// mod 06 Mar 2008 TKS to allow alternative primary matchers
	if( ArrayList_Count( self->globalMatches ) == 0 ) {
		MultiMatch_BuildGlobalKD (self);
		MultiMatch_BuildGlobalMatchList (self);
	}

	MultiMatch_PartitionMatches (self);

	self->filteredMatchSets = ArrayList_new0 (NULL);

	// Walk all image combinations.
	int n0;
	for ( n0 = 0 ; n0 < self->imageCount ; ++n0) {
		int n1;
		for ( n1 = n0 + 1 ; n1 < self->imageCount ; ++n1) {
			MatchSet* ms = (MatchSet*) self->matchSets[n0][n1];
			if (ms == NULL || ms->matches == NULL)
				continue;
			
			if (useRANSAC) {
				ArrayList* ransacMatches = NULL;
				int maxX = ms->xDim1 >= ms->xDim2 ? ms->xDim1 : ms->xDim2;
				int maxY = ms->yDim1 >= ms->yDim2 ? ms->yDim1 : ms->yDim2;
				
				// TODO: 16.0 -> configurable
				ImageMatchModel* bestRansac = MatchDriver_FilterMatchSet
					(ms->matches, 16.0, maxX, maxY);
				
				ms->bestMatchFit = bestRansac;
				if (bestRansac != NULL)
					ransacMatches = bestRansac->fittingGround;

				if (ransacMatches == NULL) {
					if (self->verbose)
						WriteLine ("RANSAC says: no good matches from " 
							   "%d original matches", ArrayList_Count(ms->matches));

					continue;
				} else {
					if (self->verbose)
						WriteLine ("RANSAC purged: %d to %d",
							   ArrayList_Count(ms->matches), ArrayList_Count(ransacMatches));
				}
				
				// Overwrite matches with RANSAC checked ones.
				ms->matches = ransacMatches;
			}

			// Number of RANSAC-ok matches before count-filtering.
			int beforeBestFilterCount = ArrayList_Count(ms->matches);
			
			// TODO: replace with area filtering
			WriteLine ("Filtering... (%s, %s)",
				   ms->file1,
				   ms->file2);
			
			// First filtration: Join matches
			int beforeJoinFiltering = ArrayList_Count(ms->matches);
			ms->matches = MatchKeys_FilterJoins (ms->matches);
			WriteLine ("   A. Join Filtration: %d to %d",
				   beforeJoinFiltering, ArrayList_Count(ms->matches));
			
#if 0
			// Second: Area filtration
			if (useAreaFiltration && bestMatches > 0) {
				int beforeAreaFiltering = ms.Matches.Count;
				double pixelCount = ms.xDim1 * ms.yDim1;
				double areaPixels;
				ms.Matches = FilterMatchesByArea (ms.Matches, bestMatches,
								  out areaPixels);

				Console.WriteLine ("   B. Area Filtration: {0} to {1}, covering {2:N2} % of image \"{3}\"",
						   beforeAreaFiltering, ms.Matches.Count,
						   (areaPixels * 100.0) / pixelCount,
						   System.IO.Path.GetFileName (ms.File1));
			}
#endif
#if 0
			if (useAreaFiltration && bestMatches > 0) {
				int beforeFiltering = ms.Matches.Count;
				
				ms.Matches = FilterMatchesByCoverage (ms, ms.Matches, bestMatches);
				
				Console.WriteLine ("   B. Coverage Filtration: {0} to {1} on image \"{2}\"",
						   beforeFiltering, ms.Matches.Count,
						   System.IO.Path.GetFileName (ms.File1));
			} else
#endif
				if (bestMatches > 0) {
					int beforeScoreFiltering = ArrayList_Count(ms->matches);
					ms->matches = MultiMatch_FilterMatches (self, ms->matches, bestMatches);
				
					WriteLine ("   B. Score Filtration: %d to %d",
						   beforeScoreFiltering, ArrayList_Count(ms->matches));
				}
			
			WriteLine ("Filtered partition [%d,%d] from %d matches down to %d",
				   n0, n1, beforeBestFilterCount, ArrayList_Count(ms->matches));
			
			if (ArrayList_Count(ms->matches) >= minimumMatches)
				ArrayList_AddItem(self->filteredMatchSets, ms);
		}
	}
	
	return (self->filteredMatchSets);
}

ArrayList* MultiMatch_CreatePointList (MultiMatch* self, ArrayList* matches)
{
	ArrayList* points = ArrayList_new0 (FilterPoint_delete);

	// Create a point list.
	int i;
	for(i=0; i<ArrayList_Count(matches); i++) {
		Match* m = (Match*) ArrayList_GetItem(matches, i);
		FilterPoint* p = FilterPoint_new0 ();

		p->x = m->kp1->x;
		p->y = m->kp1->y;
		p->user = m;

		WriteLine ("%f %f # CPOINTS", p->x, p->y);
		ArrayList_AddItem(points, p);
	}
	
	return (points);
}

// This was a planned attempt to use a more complex FMM based filter to
// prune keypoints in a way to maximize coverage.  Failed, tho.
#if 0
		private ArrayList FilterMatchesByCoverage (MatchSet ms,
					   ArrayList matches, int bestMatches)
		{
			// Nothing to do.
			if (matches.Count <= bestMatches)
				return (matches);

			ArrayList remainingPoints = CreatePointList (matches);
			foreach (FilterPoint p in remainingPoints)
				Console.WriteLine ("{0} {1} # INPUT", p.x, p.y);

			/* Prune them.
			 */
			while (remainingPoints.Count > bestMatches) {
				CoverageFilter cf = new CoverageFilter (ms, remainingPoints, bestMatches);
				remainingPoints = cf.PruneOne ();
				Console.WriteLine ("Pruned one point, {0}.  Goal: {1}.",
						   remainingPoints.Count, bestMatches);
			}

			foreach (FilterPoint p in remainingPoints)
				Console.WriteLine ("{0} {1} # OUTPUT", p.x, p.y);

			/*double[] pruneScores = cf.GetScore ();
			  for (int k = 0 ; k < pruneScores.Length ; ++k)
			  Console.WriteLine ("{0}: {1}", k, pruneScores[k]);*/

			/* Restore the remaining points to match points.
			 */
			ArrayList filteredMatches = new ArrayList ();
			foreach (FilterPoint p in remainingPoints)
				filteredMatches.Add ((Match) p.user);

			return (filteredMatches);
		}
#endif

ArrayList* MultiMatch_FilterMatchesByArea (MultiMatch* self, ArrayList* matches, int bestMatches,
					 double* areaPixels)
{
	*areaPixels = 0.0;
	
	// Trivial case: no more matches available.
	if (ArrayList_Count(matches) <= bestMatches)
		return (matches);
	
	ArrayList* points = MultiMatch_CreatePointList (self, matches);

	AreaFilter* areaF = AreaFilter_new0 ();
	ArrayList* convexHull = AreaFilter_CreateConvexHull(areaF, points);
	//WriteLine ("convex hull holds %d points", ArrayList_Count(convexHull));

	// Case one: we have more points in the convex hull than we want
	// Solution: Iteratively remove the point that reduces the hull area
	//   the least, until we have only bestMatches left.
	//WriteLine ("Polygon area before: %f", AreaFilter_PolygonArea(areaF, convexHull));
	while (ArrayList_Count(convexHull) > bestMatches) {
		double maxArea = -1.0;
		int removeIndex = -1;

		// Remove exactly one element from the convex hull, that element
		// which still maximizes the polygon area (ie image coverage).
		int n;
		for (n = 0 ; n < ArrayList_Count(convexHull) ; ++n) {
			ArrayList* convexHullMinusOne = ArrayList_clone(convexHull);
			ArrayList_RemoveAt(convexHullMinusOne, n);

			double remArea = AreaFilter_PolygonArea (areaF, convexHullMinusOne);
			if (removeIndex < 0 || remArea > maxArea) {
				removeIndex = n;
				maxArea = remArea;
			}
		}

		//WriteLine ("DEBUG Removing %d, area still %f", removeIndex, maxArea);
		ArrayList_RemoveAt(convexHull, removeIndex);
	}

	//WriteLine ("Polygon area after: %f", AreaFilter_PolygonArea(areaF, convexHull));
	*areaPixels = AreaFilter_PolygonArea(areaF, convexHull);

	// Case two: we have less points in the convex hull than we want.
	// Solution: Add points based on their average distance to all convex
	//   hull points.
	//
	// We know there are enough matches available as
	// matches.Count >= bestMatches.
	while (ArrayList_Count(convexHull) < bestMatches)
	{
		double maxDistance = -1.0;
		Match* addMatch = NULL;
		
		int i;
		for(i=0; i<ArrayList_Count(matches); i++) {
			Match* m = (Match*) ArrayList_GetItem(matches, i);
			bool matchFound = false;
			int j;
			for(j=0; j<ArrayList_Count(convexHull); j++) {
				FilterPoint* p = (FilterPoint*) ArrayList_GetItem(convexHull, j);
				if (p->user != m)
					continue;
				
				matchFound = true;
				break;
			}
			
			// Match already in pointlist.
			if (matchFound)
				continue;
			
			// Now that we have a unique point, calculate its average
			// distance to all points in the convex hull.
			double dist = 0.0;
			int distCount = 0;

			int k;
			for(k=0; k<ArrayList_Count(convexHull); k++) {
				FilterPoint* p = (FilterPoint*) ArrayList_GetItem(convexHull, k);
				dist += sqrt (pow (p->x - m->kp1->x, 2.0) +
					      pow (p->y - m->kp1->y, 2.0));
				distCount += 1;
			}

			dist /= (double) distCount;
			//WriteLine ("  max: %f, this: %f", maxDistance, dist);
			if (addMatch == NULL || dist > maxDistance) {
				addMatch = m;
				maxDistance = dist;
			}
		}
		
		// Add point, although its not in the hull. It just happens to be
		// farthest away from all points in the hull, so the image
		// coverage is improved most.
		FilterPoint* pNew = FilterPoint_new0 ();
		pNew->x = addMatch->kp1->x;
		pNew->y = addMatch->kp1->y;
		pNew->user = addMatch;
		ArrayList_AddItem(convexHull, pNew);
	}

	ArrayList* filteredMatches = ArrayList_new0 (NULL);
	int j;
	for(j=0; j<ArrayList_Count(convexHull); j++) {
		FilterPoint* p = (FilterPoint*) ArrayList_GetItem(convexHull, j);
		ArrayList_AddItem(filteredMatches, p->user);
	}
		
	return (filteredMatches);
}

ArrayList* MultiMatch_FilterMatches (MultiMatch* self, ArrayList* matches, int bestMatches)
{
	matches = MatchKeys_FilterJoins (matches);
	MatchKeys_FilterNBest (matches, bestMatches);
	
	/*
	  Match.MatchWeighter mw = new Match.MatchWeighter ();
	  
	  foreach (Match m in matches) {
	  Console.WriteLine ("    ({0},{1}) ({2},{3}) {4}, 2nd: {5}, weight {6}",
	  (int)(m.Kp1.X + 0.5), (int)(m.Kp1.Y + 0.5),
	  (int)(m.Kp2.X + 0.5), (int)(m.Kp2.Y + 0.5),
	  m.Dist1, m.Dist2, mw.OverallFitness (m));
	  }*/
	
	return (matches);
}

void MultiMatch_BuildGlobalKD (MultiMatch* self)
{
	int i;
	for(i=0; i<ArrayList_Count(self->keySets); i++) {
		KeypointXMLList* list = (KeypointXMLList*) ArrayList_GetItem(self->keySets, i);
		int j;
		for(j=0; j<ArrayList_Count(list->array); j++) {
			KeypointN* kp = (KeypointN*) ArrayList_GetItem(list->array, j);
			ArrayList_AddItem(self->globalKeys, kp);
		}
	}
	
	self->globalKeyKD = KDTree_CreateKDTree (self->globalKeys);
	
	if (self->verbose)
		WriteLine ("Created global k-d tree containing %d keypoints",
			   ArrayList_Count(self->globalKeys));
}

// Partition the matches into image pair groups
void MultiMatch_PartitionMatches (MultiMatch* self)
{
	self->matchSets = PtrMap_new(self->imageCount, self->imageCount);
	int createCount = 0;
	int maxL = 0;
	
	// Create all possible partition combinations, while pruning reverse
	// matches
	int i;
	for(i=0; i<ArrayList_Count(self->globalMatches); i++) {
		Match* m = (Match*) ArrayList_GetItem(self->globalMatches, i);
		int l0 = MultiMatch_FindOrigin (self, m->kp1);
		int l1 = MultiMatch_FindOrigin (self, m->kp2);
		
		if (l0 > maxL)
			maxL = l0;
		if (l1 > maxL)
			maxL = l1;
		bool reverseAlreadyIn = false;
		
		// FIXME: remove/is this correct?
		// FIXME: rewrite this whole crappy function
		if (l0 >= l1)
			continue;
		
		if (self->matchSets[l1][l0] != NULL) {
			MatchSet* rev = (MatchSet*) self->matchSets[l1][l0];
			if (rev != NULL) {
				int i;
				for(i=0; i<ArrayList_Count(rev->matches); i++) {
					Match* mr = (Match*) ArrayList_GetItem(rev->matches, i);
					if (mr->kp1 == m->kp2 && mr->kp2 == m->kp1) {
						reverseAlreadyIn = true;
						
						break;
					}
				}
			}
		}
		if (reverseAlreadyIn)
			continue;
		
		if (self->matchSets[l0][l1] == NULL) {
			createCount += 1;
			KeypointXMLList* k0 = (KeypointXMLList*) ArrayList_GetItem(self->keySets, l0);
			KeypointXMLList* k1 = (KeypointXMLList*) ArrayList_GetItem(self->keySets, l1);
			self->matchSets[l0][l1] = MatchSet_new (self,
								k0->imageFile, k0->xDim, k0->yDim,
								k1->imageFile, k1->xDim, k1->yDim,
								k0, k1);
		}
		
		ArrayList_AddItem(((MatchSet*)self->matchSets[l0][l1])->matches, m);
	}
	
	if (self->verbose) {
		WriteLine ("Created %d match partitions, max l = %d",
			   createCount, maxL);
		
		/*
		  for (int l0 = 0 ; l0 <= maxL ; ++l0) {
		  for (int l1 = 0 ; l1 <= maxL ; ++l1) {
		  Console.Write ("({0},{1}: {2}), ", l0, l1,
		  matchSets[l0, l1] == null ? "empty" :
		  String.Format ("{0}", matchSets[l0, l1].Matches.Count));
		  }
		  }
		*/
		
		WriteLine ("");
	}
}

int MultiMatch_FindOrigin (MultiMatch* self, KeypointN* kp)
{
	int ksn;
	for ( ksn = 0 ; ksn < ArrayList_Count(self->keySets) ; ++ksn) {
		KeypointXMLList* list = (KeypointXMLList*) ArrayList_GetItem(self->keySets, ksn);
		
		int lIdx = ArrayList_IndexOf (list->array, kp);
		if (lIdx >= 0)
			return (ksn);
	}
	
	FatalError ("BUG: keypoint origin unknown");
	return -1;
}


void MultiMatch_BuildGlobalMatchList (MultiMatch* self)
{
	int count = 0;
	double searchDepth = max (130.0,
				  (log ((double) ArrayList_Count(self->globalKeys)) / log (1000.0)) * 130.0);
	int searchDepthI = (int) searchDepth;
	
	if (self->verbose)
		WriteLine ("Using a BBF cut search depth of %d", searchDepthI);

	int i;
	for(i=0; i<ArrayList_Count(self->globalKeys); i++) {
		KeypointN* kp = (KeypointN*) ArrayList_GetItem(self->globalKeys, i);

		if (self->verbose) {
			if ((count % 250) == 0)
				Write ("\r%2.2f%%, %d/%d        ",
					   (100 * ((double) count)) / ((double) ArrayList_Count(self->globalKeys)),
					   count, ArrayList_Count(self->globalKeys));
		}
		count++;
		
		// There should be one exact hit (the keypoint itself, which we
		// ignore) and two real hits, so lets search for the three best
		// hits. But it could be the exact match is not found for, as we
		// use probabilistic bbf matching.
		ArrayList* kpNNList =
			(ArrayList*)KDTree_NearestNeighbourListBBF(self->globalKeyKD, (IKDTreeDomain*)kp, 3, searchDepthI);
		
		if (ArrayList_Count(kpNNList) < 3)
			FatalError ("BUG: less than three neighbours!");

		KDTreeBestEntry* be1 = (KDTreeBestEntry*) ArrayList_GetItem(kpNNList, 0);
		KDTreeBestEntry* be2 = (KDTreeBestEntry*) ArrayList_GetItem(kpNNList, 1);

		// If be1 is the same (exact hit), shift one
		if (be1->neighbour == (IKDTreeDomain*)kp) {
			be1 = be2;
			be2 = (KDTreeBestEntry*) ArrayList_GetItem(kpNNList, 2);
		}
		if (be1->neighbour == (IKDTreeDomain*)kp || be2->neighbour == (IKDTreeDomain*)kp ||
		    be1->neighbour == (IKDTreeDomain*) be2->neighbour) {
			ArrayList_delete(kpNNList);
			continue;
		}
		//FatalError ("BUG: wrong keypoints caught");


		if ((be1->distance / be2->distance) > 0.6) {
			ArrayList_delete(kpNNList);
			continue;
		}

		ArrayList_AddItem(self->globalMatches, Match_new (kp, (KeypointN*)(be1->neighbour),
							      be1->distance, be2->distance));
		ArrayList_delete(kpNNList);
	}
	if (self->verbose) {
		Write ("\r %2.2f%%, %d/%d        ",
		       (100 * ((double) count)) / ((double) ArrayList_Count(self->globalKeys)),
		       count, ArrayList_Count(self->globalKeys));
		
		WriteLine ("\nGlobal match search yielded %d matches",
			   ArrayList_Count(self->globalMatches));
	}
}

// matches: ArrayList of MatchSet
// returns: ArrayList of ArrayList of string (filenames)
ArrayList* MultiMatch_ComponentCheck (MultiMatch* self, ArrayList* matches)
{
	ArrayList* components = ArrayList_new0 (Component_delete);

	int i;
	for(i=0; i<ArrayList_Count(matches); i++) {
		MatchSet* ms = (MatchSet*) ArrayList_GetItem(matches, i);

		Component* c1 = MultiMatch_FindComponent (components, ms->file1);
		Component* c2 = MultiMatch_FindComponent (components, ms->file2);

		// case: new component
		if (c1 == NULL && c2 == NULL) {
			Component* comp = Component_new0 ();

			Component_AddFile(comp, ms->file1);
			Component_AddFile(comp, ms->file2);

			ArrayList_AddItem(components, comp);
			// c2 is new in c1-component
		} else if (c1 != NULL && c2 == NULL) {
			Component_AddFile(c1, ms->file2);
			// c1 is new in c2-component
		} else if (c1 == NULL && c2 != NULL) {
			Component_AddFile(c2, ms->file1);
			// same component already, do nothing
		} else if (c1 == c2) {
			// different component: join components
		} else if (c1 != c2) {
			Component_AddComponent(c1, c2);
			ArrayList_RemoveItem(components, c2);
		}
	}
	
	// Now locate all components with no matches at all and add them to
	// the final result.
	int j;
	for(j=0; j<ArrayList_Count(self->keySets); j++) {
		KeypointXMLList* klist = (KeypointXMLList*) ArrayList_GetItem(self->keySets, j);
		char* filename = klist->imageFile;

		if (MultiMatch_FindComponent (components, filename) != NULL)
			continue;
		
		Component* isolatedFile = Component_new0 ();
		Component_AddFile(isolatedFile, filename);
		ArrayList_AddItem(components, isolatedFile);
	}
	
	return (components);
}


Component* MultiMatch_FindComponent (ArrayList* components, char* filename)
{
	int i;
	for(i=0; i<ArrayList_Count(components); i++) {
		Component* comp = (Component*) ArrayList_GetItem(components, i);
		if (Component_IsIncluded(comp, filename)) 
			return (comp);
	}
	
	return (NULL);
}

Component* Component_new0() 
{
	Component* self = (Component*)malloc(sizeof(Component));
	self->files = ArrayList_new0(NULL);
	return self;
}

void Component_delete(Component* self)
{
	if (self) {
		ArrayList_delete(self->files);
		free(self);
	}
}


void Component_AddComponent(Component* self, Component* comp)
{
	int i;
	for(i=0; i<ArrayList_Count(comp->files); i++) {
		char* filename = (char*) ArrayList_GetItem(comp->files, i);
		ArrayList_AddItem(self->files, filename);
	}
}

bool Component_IsIncluded (Component* self, char* filename)
{
	return ArrayList_Contains(self->files, filename);
}

void Component_AddFile (Component* self, char* filename)
{
	if (Component_IsIncluded (self, filename))
		return;
	
	ArrayList_AddItem (self->files, filename);
}

// fixed by Seb Perez D
char* Component_ToString (Component* self)
{
	char *buffer;
	
	bool first = true;
	int cCount = 0;
	int i;
    int buflen=256;

    buffer = (char *) malloc(buflen);
    strcpy(buffer,"");

	for(i=0; i<ArrayList_Count(self->files); i++) {
		char* filename = (char*) ArrayList_GetItem(self->files, i);
		if (first) {
			first = false;
		} else {
            if (strlen(buffer) + 2 + 1> buflen) {
                buflen += buflen;
                buffer = (char*) realloc(buffer, buflen);
            }
			strcat(buffer, ", ");
			cCount += 2;
		}
		
		char* justFile = FullPathToFileName(filename);
        if (strlen(buffer) + strlen(justFile) + 1 > buflen -1) {
            if (buflen + 1 < strlen(justFile) ) buflen += strlen(justFile) + 1;
            else buflen += buflen;
            buffer = (char*) realloc(buffer, buflen);
        }
        strcat(buffer, justFile);
		cCount += strlen(justFile);
		free(justFile);
		
		if (cCount > 65) {
			strcat(buffer, "\n  ");
			cCount = 2;
			first = true;
		}
	}
	
    return buffer;
}


BondBall* MultiMatch_BuildBondBall (MultiMatch* self, ArrayList* ransacFiltered, int bottomDefault)
{
	BondBall* bb = NULL;
	bool first = true;
	char* fileNow;
	char* fileNext;

	int fileN;
	for ( fileN = 0 ; fileN < (ArrayList_Count(self->keySets) - 1) ; ++fileN) {
		fileNow = ((KeypointXMLList*)ArrayList_GetItem(self->keySets,fileN))->imageFile;
		fileNext = ((KeypointXMLList*)ArrayList_GetItem(self->keySets, fileN + 1))->imageFile;
		
		WriteLine ("Searching for matches between %s-%s",
			   fileNow, fileNext);
		// Process only the MatchSet that is build from this exact two
		// image files.
		MatchSet* msNext = NULL;
		int i;
		for(i=0; i<ArrayList_Count(ransacFiltered); i++) {
			MatchSet* ms = (MatchSet*) ArrayList_GetItem(ransacFiltered, i);
			if (strcmp (ms->file1, fileNow) == 0 &&
			    strcmp (ms->file2, fileNext) == 0)
			{
				msNext = ms;
				WriteLine ("  found!");
				break;
			}
		}

		// In case no matchset can be found this means we reached the end
		// of the panorama (it could still be a 360 degree one).
		if (msNext == NULL) {
			WriteLine ("  NOT found!");
			break;
		}
		
		if (first) {
			WriteLine ("Building bondball");
			if (bottomDefault == -1)
				bb = BondBall_new0 ();
			else
				bb = BondBall_new (bottomDefault);
			
			if (BondBall_InitiateBond(bb, msNext) == false) {
				WriteLine ("  FAILED");
				break;
			} else {
				WriteLine ("  SUCCESS: %s", BondBall_ToString(bb));
			}
			first = false;
			
			continue;
		}
		
		// Try to add one more image to the row.
		if (BondBall_AddRowImage (bb, msNext) == true) {
			WriteLine ("Terminating first row creation.");
			break;
		}
	}

	// Case: no bondball has been build, because first pair does not
	// exist.
	if (bb == NULL)
		return (NULL);
	
	ArrayList* rowFileNames = ArrayList_new0 (NULL);
	char* last = NULL;
	int i;
	for(i=0; i<ArrayList_Count(bb->sets); i++) {
        MatchSet* ms = (MatchSet*) ArrayList_GetItem(bb->sets, i);
		ArrayList_AddItem(rowFileNames, ms->file1);
		last = ms->file2;
	}
	ArrayList_AddItem(rowFileNames, last);

	WriteLine ("First row is:");
	int j;
	for(j=0; j<ArrayList_Count(rowFileNames); j++) {
		char* rowFileName = (char*) ArrayList_GetItem(rowFileNames, j);
		WriteLine ("  %s", rowFileName);
	}

	bb->firstRow = rowFileNames;

	// Check if we have a 360 degree panorama, but only if we have more
	// than two images in the first row. This rules out very wide angle
	// lenses (> 180 degrees), which seems ok to me. Heck, is it even
	// possible?
	MatchSet* msJoining = NULL;
	if (ArrayList_Count(bb->sets) > 2) {
		int k;
		for(k=0; k<ArrayList_Count(ransacFiltered); k++) {
			MatchSet* ms = (MatchSet*) ArrayList_GetItem(ransacFiltered, k);
			if ((strcmp (ms->file1, bb->last->file2) == 0 &&
			     strcmp (ms->file2, bb->first->file1) == 0) ||
			    (strcmp (ms->file2, bb->last->file2) == 0 &&
			     strcmp (ms->file1, bb->first->file1) == 0))
			{
				msJoining = ms;
				break;
			}
		}
	}
	
	bool is360 = false;
	if (msJoining != NULL) {
		WriteLine ("Found 360 degree panorama, merging between \"%s\" and \"%s\" :-)",
			   msJoining->file1, msJoining->file2);
		
		is360 = true;
	} else {
		WriteLine ("Found no 360 degree boundary, assuming a < 360 degree panorama.");
		WriteLine ("In case this is an error and it is a 360 degree boundary, please");
		WriteLine ("consult the manpage documentation for information how to improve");
		WriteLine ("the detection results. Thank you :-)");
		
		is360 = false;
	}
	
	// Align first row.
	BondBall_StretchImages (bb, is360);
	
	// Now roughly add all remaining images.
	bool alignGap = true;
	int alignCount = 0;
	
	WriteLine ("Estimating remaining images' positions");
	while (alignGap == true && alignCount < ArrayList_Count(ransacFiltered)) {
		alignGap = false;
		
		int n;
		for(n=0; n<ArrayList_Count(ransacFiltered); n++) {
			MatchSet* ms = (MatchSet*) ArrayList_GetItem(ransacFiltered, n);
			// Already aligned.
			if (HashTable_Contains(bb->positions,ms->file1) && HashTable_Contains(bb->positions, ms->file2))
				continue;
			
			// Unable to align (yet)
			if (HashTable_Contains(bb->positions, ms->file1) == false &&
			    HashTable_Contains(bb->positions, ms->file2) == false)
			{
				alignGap = true;
				continue;
			}
			
			Position* pPos = NULL;
			char* pPosFile = NULL;
			
			// Alignable cases: one is part of the aligned set
			if (HashTable_Contains(bb->positions, ms->file1) == true &&
			    HashTable_Contains(bb->positions, ms->file2) == false)
			{
				pPos = BondBall_EstimateImage (bb, ms);
				HashTable_AddItem(bb->positions, ms->file2, pPos);
				pPosFile = ms->file2;
			} else if (HashTable_Contains(bb->positions, ms->file1) == false &&
				   HashTable_Contains(bb->positions, ms->file2) == true)
			{
				pPos = BondBall_EstimateImage (bb, ms);
				HashTable_AddItem(bb->positions, ms->file1, pPos);
				pPosFile = ms->file1;
			}
			WriteLine ("  estimated: \"%d\": %d",
				   pPosFile, pPos);
		}
		
		alignCount += 1;
	}
	
	if (alignGap) {
		WriteLine ("");
		WriteLine ("Warning: Unaligned images remain. This is most likely due to loose");
		WriteLine ("         components, see the manual page documentation for details.");
		WriteLine ("");
	} else
		WriteLine ("Done.\n");
	
	/*
	  foreach (string posFile in bb.Positions.Keys) {
	  Console.WriteLine ("have for \"{0}\": {1}", posFile,
	  (BondBall.Position) bb.Positions[posFile]);
	  }
	*/
	WriteLine ("%s", BondBall_ToString(bb));
	return (bb);
}

FilterPoint* FilterPoint_new0()
{
	FilterPoint* self = (FilterPoint*)malloc(sizeof(FilterPoint));
	self->user = NULL;
	return self;
}

void FilterPoint_delete(FilterPoint* self)
{
	if (self) {
		free(self);
	}
}
