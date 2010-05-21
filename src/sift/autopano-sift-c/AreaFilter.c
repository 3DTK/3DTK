
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* AreaFilter.cs
 *
 * Area maximizing match filtration using convex hull and polygon-area
 * maximization.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 *
 * Convex hull code based on Ken Clarksons original C implementation.
 * Polygon area formula by Paul Bourke.
 */

#include "AutoPanoSift.h"

#ifdef TEST_MAIN
int main (int argc, char* argv[])
{
	ArrayList* al = ArrayList_new0 (FilterPoint_delete);
	Random* rnd = Random_new0 ();

	int n;
	for (n = 0 ; n < 10 ; ++n) {
		FilterPoint* p = FilterPoint_new0 ();
	
		p->x = Random_NextDouble(rnd) * 400.0;
		p->y = Random_NextDouble(rnd) * 400.0;
	
		ArrayList_AddItem(al, p);
	}
	int i;
	for(i=0; i<ArrayList_Count(al); i++) {
		FilterPoint* p = ArrayList_GetItem(al, i);
		WriteLine ("%f %f # GNUPLOT1", p->x, p->y);
	}

	AreaFilter* conv = AreaFilter_new0 ();
	ArrayList* hull = AreaFilter_CreateConvexHull(conv, al);
	
	WriteLine ("\nhull: %d elements", ArrayList_Count(hull));
	int j;
	for(j=0; j<ArrayList_Count(hull); j++) {
		FilterPoint* p = ArrayList_GetItem(hull, j);
		WriteLine ("%f %f # GNUPLOT2", p->x, p->y);
	}
	
	WriteLine ("\npolygon area: %f", AreaFilter_PolygonArea(conv, hull));
	ArrayList_delete(al);
	return 0;
}

#else

AreaFilter* AreaFilter_new0()
{
	AreaFilter* self = (AreaFilter*)malloc(sizeof(AreaFilter));
	return self;
}

void AreaFilter_delete(AreaFilter* self)
{
	if (self) {
		free(self);
	}
}

// Measure the absolute area of a non self-intersecting polygon.
// Formula by Paul Bourke
// (http://astronomy.swin.edu.au/~pbourke/geometry/polyarea/)
//
// The input points must be ordered (clock- or counter-clockwise). The
// polygon is closed automatically between the first and the last point,
// so there should not be two same points in the polygon point set.
double AreaFilter_PolygonArea (AreaFilter* self, ArrayList* orderedPoints)
{
	double A = 0.0;
	
	int n;
	for (n = 0 ; n < ArrayList_Count(orderedPoints) ; ++n) {
		FilterPoint* p0 = (FilterPoint*) ArrayList_GetItem(orderedPoints,n);
		FilterPoint* p1 = (FilterPoint*) ArrayList_GetItem(orderedPoints, (n + 1) % ArrayList_Count(orderedPoints));
		
		A += p0->x * p1->y - p1->x * p0->y;
	}

	A *= 0.5;
	
	return (abs (A));
}

// Create the convex hull of a point set.
ArrayList* AreaFilter_CreateConvexHull (AreaFilter* self, ArrayList* points)
{
	int chn = AreaFilter_CreateHull (self, points);
	
	ArrayList* hull = ArrayList_new0 (NULL);
	int k;
	for (k = 0 ; k < chn ; ++k)
		ArrayList_AddItem(hull, ArrayList_GetItem (points, k));
				  
	return (hull);
}

int AreaFilter_CompareLow (const FilterPoint* p1, const FilterPoint* p2)
{
	
	double v = p1->x - p2->x;
	if (v > 0)
		return (1);
	if (v < 0)
		return (-1);
	
	v = p2->y - p1->y;
	if (v > 0)
		return (1);
	if (v < 0)
		return (-1);
	
	// Equal point
	return (0);
}

int AreaFilter_CompareHigh (const FilterPoint* p1, const FilterPoint* p2)
{
	return AreaFilter_CompareLow(p2, p1);
}


bool AreaFilter_ccw (ArrayList* points, int i, int j, int k)
{
	double a = ((FilterPoint*)ArrayList_GetItem(points, i))->x - ((FilterPoint*)ArrayList_GetItem(points, j))->x;
	double b = ((FilterPoint*)ArrayList_GetItem(points, i))->y - ((FilterPoint*)ArrayList_GetItem(points, j))->y;
	double c = ((FilterPoint*)ArrayList_GetItem(points, k))->x - ((FilterPoint*)ArrayList_GetItem(points, j))->x;
	double d = ((FilterPoint*)ArrayList_GetItem(points, k))->y - ((FilterPoint*)ArrayList_GetItem(points, j))->y;
	
	return ((a * d - b * c) <= 0.0);
}

int AreaFilter_MakeChain (AreaFilter* self, ArrayList* points, int (*comp)(const FilterPoint*, const FilterPoint*))
{
	IComparator comparator;
	comparator.compareTo = (int ( *)(IComparator *,const void *,const void *)) comp;
	ArrayList_Sort(points, &comparator);
	
	int s = 1;
	int pCount = ArrayList_Count(points);
	int i;
	for (i = 2 ; i < pCount ; ++i) {
		int j;
		for (j = s ; j >= 1 && AreaFilter_ccw (points, i, j, j - 1) ; --j)
			;
		
		s = j + 1;
		
		// Swap
		FilterPoint* t = (FilterPoint*) ArrayList_GetItem(points, s);
		ArrayList_SetItem(points, s, ArrayList_GetItem(points, i));
		ArrayList_SetItem(points, i, t);
	}
	
	return (s);
}

int AreaFilter_CreateHull (AreaFilter* self, ArrayList* points)
{
	int u = AreaFilter_MakeChain (self, points, AreaFilter_CompareLow);
	
	if (ArrayList_Count(points) == 0)
		return (0);
	/*
	  int k;
	  for (k = 0 ; k < u ; ++k)
	      WriteLine ("point %d: %f %f", k, ((FilterPoint*)ArrayList_GetItem(points, k)[k])->x, 
                                       	       ((FilterPoint*)ArrayList_GetItem(points, k)[k])->y);
	*/

	ArrayList* pointsHigh = ArrayList_new(ArrayList_Count(points)+1-u, NULL);
	//WriteLine ("points.Length = %d, u = %d", ArrayList_Count(points), u);
	
	ArrayList_Copy (points, u, pointsHigh, 0, ArrayList_Count(points) - u);
	ArrayList_SetItem(pointsHigh, ArrayList_Count(pointsHigh) - 1,  ArrayList_GetItem(points, 0));

	int h = AreaFilter_MakeChain (self, pointsHigh, AreaFilter_CompareHigh);
	
	int p;
	for ( p = u ; p < ArrayList_Count(points) ; ++p) {
		ArrayList_SetItem(points, p, ArrayList_GetItem(pointsHigh, p-u));
	}
	
	/*
	  WriteLine ("h = %d, u = %d", h, u);
	  int k;
	  for (k = 0 ; k < (h + u) ; ++k)
	      WriteLine ("point %d: %f %f", k, ((FilterPoint*)ArrayList_GetItem(points, k)[k])->x, 
                                       	       ((FilterPoint*)ArrayList_GetItem(points, k)[k])->y);
	*/

	return (h + u);
}

#endif

