//----------------------------------------------------------------------
//	File:			ANNx.h
//	Programmer: 	Sunil Arya and David Mount
//	Last modified:	03/04/98 (Release 0.1)
//	Description:	Internal include file for ANN
//
//	These declarations are of use in manipulating some of
//	the internal data objects appearing in ANN, but are not
//	needed for applications just using the nearest neighbor
//	search.
//
//	Typical users of ANN should not need to access this file.
//----------------------------------------------------------------------
// Copyright (c) 1997-2005 University of Maryland and Sunil Arya and
// David Mount.  All Rights Reserved.
// 
// This software and related documentation is part of the Approximate
// Nearest Neighbor Library (ANN).  This software is provided under
// the provisions of the Lesser GNU Public License (LGPL).  See the
// file ../ReadMe.txt for further information.
// 
// The University of Maryland (U.M.) and the authors make no
// representations about the suitability or fitness of this software for
// any purpose.  It is provided "as is" without express or implied
// warranty.
//----------------------------------------------------------------------
//	History:
//	Revision 0.1  03/04/98
//	    Initial release
//	Revision 1.0  04/01/05
//	    Changed LO, HI, IN, OUT to ANN_LO, ANN_HI, etc.
//----------------------------------------------------------------------

#ifndef ANNx_H
#define ANNx_H

#include <iomanip>				// I/O manipulators
#include <ANN/ANN.h>			// ANN includes

//----------------------------------------------------------------------
//	Global constants and types
//----------------------------------------------------------------------
enum	{ANN_LO=0, ANN_HI=1};	// splitting indices
enum	{ANN_IN=0, ANN_OUT=1};	// shrinking indices
								// what to do in case of error
enum ANNerr {ANNwarn = 0, ANNabort = 1};

//----------------------------------------------------------------------
//	Maximum number of points to visit
//	We have an option for terminating the search early if the
//	number of points visited exceeds some threshold.  If the
//	threshold is 0 (its default)  this means there is no limit
//	and the algorithm applies its normal termination condition.
//----------------------------------------------------------------------

extern int		ANNmaxPtsVisited;	// maximum number of pts visited
extern int		ANNptsVisited;		// number of pts visited in search

//----------------------------------------------------------------------
//	Global function declarations
//----------------------------------------------------------------------

void annError(					// ANN error routine
	char			*msg,		// error message
	ANNerr			level);		// level of error

void annPrintPt(				// print a point
	ANNpoint		pt,			// the point
	int				dim,		// the dimension
	std::ostream	&out);		// output stream



//----------------------------------------------------------------------
//	Orthogonal (axis aligned) halfspace
//	An orthogonal halfspace is represented by an integer cutting
//	dimension cd, coordinate cutting value, cv, and side, sd, which is
//	either +1 or -1. Our convention is that point q lies in the (closed)
//	halfspace if (q[cd] - cv)*sd >= 0.
//----------------------------------------------------------------------

class ANNorthHalfSpace {
public:
	int				cd;			// cutting dimension
	ANNcoord		cv;			// cutting value
	int				sd;			// which side
//
	ANNorthHalfSpace()			// default constructor
	{  cd = 0; cv = 0;  sd = 0;  }

	ANNorthHalfSpace(			// basic constructor
	int				cdd,		// dimension of space
	ANNcoord		cvv,		// cutting value
	int				sdd)		// side
	{  cd = cdd;  cv = cvv;  sd = sdd;  }

	ANNbool in(ANNpoint q) const	// is q inside halfspace?
	{  return  (ANNbool) ((q[cd] - cv)*sd >= 0);  }

	ANNbool out(ANNpoint q) const	// is q outside halfspace?
	{  return  (ANNbool) ((q[cd] - cv)*sd < 0);  }

	ANNdist dist(ANNpoint q) const	// (squared) distance from q
	{  return  (ANNdist) ANN_POW(q[cd] - cv);  }

	void setLowerBound(int d, ANNpoint p)// set to lower bound at p[i]
	{  cd = d;  cv = p[d];  sd = +1;  }

	void setUpperBound(int d, ANNpoint p)// set to upper bound at p[i]
	{  cd = d;  cv = p[d];  sd = -1;  }

	void project(ANNpoint &q)		// project q (modified) onto halfspace
	{  if (out(q)) q[cd] = cv;  }
};

								// array of halfspaces
typedef ANNorthHalfSpace *ANNorthHSArray;

#endif
