
/* autopano-sift, Automatic panorama image creation
 * Copyright (C) 2004 -- Sebastian Nowozin
 *
 * This program is free software released under the GNU General Public
 * License, which is included in this software package (doc/LICENSE).
 */

/* BondBall.cs
 *
 * Generic preliminary position finding algorithm for panoramic imaging.
 *
 * (C) Copyright 2004 -- Sebastian Nowozin (nowozin@cs.tu-berlin.de)
 *
 * Any kind of panorama made from small angle input pictures (ie. no special
 * fisheye or wideangle lenses). The input images have to fullfil a strict
 * left-to-right or right-to-left order for the first row, which has to be
 * horizontal.
 */

#include "AutoPanoSift.h"

const int Direction_Unknown = 0;
const int Direction_LeftToRight = 1;
const int Direction_RightToLeft = 2;

BondBall* BondBall_new0()
{
    BondBall* self = (BondBall*)malloc(sizeof(BondBall));
    self->dir = Direction_Unknown;
    self->center = 0.0;
    self->rotation = 0.0;
    self->bottomDefault = -1;
    self->bondOrientationTolerance = 35.0;
    self->sets = ArrayList_new0(NULL);
    self->first = NULL;
    self->last = NULL;
    self->firstRow = NULL;
    self->positions = NULL;
    return self;
}

void BondBall_delete(BondBall* self)
{
    if (self) {
	ArrayList_delete(self->sets);
	ArrayList_delete(self->firstRow);
	HashTable_delete(self->positions);
	free(self);
    }
}

BondBall* BondBall_new(int bottomDir)
{
    BondBall* self = BondBall_new0();
    self->bottomDefault = bottomDir;
    return self;
}


bool BondBall_IsWithinAngleDegree (double left, double right, double test)
{
    while (left < 0.0)
	left += 360.0;
    while (right < 0.0)
	right += 360.0;
    while (test < 0.0)
	test += 360.0;
    
    // easy case, no wraparound
    if (left < right) {
	if (test >= left && test < right)
	    return (true);
	
	return (false);
    }
    
    // left is bigger than right, this means they wrap at 0.0/360.0 degrees
    if (test >= left && test <= 360.0)
	return (true);
    if (test >= 0.0 && test < right)
	return (true);
    
    return (false);
}

// Initiate a new panorama positioning by starting with the first two
// pictures gives in the 'first' matchset.
//
// Return true if we positively set up the first row orientation and its
// picture orientation.
// Return false if we cannot determine the orientations.
bool BondBall_InitiateBond (BondBall* self, MatchSet* first)
{
    ImageMatchModel* fit = first->bestMatchFit;
    self->first = self->last = first;
    ArrayList_AddItem (self->sets, self->first);

    double centerDegree = (fit->trans->centerAngle / (2.0 * M_PI)) * 360.0;
    if (centerDegree < 0.0)
	centerDegree += 360.0;
    
    // Simple cases: normalized rotation, left to right or right to left
    // A. Left to right
    if (BondBall_IsWithinAngleDegree (0.0 - self->bondOrientationTolerance,
				      0.0 + self->bondOrientationTolerance, centerDegree))
    {
	self->center = 0.0;
	self->rotation = 0.0;
	self->dir = Direction_LeftToRight;
	
	return (true);
	// B. Right to left
    } else if (BondBall_IsWithinAngleDegree (180.0 - self->bondOrientationTolerance,
					     180.0 + self->bondOrientationTolerance, centerDegree))
    {
	self->center = 180.0;
	self->rotation = 0.0;
	self->dir = Direction_RightToLeft;
	
	return (true);
    }
    
    // Ambiguous case: 90/270 degrees, tilted by -90 or 90 degrees. Now,
    // we first determine where the bottom lies in the pictures (left or
    // right)
    int bottomDir[2] = {0,0};
    int n;
    for ( n = 0 ; n < 2 ; ++n) {
	if (self->bottomDefault != -1) {
	    bottomDir[n] = self->bottomDefault;
	} else {
	    bottomDir[n] = BondBall_GuessBottomOrientation (MatchSet_GetOriginalKeys(first,n),
							    n == 0 ? first->xDim1 : first->xDim2);
	}
    }

    // Cannot tell or directions mismatch
    if (bottomDir[0] == -1 || bottomDir[1] == -1 ||
	bottomDir[0] != bottomDir[1])
    {
	WriteLine ("Error: The picture orientation is ambiguous.");
	WriteLine ("       We have either -90 or 90 degree input pictures.\n");
	
	WriteLine ("       To possibly resolve this, please try to use the --bottom-is-left");
	WriteLine ("       or --bottom-is-right option.");
	
	return (false);
    }
    
    WriteLine ("First row pictures have the bottom on the %s side.",
	       bottomDir[0] == 0 ? "left" : "right");

    // Resolve the ambiguity among four cases
    if (BondBall_IsWithinAngleDegree (90.0 - self->bondOrientationTolerance,
				      90.0 + self->bondOrientationTolerance, centerDegree))
    {
	self->dir = Direction_LeftToRight;
	self->center = 90.0;

	if (bottomDir[0] == 0)
	    self->rotation = -90.0;
	else
	    self->rotation = 90.0;
    } else if (BondBall_IsWithinAngleDegree (270.0 - self->bondOrientationTolerance,
					     270.0 + self->bondOrientationTolerance, centerDegree))
    {
	self->dir = Direction_RightToLeft;
	self->center = 270.0;
	
	if (bottomDir[0] == 0)
	    self->rotation = -90.0;
	else
	    self->rotation = 90.0;
    }
    
    return (true);
}

// Return true on end.
bool BondBall_AddRowImage (BondBall* self, MatchSet* next)
{
    if (strcmp (self->last->file2, next->file1) != 0)
	FatalError("The row is not continuous in the next matchset.");
    
    // Get angle between pictures
    ImageMatchModel* fit = next->bestMatchFit;
    double centerDegree = (fit->trans->centerAngle / (2.0 * M_PI)) * 360.0;
    if (centerDegree < 0.0)
	centerDegree += 360.0;
    
    if (BondBall_IsWithinAngleDegree (self->center - self->bondOrientationTolerance,
				      self->center + self->bondOrientationTolerance,
				      centerDegree))
    {
	self->last = next;
	ArrayList_AddItem (self->sets, next);
	WriteLine ("Angle %.2f degrees fits, adding \"%s\" to row.",
		   centerDegree, next->file2);
	
	return (false);
    } else {
	WriteLine ("Angle %.2f degrees of \"{%s}\" is outside of %.2f+/-%.2f range, row end reached",
		   centerDegree - 360.0, next->file2, self->center, self->bondOrientationTolerance);
	
	return (true);
    }
}

// One of the pictures in ms must be in the positions hashtable already,
// and the other must be outside.
Position* BondBall_EstimateImage (BondBall* self, MatchSet* ms)
{
    if (self->positions == NULL)
	FatalError ("Positions hashtable is empty, cannot align.");
    
    bool image1known = HashTable_Contains(self->positions, ms->file1);
    char* knownFile = image1known ? ms->file1 : ms->file2;
    //char* unknownFile = image1known ? ms->file2 : ms->file1;

    Position* knownPos = (Position*) HashTable_GetItem(self->positions, knownFile);
    ImageMatchModel* fit = (ImageMatchModel*) ms->bestMatchFit;

    /*
      double centerDegree = (fit.CenterAngle / (2.0 * M_PI)) * 360.0;
      double newRotation = centerDegree; +
      (knownPos.Rotation * 2.0 * M_PI) / 360.0;
    */
    //Console.WriteLine ("fit.RotationAngle = %f", fit->rotationAngle);
    double newRotation = fit->trans->rotationAngle +
	(knownPos->rotation * 2.0 * M_PI) / 360.0;
    newRotation = (newRotation / (2.0 * M_PI)) * 360.0;

    /*Console.WriteLine ("new rotation for image \"%s\" is %f degrees",
      unknownFile, newRotation);
      
      Console.WriteLine ("  centerangle = %f", fit->centerAngle);
      Console.WriteLine ("  sin(centerangle) = %f", sin (fit->centerAngle));
      Console.WriteLine ("  cos(centerangle) = %f", cos (fit->centerAngle));
    */
    
    double yaw = knownPos->yaw;
    double pitch = knownPos->pitch;
    /*Console.WriteLine ("fit.ShiftWidth = %f", fit->shiftWidth);
      Console.WriteLine ("ms.xDim1 = %d", ms->xDim1);*/
    double angleShiftHorizontal = min (
	((double) fit->trans->shiftWidth / (double) ms->xDim1) * self->yawStep * 2.0,
	20.0);
    double angleShiftVertical = min (
	((double) fit->trans->shiftWidth / (double) ms->yDim1) * self->yawStep * 2.0,
	20.0);
    /*Console.WriteLine ("shift h/v: {0}, {1}, yawStep = {2}",
      angleShiftHorizontal, angleShiftVertical, yawStep);*/
    
    /*
      Console.WriteLine ("## CenterAngle = {0}, RotationAngle = {1}",
      fit.CenterAngle, fit.RotationAngle);
      Console.WriteLine ("## knownPos.Rotation = {0}", knownPos.Rotation);
      Console.WriteLine ("## newPos.Rotation = {0}", newRotation);
    */
    
    double ca = fit->trans->centerAngle + ((knownPos->rotation / 360.0) * 2.0 * M_PI);
    /*ca += M_PI;	// invert direction
      if (ca >= (2.0 * M_PI))
      ca -= 2.0 * M_PI;*/
    //Console.WriteLine ("ca = {0}", ca);
    
    double reverseFactor = image1known ? 1.0 : -1.0;
    //Console.WriteLine ("yaw -= {0}", reverseFactor * cos (ca) * angleShiftHorizontal);
    //Console.WriteLine ("pitch -= {0}", reverseFactor * sin (ca) * angleShiftVertical);
    
    yaw -= cos (ca) * angleShiftHorizontal;
    pitch -= reverseFactor * sin (ca) * angleShiftVertical;
    
    return (Position_new (yaw, pitch, newRotation));
    /*
      if (centerDegree < 0.0)
      centerDegree += 360.0;
    */
}

Position* Position_new0()
{
    Position* self = (Position*)malloc(sizeof(Position));
    return self;
}

void Position_delete(Position* self)
{
    if (self) {
	free(self);
    }
}

Position* Position_new(double yaw, double pitch, double rotation)
{
    Position* self = Position_new0();
    self->yaw = yaw;
    self->pitch = pitch;
    self->rotation = rotation;
    return self;
}

char* Position_ToString (Position* self)
{
    char* str = (char*) malloc(60);
    sprintf(str, "pos (yaw = %.2f, pitch = %.2f, rotation = %.2f)",
	    self->yaw, self->pitch, self->rotation);
    return str;
}

void BondBall_StretchImages (BondBall* self, bool is360)
{
    self->positions = HashTable_new0 (NULL, Position_delete);
    
    // In case its a 360 degree panorama, things are easy
    self->yawStep = 360.0 / ArrayList_Count(self->firstRow);
    
    // In case it's not a full pano, we lower the value. To have an exact
    // value is really not so important, its just a rough minimum for the
    // later optimization process to base its work upon.
    if (is360 == false) {
	// Now we employ heuristics/educated guessing... humpf.
	self->yawStep = min (20.0, self->yawStep);
    }
    
    // Set the yaw, pitch and rotation values
    int xs = 0;
    int i;
    for(i=0; i<ArrayList_Count(self->firstRow); i++) {
	char* filename = (char *) ArrayList_GetItem(self->firstRow, i);
	double yawCur = xs * self->yawStep;

	// Roughly align in center for non 360 degree panoramas.
	if (is360 == false)
	    yawCur -= (ArrayList_Count(self->firstRow) / 2.0) * self->yawStep;
	WriteLine ("DEBUG %s: yawCur = %f, xs: %d, yawStep = %f", filename, yawCur, xs, self->yawStep);

	//WriteLine ("%s: yaw = %f, rotation = %f", filename, yawCur, self->rotation);
	HashTable_AddItem(self->positions , filename,
			  Position_new (yawCur, 0.0, self->rotation));
	xs += 1;
    }
}

// Guesses where the bottom of the image is, based on keypoint density.
// This is highly experimental code.
// The only cases considered is left/right, as we assume no image has the
// bottom on the top.
// TODO: maybe add a third case: bottom at bottom, though this could be
//    reliably estimated based on image order alone.
//
// Return -1 in case its not certain,
// return 0 in case the bottom is estimated to be on the west/left side
// return 1 in case the bottom is estimated to be on the east/right side
int BondBall_GuessBottomOrientation (ArrayList* keys, int xDim)
{
    double xAccum = 0.0;

    int i;
    for(i=0; i<ArrayList_Count(keys); i++) {
	KeypointN* kp = (KeypointN*) ArrayList_GetItem(keys, i);
	xAccum += kp->x;
    }

    xAccum /= ArrayList_Count(keys);
    //WriteLine ("xDim: %d, xAverage: %f", xDim, xAccum);
    
    double averageDivergenceBoundary = xDim / 12.0;
    if (xAccum <= ((xDim / 2) - averageDivergenceBoundary))
	return (0);
    else if (xAccum >= ((xDim / 2) + averageDivergenceBoundary))
	return (1);
    
    return (-1);
}

char* BondBall_ToString (BondBall* self)
{
    char* str = (char*)malloc(40);
    sprintf(str,"%d, center %.2f, rotation %.2f",
	    self->dir, self->center, self->rotation);
    return str;
}
