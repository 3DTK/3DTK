/*
 * gridPoint implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include "grid/gridPoint.h"
#include <cstdlib>
#include <iostream>

/**
 * CTor. Sets counter and occupied to 0 and the coordinates to x and z
 *
 * @param x the x coordinate
 * @param z the z coordinate
 */
gridPoint::gridPoint(long x, long z)
{
    this->x = x;
    this->z = z;
    this->count = 0;
    this->occupied = 0;
}

/**
 * The Method increases the internal counter of the point.
 * If only count should be increased, occupied must be 0;
 *
 * @param count weighting of the information
 * @param occupied Weighting of occupied info 
 */
void gridPoint::addCount(unsigned int count, unsigned int occupied)
{   
    if(count < 0 || occupied < 0)
    {
	std::cerr << "ERROR: Invalid values in gridPoint::addCount " << std::endl;
	std::cerr << "(" << count << " " << occupied << ")" << std::endl;
	exit(1);
    }

    this->count += count;
    this->occupied += occupied;
}

/**
 * Sets the counters to count and occupied
 *
 * @param count new Count
 * @param occupied new Occupied
 */
void gridPoint::setFixed(unsigned int count, unsigned int occupied)
{
    this->count = count;
    this->occupied = occupied;
}


/**
 * Calculates the percentage for the gridPoint
 * If count is 0 the point has not been reached and -1 is returned,
 * otherwise occupied/count
 *
 * @return percentage of the gridPoint occupancy [0.0 1.0] or -1.0
 */
float gridPoint::getPercent() const
{
    if(this->count == 0) 
	return -1.0;
	
    return (float)this->occupied / (float)this->count;
}
