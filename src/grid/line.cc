/*
 * line implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include "grid/line.h"
#include <cmath>

/**
 * CTOR. 
 * 
 * @param start the startpoint of the line
 * @param end the endpoint of the line
 */
line::line(gridPoint* start, gridPoint* end)
{
    this->start = start;
    this->end = end;
}

/** 
 * Returns the length of the line
 * 
 * @return the length of the line 
 */
double line::getLength()
{
    return sqrt(pow((double)start->getX() - end->getX(), 2) +
 		pow((double)start->getZ() - end->getZ(), 2));
}
