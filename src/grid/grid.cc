/*
 * grid implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include <iostream>

#include "grid/grid.h"
#include <cstdlib>

/**
 * CTor.
 * Allocates an array of gridPoints
 * and sets the offsets and sizes
 *
 * @param offsetX the x-offest of the array
 * @param offsetZ the z-offset of the array
 * @param sizeX the x-size of the array
 * @param sizeZ the y-size of the array
 */
grid::grid(long offsetX, long offsetZ, long sizeX, long sizeZ)
{
    if(sizeX < 1 || sizeZ < 1)
    {
	std::cerr << "ERROR: Invalid array size in grid::grid! "<< std::endl;
	exit(1);
    }
 
    points = NULL;

    // Allocate array
    this->points = new gridPoint**[sizeX];

    for(int i=0; i < sizeX; ++i)
    {
        this->points[i] = new gridPoint*[sizeZ];
        for(int j=0; j < sizeZ; ++j)
        {
            this->points[i][j] = new gridPoint(offsetX + i, offsetZ + j);       
        }
    }
    
    this->offsetX = offsetX;
    this->offsetZ = offsetZ;
    this->sizeX = sizeX;
    this->sizeZ = sizeZ;
}

/**
 * DTor. Deletes the array
 */
grid::~grid()
{
    clear();
}

/**
 * Frees the memory if array is set
 */
void grid::clear()
{
    if(this->points == NULL)
        return;

    // delete all points
    for(int i=0; i < getSizeX(); ++i)
    {
        for(int j=0; j < getSizeZ(); ++j)
        {
            delete this->points[i][j];      
        }
        
        delete[] this->points[i];
    }

    delete[] this->points;

    this->points = NULL;
}

/**
 * Adds point to given point in the grid (updates the point)
 *
 * @param x the absolute x coordinate
 * @param z the absolute z coordinate
 * @param occupied is the point occupied or free?
 * @param weight weighting of the information
 */
void grid::addPoint(long x, long z, unsigned int count, unsigned int occupied)
{
    if(this->points == NULL)
    {
	std::cerr << "ERROR: In grid::addPoint, allocate never called!" << std::endl;
	exit(1);
    }

    // get the absolute gridPoint 
    gridPoint *p = getAbsolutePoint(x, z);

    // increase counters 
    p->addCount(count, occupied);
}

/**
 * Adds new point into the grid or updates the point with new
 * point information 
 * @param point a reference to the point (new information)
 */
void grid::addPoint(const gridPoint& point)
{
    gridPoint *p = getAbsolutePoint(point.getX(), point.getZ());
    p->addCount(point.getCount(), point.getOccupied());
}

/**
 * Sets values for the point
 *
 * @param x absolute x coordinate
 * @param z absolute z coordinate
 * @param count new counter value (number of counts)
 * @param occupied new occupied value (number of occupieds)
 */
void grid::setPoint(long x, long z, unsigned int count, unsigned int occupied)
{
    gridPoint* p = getAbsolutePoint(x, z);
    p->setFixed(count, occupied);
}

/**
 * Returns point of the absolute coordinates
 * The absolute coordinates get transformed to relative coordinates

 * @param x the absolute x coordinate
 * @param z the absolute z coordinate
 *
 * @return gridPoint* pointer to the found gridPoint
 */
gridPoint* grid::getAbsolutePoint(long x, long z) const
{
    // Transform to relative coordinates
    x -= getOffsetX();
    z -= getOffsetZ();
   
    if( x < 0 || x >= getSizeX() || z < 0 || z >= getSizeZ())
    {
	std::cerr << "ERROR: In grid::getAbsolutePoint (" 
		  << x << "|" << z <<") not in the grid!" << std::endl;
	exit(1);
    }
    
    return this->points[x][z];
}


 
