/*
 * parcel implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include "grid/parcel.h"
#include <fstream>
#include <cstdlib>
#include <iostream>
using std::cerr;
using std::endl;

/**
 * Ctor
 *
 * @param offSetX the x-offset of the parcel
 * @param offSetZ the z-offset of the parcel
 * @param sizeX the width of the parcel
 * @param sizeZ the height of the parcel
 */
parcel::parcel(long offSetX, long offSetZ, long sizeX, long sizeZ)
    : grid(offSetX, offSetZ, sizeX, sizeZ)
{
}

/**
 * Adds a grid to the parcel.
 * Adds just the points belonging to the parcel and
 * ignoring the rest.
 * 
 * @param grid the grid to be added to the parcel
 */
void parcel::addGrid(const grid* g)
{
    // Calculate the startvalue of x
    long startX = g->getOffsetX() < getOffsetX() ? getOffsetX() : g->getOffsetX();
    
    // calculate z-startvalue
    long startZ = g->getOffsetZ() < getOffsetZ() ? getOffsetZ() : g->getOffsetZ();
    
    // calculate x-endvalue
    long endX = g->getOffsetX() + g->getSizeX() < getOffsetX() + getSizeX() ? g->getOffsetX() + g->getSizeX() 
	: getOffsetX() + getSizeX();

    // calculate z-endvalue
    long endZ = g->getOffsetZ() + g->getSizeZ() < getOffsetZ() + getSizeZ() ? g->getOffsetZ() + g->getSizeZ() 
	: getOffsetZ() + getSizeZ();

    // add every point
    for(int i = startX; i < endX; i++)
    {
	for(int j = startZ; j < endZ; j++)
	{
	    this->addPoint(*g->getAbsolutePoint(i, j));
	}
    }
} 

/**
 * The static method reads the file and creates a new parcel.
 * The parcel is allocated with new, so the caller has to make
 * sure it is deleted properly!
 *
 * @param filename the filename of the parcel
 * @return parcel the created parcel
 */
parcel* parcel::readParcel(std::string filename)
{
    std::ifstream infile(filename.c_str());
    
    // Stream ok?
    if(!infile.good())
    {
	std::cerr << "ERROR: In parcel::readparcel, couldn't open stream!" << std::endl;
	exit(1);
    }
    
    // Read sizes
    long sizeX;
    long sizeZ;
    infile >> sizeX >> sizeZ;
    
    // Read offsets
    long offsetX;
    long offsetZ;
    infile >> offsetX >> offsetZ;

    // Create parcel
    parcel* p = new parcel(offsetX, offsetZ, sizeX, sizeX);
    
    // Read all information
    long x, z, count, occupied;
    while(!infile.eof())
    {
	infile >> x >> z >> count >> occupied;
	p->setPoint(x, z, count, occupied);      	
    }
    
    infile.close();
    
    return p;
}
