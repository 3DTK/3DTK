/*
 * parcelinfo implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include "grid/parcelinfo.h"

long parcelinfo::parcelwidth;
long parcelinfo::parcelheight;

/**
 * Ctor
 * @param offsetX the x-offset
 * @param offsetZ the z-offset
 * @param filename the filename of the parcel
 */
parcelinfo::parcelinfo(long offsetX, long offsetZ, std::string filename)
{
    this->offsetX = offsetX;
    this->offsetZ = offsetZ;
    this->filename = filename;
    this->used = true;
}

/** 
 * Sets the parcelsize
 * @param with the width of the parcel
 * @param height the height of the parcel
 */
void parcelinfo::setParcelsize(long width, long height)
{
    parcelinfo::parcelwidth = width;
    parcelinfo::parcelheight = height;
}

/**
 * Sets the used-flag true (Which means
 * that the parcel was used during the last scan)
 */
void parcelinfo::setUsed()
{
    this->used = true;
}

/**
 * Resets the used-flag
 */
void parcelinfo::resetUsed()
{
    this->used = false;
}

/**
 * Returns whether the parcel was needed
 * @return True if it was needed 
 */
int parcelinfo::wasUsed() const
{
    return this->used;
}

/**
 * Operator <. Compares by offsets.
 *
 * @param parcel the parcel to be compared to this
 * @return true if this < parcel
 */
bool parcelinfo::operator< (const parcelinfo& parcel) const
{
    if(getOffsetX() < parcel.getOffsetZ())
    {
	return true;
    }
    else if(getOffsetX() == parcel.getOffsetX())
    {
	if(getOffsetZ() < parcel.getOffsetZ())
	    return true;
	else
	    return false;
    }
    else
    {
	return false;
    }
}

/**
 * Compares two parcels if they are equal (Offset-X and -Z equal)
 *
 * @param parcel the other parcel
 * @return true if euqal
 */
bool parcelinfo::operator== (const parcelinfo &parcel) const
{
    return (getOffsetX() == parcel.getOffsetX() &&
	    getOffsetZ() == parcel.getOffsetZ());
}

/**
 * The methods checks if the coordinates given lie in 
 * the parcel represented by the parcelinfo
 *
 * @param x The absolute x coordinate
 * @param z the absolute z coordinate
 * @return true if the point lies within the parcel
 */
bool parcelinfo::contains(long x, long z) const
{
    bool xIn = (x >= getOffsetX() &&
		x <= getOffsetX() + parcelinfo::parcelwidth);
    
    bool zIn = (z >= getOffsetZ() &&
		z <= getOffsetZ() + parcelinfo::parcelheight);
   
    return xIn && zIn;
}


