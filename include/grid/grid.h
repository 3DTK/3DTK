#ifndef __GRID_H_
#define __GRID_H_

#include "grid/gridPoint.h"

/**
 * The class represents the base class for all 2D views.
 * It contains a two dimensional array of gridPoints, 
 * the offset (in absolute coordinates) of the grid and
 * the size of the grid.
 * The size of the grid must be set during the instanciation of
 * the grid and cant be changed afterwards.
 *
 * It provides methods for adding and setting information about
 * points to the grid and obtaining a point and writing the grid
 * using a gridWriter-object.

 *
 * @author Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * @date 12.02.2008
 */
class grid
{
 public:
    /** Array containing the gridPoints */
    gridPoint*** points;

 private:
    /** X offset (absolute coordinate) */
    long offsetX;

    /** Z offset (absolute coordinate) */
    long offsetZ;

    /** Width of the array */
    long sizeX;

    /** Height of the array */
    long sizeZ;      
    
    /** @brief The method frees the internal array */
    void clear();

    /** @brief The method allocates and initialises the internal array */
    void allocate(long maxX, long maxZ);

 public:
    /** @brief CTor */
    grid(long offsetX, long offsetZ, long sizeX, long sizeZ);

    /** @brief Dtor */
    virtual ~grid();

    /** @brief Adds the information of the given point to the grid */
    virtual void addPoint(const gridPoint& point);

    /** @brief Adds the information to the grid at the specified coords */
    virtual void addPoint(long x, long z, unsigned int count, unsigned int occupied);

    /** @brief Sets the fixed values for a points */
    virtual void setPoint(long x, long z, unsigned int count, unsigned int occupied);
    
    /** @brief Returns the point of the absolute coordinates */
    gridPoint* getAbsolutePoint(long x, long z) const;

    /**
     * Method checks if the given Point is in this grid
     * @param x The absolute x coordinate
     * @param z The absolute z coordinate
     */
    inline bool contains(long x, long z) const
    {
	x -= getOffsetX();
	z -= getOffsetZ();
	if( x < 0 || z < 0 || x >= getSizeX() || z >= getSizeZ())
	    return false;
	else
	    return true;	    
    }

    /**
     * Getter for offsetx
     * @return offsetX
     */
    inline long getOffsetX() const {
	return this->offsetX;
    }

    /** 
     * Getter for offsetZ
     * @return offsetZ
     */
    inline long getOffsetZ() const {
	return this->offsetZ;
    }

    /**
     * Getter for the arraysize (X)
     * @return arraysize x
     */
    inline long getSizeX() const {
	return this->sizeX;
    }

    /**
     * Getter for the arraysize (Z)
     * @return arraysize z
     */
    inline long getSizeZ() const {
	return this->sizeZ;
    }
};

#endif
