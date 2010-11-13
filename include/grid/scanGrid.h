#ifndef __SCANGRID_H_
#define __SCANGRID_H_

#include "grid/grid.h"

/**
 * The class represents a 2D section of a scan.
 * It inherrits from grid -> the internal representation is an array
 *
 * @author Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * @date 14.02.2008
 */
class scanGrid : public grid
{   
    /** The position of the robot (X) */
    long viewpointX;

    /** The position of the robot (Z) */
    long viewpointZ;

 public:
    /** @brief CTor */
    scanGrid(long viewX, long viewZ, long offsetX, long offsetZ,
	     long sizeX, long sizeZ);

    /** @brief DTor */
    virtual ~scanGrid();
   
    /**
     * Getter for the x position of the robot when the scan was made 
     * @return x position
     */
    inline long getViewpointX() const {
	return this->viewpointX;
    }

    /**
     * Getter for the z position of the robot when the scan was made
     * @return z position
     */
    inline long getViewpointZ() const {
	return this->viewpointZ;
    }
};

#endif
