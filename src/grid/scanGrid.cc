/*
 * scanGrid implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include "grid/scanGrid.h"

/**
 * CTor.
 * Calls CTor of grid and sets viewpointx and viewpointz
 *
 * @param offsetX the x offset of the robot
 * @param offsetZ the z offset of the robot
 * @param viewpointX the x-position of the robot
 * @param viewpointZ the z-position of the robot
 * @param sizeX the x size of the grid
 * @param sizeZ the z size of the grid
 */
scanGrid::scanGrid(long viewpointX, long viewpointZ,
		   long offsetX, long offsetZ,
		   long sizeX, long sizeZ)
    : grid(offsetX, offsetZ, sizeX, sizeZ)
{
    this->viewpointX = viewpointX;
    this->viewpointZ = viewpointZ;
}

/**
 * DTor.
 */
scanGrid::~scanGrid()
{
}
