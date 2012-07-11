/*
 * scanToGrid implementation
 *
 * Copyright (C) Uwe Hebbelmann, Sebastian Stock, Andre Schemschat
 *
 * Released under the GPL version 3.
 *
 */

#include <cmath>
#include <iostream>
#include <map>

#include "grid/scanToGrid.h"

using std::vector;
using std::map;

/**
 * CTor.
 *
 * @param resolution the resolution of the grid (centimeters per gridunit)
 * @param minRelevantHeight The minimal height of a scan that is relevant
 * @param maxRelevantHeight The maximal height of a scan that is relevant
 * @param maxDistance The maximal relevant distance
 * @param minimalWeighting The minimal weighting for the waypoints
 * @param createWaypoints If set waypoints will be created
 * @param neighbours If set neighbours will be created
 */ 
scanToGrid::scanToGrid(double resolution,
		       double minRelevantHeight, double maxRelevantHeight,
		       int maxDistance, int spotRadius,
		       bool createWaypoints, bool neighbours)
{
    this->resolution = resolution;
    this->minRelevantHeight = minRelevantHeight;
    this->maxRelevantHeight = maxRelevantHeight;
    this->waypoints = createWaypoints;
    this->neighbours = neighbours;
    this->spot = (float)spotRadius / 3000.0;

    // Calculate minimal weighting
    this->minimalWeighting = 1;//(1((float)minimalWeighting-1)/
//    		      (float)scaleToGrid(maxDistance));
}

/**
 * The method checks if the given point is relevant
 * for creating the grid.
 * It is not relevant if it is below or above a specified
 * height, or if it lies to far away (??)
 *
 * @param p The point to check (not scaled to grid)
 * @return True if the point is relevant
 */
bool scanToGrid::isPointRelevant(const Point &p) const
{
    if(p.y < this->minRelevantHeight || p.y > this->maxRelevantHeight)
    {
	return false;
    }
    else
	return true;
}

/**
 * The method scales the absolute coordinate of the scanpoint 
 * to the matching coordinate of the gridpoint
 *
 * @param point the absolute coordinate of the scanpoint
 *
 * @return the matching coordiante of the gridpoint
 */
long scanToGrid::scaleToGrid(double point)
{
    return (long)(point / this->resolution);
}

/**
 * The methods calculates the norm vector of the vector given
 * through x and z.
 * The normvector has one direction oflength 1 or -1  and the other
 * length in [-1,1]
 * @param x The x component of the vector
 * @param z the z component of the vector
 * @param xnorm Referenz to the double containing the normalized x value afterwards
 * @param znorm Referenz to the double containing the normalized z value afterwards
 */
void scanToGrid::calculateNormvector(long x, long z,
				     double& xnorm, double& znorm)
{
    xnorm = 0;
    znorm = 0;

    // if |X| < |Z|, then norm z to 1 and x to x/z
    if(abs(x) < abs(z))
    {
	if(z == 0)
	    return;
	   
	xnorm = (double)x / (double)abs(z);
	znorm = z / abs(z); // now znorm is 1 or -1
    }
    else
    {
	if(x == 0)
	    return;

	xnorm = x / abs(x); // now xnorm is 1 or -1
	znorm = (double)z / (double)abs(x); 
    }
}

/**
 * The method creates a new grid. It determines the size and offset 
 * of the new grid and calculates the viewpoint of the roboter during
 * the scan.
 * The new scangrid is allocated using new, but the pointer is not stored, 
 * so it must be freed by the calling method.
 *
 * The scan has to be transformed to absolute values before this method is
 * called
 *
 * @param scan the scan to be transformed
 * @param transformation the transformationmatrix needed for calculating
 *                       the viewpoint of the roboter
 *
 * @return pointer to the created grid.
 *         (Just gets allocated, does not get freed)
 */ 
scanGrid* scanToGrid::createGrid(Scan& scan,
						   const double* transformation)
{
    // Calculate the viewpoint of the scan
    double rPos[3];
    double rPosTheta[3];

    Matrix4ToEuler(transformation, rPosTheta, rPos);
 
    long vpX = scaleToGrid(rPos[0]);
    long vpZ = scaleToGrid(rPos[2]); 

    // calculate maximal and minimal coordiantes
    double maxX = 0;
    double maxZ = 0;
    double minX = 0;
    double minZ = 0;

    DataXYZ xyz(scan.get("xyz"));

    for(size_t i = 0; i < xyz.size(); ++i)
    {
	 Point it = xyz[i];
	
        // If the scan is not relevant, skip it
	if(!isPointRelevant(it))
	    continue;	

	if(it.x < minX) minX = it.x;
	if(it.x > maxX) maxX = it.x;
	if(it.z < minZ) minZ = it.z;
	if(it.z > maxZ) maxZ = it.z;

    }
    
    // get offsets and sizes 
    long offsetX = scaleToGrid(minX);
    long offsetZ = scaleToGrid(minZ);
    long sizeX = scaleToGrid(maxX) - offsetX + 1;
    long sizeZ = scaleToGrid(maxZ) - offsetZ + 1;

    //returning new grid
    return new scanGrid(vpX, vpZ,
			offsetX, offsetZ,
			sizeX, sizeZ);
}

/**
 * 
 * !NOTE! Experimental. Currently this is not used! !NOTE! 
 *
 * The method iterates through the grid and checks for each point
 * if it has a specified amount of neighbours. If not enough neighbours
 * are found, the point is deleted.
 * 
 * @param grid The grid to check
 * @param distance 
 * @param neighbours The amount of neighbours needed for not getting deleted
 */
void scanToGrid::killAlonePoints(scanGrid* grid,
				 int distance, int neighbours)
{
    std::vector< gridPoint* > exPoints;

    
    for(int i=0; i < grid->getSizeX(); ++i)
    {
	for(int j=0; j < grid->getSizeZ(); ++j)
	{
	    int found = 0;
	    
	    for(int a=i - distance; a < i + distance; ++a)
	    {
		for(int b=i - distance; b < i + distance; ++b)
		{
		    if(a > 0 && b > 0 &&
		       a < grid->getSizeX() && b < grid->getSizeZ())
		    {
			if(grid->points[a][b]->getPercent() > 0)
			{
			    ++found;
			}
		    }
		    
		}
	    }
	    
	    // not enough neigbours found
	    if(found < neighbours)
	    {
		exPoints.push_back(grid->points[i][j]);
	    }
	    
	}
    }
    
    vector<gridPoint*>::iterator it = exPoints.begin();
    vector<gridPoint*>::iterator end = exPoints.end();

    while(it != end)
    {
	//std::cout << "Deleting " << (*it)->getX() << " " << (*it)->getZ() << " " << std::endl;
	(*it)->setFixed(0, 0);
	++it;
    }
}

/**
 * The method adds additional information about the neighbours of
 * the given point based on some sort of probabilityfunction.
 * Given the degree of the scanner, the function calculates the insecure 
 * points and weights them 
 *
 * @param grid The grid to add the information to
 * @param x The absolute x coordinate
 * @param z The absolute z coordinate
 * @param weighting The weighting of the point
 */
void scanToGrid::createNeighbours(scanGrid *grid, long x, long z, float weighting) 
{
    // Calculate direction of the neighbours
    long dx = - (grid->getViewpointZ() - z);
    long dz = grid->getViewpointX() - x;
    
    // calculating the normvector of them
    double xnorm, znorm;
    calculateNormvector(dx, dz, xnorm, znorm);

    // calculating number of neighbours to weight    
    int r = (int) (sqrt((double)(dx*dx + dz*dz)) * this->spot);

    for(int i = -r; i <= r; ++i)
    { 
	// Skip 0 (0 is the actual point on the vector)
	if(i == 0)
	    continue;

	long px = x + (int) (i * xnorm);
	long pz = z + (int) (i * znorm);

	// if calculated point not in grid, skip
	if( !grid->contains(px, pz))
	    continue;

	float weight = weighting / pow(2.0, abs(i*r));

	//	grid->addPoint(px, pz,
	//       SOLIDWEIGHT * weight, SOLIDWEIGHT * weight);
	grid->addPoint(px, pz, SOLIDWEIGHT, SOLIDWEIGHT);

	if(this->waypoints)
	    createWaypoints(grid, px, pz, weight);	
    }
}
   
  
/**
 * The method calculates the free waypoints between the robot and
 * the occupied point and adds them to the grid.
 * The way is weighted with the weighting-parameter to avoid
 * that a path overwrites found obstacles
 *
 * @param grid pointer to the grid to which the points will be added
 * @param x the x-coordinate of the occupied point
 * @param z the z-coordinate of the occupied point
 * @param weighting The weighting of the point on which the path is based.
 */
void scanToGrid::createWaypoints(scanGrid *grid, long x, long z, float weighting)
{
    if(x == 0 || z == 0)
	return;

    long dx = x - grid->getViewpointX();
    long dz = z - grid->getViewpointZ();

    // calculating the normal vector
    double xnorm, znorm;
    calculateNormvector(dx, dz, xnorm, znorm);

    int distance = (int)floor(sqrt(pow((double)dx, 2) + pow((double)dz, 2)) /
			      sqrt(pow(xnorm, 2) + pow(znorm, 2)));


    for(int i=0; i < distance; ++i)
    {
	long px = grid->getViewpointX() + (long)(i * xnorm);
	long pz = grid->getViewpointZ() + (long)(i * znorm);

	// due to some problems with the viewpoint lieing outside
        // of the grid, first check if point is in grid
	if(grid->contains(px, pz))
	  //grid->addPoint(px, pz, (int)(WAYPOINTWEIGHT * weighting), 0);
	  grid->addPoint(px, pz, WAYPOINTWEIGHT, 0);
    }  
}

/**
 * The method adds the point represented by the 2 coordinates. It weights
 * the point based on weighting.
 *
 * @param grid The grid to add the point to
 * @param x The x coordinate (scaled to grid)
 * @param z The z coordinate (scaled to grid)
 * @param weighting Importance of the point (0 <= w <= 1)
 */
void scanToGrid::createPoint(scanGrid *grid, long x, long z, float weighting)
{
  //    grid->addPoint(x, z, SOLIDWEIGHT, (int)(SOLIDWEIGHT * weighting));    
  grid->addPoint(x, z, SOLIDWEIGHT, SOLIDWEIGHT);
}

/**
 * Converts a scan(3D) to a grid(2D). It iterates through each
 * found point of the scan, translates it to the grid and adds it
 * to the grid. Points that are somehow not relevant to the grid (e.g. points
 * that are to low or to high are ignored. See isPointRelevant)
 *
 * If waypoints is true, it will also create waypoints from the origin of
 * the scan to the found point and mark the space in between as free.
 * If Neighbours should be created, it will also weight the surrounding 
 * spaces of the found point.
 *
 * @param scan The scan to be converted
 * @param transformation The transformationmatrix
 * @return pointer to the grid
 */ 
scanGrid* scanToGrid::convert(Scan& scan, const double* transformation)
{
    scanGrid* grid = createGrid(scan, transformation);
 
    DataXYZ xyz(scan.get("xyz"));

    // go through all points and create the grid
    for(size_t i = 0; i < xyz.size(); ++i)
    {
	 Point p = xyz[i];

	// if the scan is in the relevant area, create Point for it
	if(isPointRelevant(p))
	{
	    long x = scaleToGrid(p.x);
	    long z = scaleToGrid(p.z);

	     float weighting = calculateWeighting(x - grid->getViewpointX(),
						  z - grid->getViewpointZ());
	     createPoint(grid, x, z, weighting);
	     
	     if(this->waypoints)
		 createWaypoints(grid, x, z, weighting);

	     if(this->neighbours)
		 createNeighbours(grid, x, z, weighting);
	     
	}
    }

    // Kill all points which have less then 8 out of 25 neighbours
    /*killAlonePoints(grid, 3, 1);

    for(int i = 0; i < grid->getSizeX(); ++i)
    {
	for(int j = 0; j < grid->getSizeZ(); ++j)
	{
	    gridPoint *p = grid->points[i][j];
	    
	    float weighting = calculateWeighting(p->getX() - grid->getViewpointX(), p->getZ() - grid->getViewpointZ());
	    
	     if(this->waypoints)
		createWaypoints(grid, p->getX(), p->getZ(), weighting);

	     if(this->neighbours)
		 createNeighbours(grid, p->getX(), p->getZ(), weighting);
	}
	}*/

    return grid;
}
