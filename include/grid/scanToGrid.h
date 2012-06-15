#ifndef __SCANTOGRID_H_
#define __SCANTOGRID_H_

#include "grid/scanGrid.h"
#include "slam6d/scan.h"
#include "slam6d/globals.icc"

#define WAYPOINTWEIGHT 10
#define SOLIDWEIGHT 5000

/** 
 * The class provides methods to create a 2D scanGrid from a given scan.
 * It sorts out points not relevant for the grid and tries to
 * reduce noise within the data.
 * It also marks free spaces between found obstacles.
 * 
 * @author Sebastian Stock, Uwe Hebbelmann, Andre Schemschat
 * @date 11.02.2008
 */
class scanToGrid
{
 private:
    /** The minimal height of a scan that is relevant */
    double minRelevantHeight;

    /** The maximal height of a scan that is relevant */
    double maxRelevantHeight;

    /** The resolution of the grid (centimeters per gridunit) */
    double resolution;

    /** The maximal distance */
    int maxDistance;
    
    /** The distance weighting  */
    double minimalWeighting; 

    /** the spot-value of the laser */
    float spot;

    /** If set, ways between the viewpoint and found point will be created */
    bool waypoints;

    /** If set, neighbours for each point will be weighted */
    bool neighbours;

    /**
     * The method calculates the weighting of a Point
     * based on its distance and an internal formula.
     * 
     * @param distance The distance of the point to the robots position
     * @return The calculated weighting for the point
     */    
    inline float calculateWeighting(long x, long z)
    {
	double distance = sqrt(pow((double)x, 2) + pow((double)z, 2));
	return this->minimalWeighting * distance + 1; 
    }

    /** @brief Calculates the normvector for the vector (x|z) */
    void calculateNormvector(long x, long z, double& xnorm, double &znorm);

    /** @brief Creates a new grid and sets the offsets */
    scanGrid* createGrid(Scan& scan, const double* transformation);

    /** @brief Creates the free points between the robot and the occupied point */
    void createWaypoints(scanGrid* grid, long x, long z, float weighting);

    /** @brief Creates the neighbours for the given point */
    void createNeighbours(scanGrid *grid, long x, long z, float weighting);

    /** @brief Method adds a point (and maybee its neighbours) to the grid */
    void createPoint(scanGrid* grid, long x, long z, float weighting);

    /** @brief Method checks the grid for values which stand alone and deletes them */
    void killAlonePoints(scanGrid* grid, int distance, int neighbours);
 
    /** @brief Checks if the points lies within the relevant area */
    bool isPointRelevant(const Point& p) const;

    /** @brief Converts the coordinate of a scanpoint to the grid raster */
    long scaleToGrid(double point);
        
 public:
    /** @brief Ctor*/
    scanToGrid(double resolution, double minRelevantHeight, 
	       double maxRelevantHeight, int maxDistance ,
	       int spotRadius, bool createWays,
	       bool neighbours);

    /** @brief Converts the scan to a grid */
    scanGrid* convert(Scan& scan, const double* transformation);
};

#endif
