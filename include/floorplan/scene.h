/**
 * @file scene.h
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 13 Feb 2012
 *
 */

#ifndef SCENE_H_
#define SCENE_H_

//==============================================================================
//  Includes
//==============================================================================
// User includes.
#include "floorplan/point3d.h"
#include "floorplan/vector3d.h"
#include "floorplan/commonTypes.h"
#include "slam6d/io_types.h"
#include "slam6d/Boctree.h"

// C++ includes.
#include <vector>
#include <utility>
#include <iostream>
#include <limits>

// OpenCV includes.
#include <opencv2/opencv.hpp>

//==============================================================================
//  Global variables.
//==============================================================================
extern bool quiet;

//==============================================================================
//  Class declaration.
//==============================================================================
namespace floorplan {

/**
 * A class defining a 3d scene.
 */
class Scene {
public:
    // Public fields.
    std::vector<Point3d> points;       //!< The 3d point cloud.
    std::vector<Pose6d> poses;         //!< Container for all the poses from where scans have been taken from.

    SearchTree *octTree;               //!< An efficient octree containing the points.
    double **octTreePoints;            //!< Used to construct the octree.
    size_t nrPoints;                   //!< The total number of points in the octree.

private:
    /**
     * Computes a 3D boolean occupancy grid.
     */
    std::vector<std::vector<std::vector<bool> > > compute3DOccGrid(const double& voxDimInCm,
            const double& minHeight = -std::numeric_limits<double>::max(),
            const double& maxHeight = +std::numeric_limits<double>::max());

    static const int MIN_CROSS_SECTION_COUNT; // How many cross-sections to take for the optimum 2D histogram.

protected:
    // Protected methods.
    Scene();

public:
    // Public methods.
    inline Scene(const std::vector<Point3d>& points) : points(points) {
        if (!quiet) std::cout << "== Creating scene..." << std::endl;
        this->nrPoints = 0;

        this->octTree = NULL;
        this->octTreePoints = NULL;
    };

    Scene(const IOType& type,
		const int& start, const int& end,
		std::string dir, const bool& scanserver,
		const int& maxDist, const int& minDist,
		const int& octree, const double& red);

    Scene(const Scene& other);
    ~Scene();

    /**
     * Returns true if the cube centered at the given coordinates is occupied.
     */
    bool isOccupied(const Point3d& center, const double& width);

    /**
     * Computes the extremities of the scene.
     */
    void findExtremities(std::pair<double, double>& xtrX, std::pair<double, double>& xtrY, std::pair<double, double>& xtrZ);

    /**
     * Computes the projection of the point cloud on the vertical axis.
     * @param bucketHeighInCm the height of a histogram bucket given in cm
     */
    std::vector<int> computeVerticalHist(const double& bucketHeighInCm);

    /**
     * Compute the 2D histogram by projecting onto the horizontal plane.
     */
    cv::Mat computeHorizontalHist(const double& voxDimInCm,
            const bool& useBest = false, const bool& unreliableWalls = false);

};

} /* namespace model */

#endif /* SCENE_H_ */
