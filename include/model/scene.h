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
#include "model/point3d.h"
#include "model/vector3d.h"
#include "model/labeledPlane3d.h"

#include "shapes/hough.h"
#include "shapes/shape.h"
#include "slam6d/Boctree.h"
#include "slam6d/io_types.h"

#include <vector>

//==============================================================================
//  Global Variables
//==============================================================================
extern bool quiet;

/**
 * All available plane detection algos.
 */
enum PlaneAlgorithm {
    RHT, SHT, PHT, PPHT, APHT, RANSAC
};

namespace model {

/**
 * A class defining a 3d scene.
 */
class Scene {
private:
    // private fields
    static const double PRECISION;       //!< Precision at which to draw the 3d Bresenham line.
    static const double RAY_DIST;        //!< The max distance to consider that a ray hit something.
    static const double PATCH_DIST;      //!< The distance between each discrete patch on a wall surface.
    static const double WALL_DIST;       //!< Tolerance to consider a point part of a wall, squared.
    static const double MIN_EMPTY_AREA;  //!< The minimum empty area percentage to be considered a window.
    static const double MIN_EDGE_COV;    //!< Minimum edge coverage coefficient.

public:
    // public fields
    SearchTree *octTree;               //!< An efficient octree containing the points.
    double **octTreePoints;            //!< Used to construct the octree.
    unsigned int nrPoints;             //!< The total number of points in the octree.

    std::vector<Point3d> points;       //!< The 3d point cloud.
    std::vector<Plane3d> planes;       //!< The list of planes in our scene.

    std::vector<LabeledPlane3d> walls; //!< The walls of the room.
    LabeledPlane3d ceiling;            //!< The ceiling of the room.
    LabeledPlane3d floor;              //!< The floor of the room.

    std::vector<Plane3d> finalOpenings;      //!< The windows of the room.

    std::vector<Pose6d> poses;         //!< Container for all the poses from where scans have been taken from.

protected:
    // protected methods
    Scene();

public:
    // public methods
    inline Scene(const std::vector<Point3d>& points,
                 const std::vector<Plane3d>& planes) :
            points(points),
            planes(planes)
    {
        if (!quiet) cout << "== Creating scene..." << endl;
        this->nrPoints = 0;
    };

    Scene(const IOType& type,
		const int& start, const int& end,
		std::string dir, const bool& scanserver,
		const int& maxDist, const int& minDist,
		const PlaneAlgorithm& alg, const int& octree, const double& red,
		const vector<Pose6d>& poses);

    Scene(const Scene& other);
    ~Scene();

    /**
     * Given some planes, returns the HORIZONTAL convex hull formed by the
     * application point inside the plane objects, only of VERTICAL planes.
     */
    static std::vector<Plane3d> getConvexHull(std::vector<Plane3d> planes);

    /**
     * Removes planes that have similar normals. [O(n^2)]
     * @warning the hull is not averaged, it is simply the first found hull for each plane
     */
    static std::vector<Plane3d> getSignificantPlanes(std::vector<Plane3d> planes);

    /**
     * Returns true if the highest horizontal plane has been found.
     */
    bool getCeiling(std::vector<Plane3d> planes, Plane3d& result);

    /**
     * Returns true if the lowest horizontal plane has been found.
     */
    bool getFloor(std::vector<Plane3d> planes, Plane3d& result);

    /**
     * Creates the walls, floor and ceiling.
     */
    void detectWalls();

    /**
     * Performs ray casting from source point to destination and returns true
     * if the ray successfully reached the destination point, returning the
     * point it hit along the way.
     */
    bool castRay(const model::Point3d& src, const model::Point3d& dest, const double& extraDist,
            Point3d& ptHit);

    /**
     * Applies labels to the given plane.
     */
    void applyLabels(LabeledPlane3d& surf);

    /**
     * Applies labels to all the walls.
     */
    void applyAllLabels();

    /**
     * Finds all openings using the SVM.
     */
    void detectPotentialOpenings(const LabeledPlane3d& surf,
            std::vector<CandidateOpening>& openings);

    /**
     * Applies KMeans clustering to the multiple windows yielding only one possible window for each cluster.
     * Does not work with OpenCV 2.4.0!!!
     */
    void clusterOpenings(const LabeledPlane3d& surf, const std::vector<CandidateOpening>& openings,
            std::vector<CandidateOpening>& result) const;

    /**
     * Returns all openings, after deciding which are the correct ones.
     */
    void addFinalOpenings(const LabeledPlane3d& surf,
            std::vector<CandidateOpening>& result);

    /**
     * Corrects an image to fill in missing data.
     */
    void correct(LabeledPlane3d& surf, const std::vector<CandidateOpening>& openings);

    /**
     * Writes the walls to appropriate files in the given folder.
     */
    void writeModel(std::string dir);

    /**
     * Writes the corrected walls to the given folder.
     */
    void writeCorrectedWalls(std::string dir);

    /**
     * Computes the corrected wall in UOS_RGB.
     */
    void getCorrectedWall(const LabeledPlane3d& surf, vector<pair<Point3d, cv::Vec3i> >& points);

private:
    // private methods

    /**
     * Floods a point on the labeling matrix with the other given value.
     */
    void flood(LabeledPlane3d& surf,
            const int& i, const int& j,
            const Label& target, const Label& replacement);

    /**
     * Returns true if the cube centered at the given coordinates is occupied.
     */
    bool isOccupied(const Point3d& center, const double& width);

};

} /* namespace model */

#endif /* SCENE_H_ */
