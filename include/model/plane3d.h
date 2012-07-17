/**
 * @file plane3d.h
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 16 Feb 2012
 *
 */

#ifndef PLANE3D_H_
#define PLANE3D_H_

//==============================================================================
//  Includes
//==============================================================================
#include "model/vector3d.h"

#include <vector>

namespace model {

/**
 * A class defining a plane.
 */
class Plane3d {
private:
    /**
     * used to determine if vertical
     */
    static constexpr double _EPSILON_VERTICAL   = 0.02;

    /**
     * used to determine if horizontal
     */
    static constexpr double _EPSILON_HORIZONTAL = 0.02;

    /**
     * if two planes are this close in cm
     * then they may be considered to be the same
     */
    static constexpr double _EPSILON_DISTANCE = 20;

    /**
     * if the length of the crossproduct of two normals is less than this
     * then the planes may be considered to be the same
     */
    static constexpr double _EPSILON_NORMAL_CROSS_PRODUCT = 2e-1;

public:
    /**
     * the normal defining the plane
     */
    Vector3d normal;

    /**
     * the point of application of the normal
     */
    Point3d pt;

    /**
     * the convex hull of this plane
     */
    std::vector<Point3d> hull;

    Plane3d();
    Plane3d(const Point3d& pt, const Vector3d& normal);
    Plane3d(const Point3d& pt, const Vector3d& normal,
            const std::vector<Point3d>& hull);

    Plane3d(const Plane3d& other);

    virtual ~Plane3d();

    /**
     * compute the intersection of three planes
     */
    Point3d intersect(const Plane3d& first, const Plane3d& second);

    /**
     * copies one plane to another
     */
    Plane3d& operator=(const Plane3d& other);

    /**
     * returns true if a plane is vertical within some predefined epsilon values
     */
    bool isVertical();

    /**
     * returns true if a plane is horizontal within some predefined epsilon values
     */
    bool isHorizontal();

    /**
     * returns true if this plane replresents the same plane as other
     * within some predefined epsilon values
     */
    bool isSamePlane(Plane3d other);

    /**
     * Computes the normal of the plane from the hull.
     */
    Vector3d computeAverageNormal();

    /**
     * Returns the 3D coordinates of points on the plane, which have a fixed
     * distance to their next neighbor.
     */
    std::vector<std::vector<Point3d> > getDiscretePoints(const double& dist) const;
};

} /* namespace model */

#endif /* PLANE3D_H_ */
