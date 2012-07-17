/**
 * @file point3d.h
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 16 Feb 2012
 *
 */

#ifndef POINT3D_H_
#define POINT3D_H_

//==============================================================================
//  Defines
//==============================================================================

//==============================================================================
//  Includes
//==============================================================================
#include "model/rotation3d.h"

#include <ostream>

namespace model {

// Forward declaration. Avoid circular inclusion.
class Plane3d;
class Vector3d;

/**
 * A point in space.
 */
class Point3d {
private:
    static constexpr double _EPSILON = 1.0e-3;

public:
    double x;
    double y;
    double z;

    Point3d();

    Point3d(const double& x, const double& y, const double& z);

    Point3d(const Point3d& other);

    /**
     * rotate this point around another point
     * @param ref the other point
     * @param rot the rotation around the other point
     */
    void rotate(const Point3d& ref, const Rotation3d& rot);

    /**
         * translate this point by a vector
         * @param vec the vector that defines the translation
         */
    void translate(const Vector3d& vec);

    /**
     * @return the distance from this point to another point
     */
    double distance(const Point3d& other) const;

    /**
     * @return the distance to a plane
     */
    double distance(const Plane3d& plane) const;

    Point3d& operator=(const Point3d& other);
    Point3d operator-(const Point3d& other);
    Point3d operator*(const double& coef);
    Point3d operator/(const double& div);
    void operator/=(const double& div);
    void operator+=(const Point3d& other);
    void operator-=(const Point3d& other);
    void operator*=(const double& coef);
    bool operator==(const Point3d& other) const;
    bool operator!=(const Point3d& other);

    friend std::ostream& operator<<(std::ostream& os, const Point3d& pt);
};


} /* namespace model */

#endif /* POINT3D_H_ */
