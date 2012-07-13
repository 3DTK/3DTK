/**
 * @file vector3d.h
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 16 Feb 2012
 *
 */

#ifndef VECTOR3D_H_
#define VECTOR3D_H_

//==============================================================================
//  Includes
//==============================================================================
#include "model/point3d.h"

#include <ostream>

namespace model {

/**
 * A vector in space.
 */
class Vector3d {
public:
    double x;
    double y;
    double z;

    Vector3d();

    Vector3d(const double& x, const double& y, const double& z);

    /**
     * constructor for vector b - a, where a and b are points
     */
    Vector3d(const Point3d& a, const Point3d& b);

    Vector3d(const Vector3d& other);

    Vector3d& operator=(const Vector3d& other);

    /**
     * normalizes the vector
     * @warning in place
     */
    void normalize();

    /**
     * inverts the vector
     * @warning in place
     */
    void invert();

    /**
     * @return the length of the vector
     */
    double length() const;

    /**
     * rotates the vector around the origin by the given rotation
     * @param rot the rotation object that describes how much to rotate the vector
     * @warning in place
     */
    void rotate(const Rotation3d& rot);

    /**
     * @return the cross product (still a vector) between this vector and another given one
     */
    Vector3d crossProduct(const Vector3d& other) const;

    /**
     * @return the dot product between this vector and another given one
     */
    double dotProduct(const Vector3d& other) const;

    /**
     * @return the angle between two vectors
     */
    double angle(const Vector3d& other);

    void operator+=(const Vector3d& other);
    Vector3d operator+(const Vector3d& other);
    Vector3d operator*(const double& val);
    void operator/=(const double& div);

    friend std::ostream& operator<<(std::ostream& os, const Vector3d& vect);
};

} /* namespace model */

#endif /* VECTOR3D_H_ */
