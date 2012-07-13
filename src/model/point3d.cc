/**
 * @file point3d.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 16 Feb 2012
 *
 */

//==============================================================================
//  Defines
//==============================================================================


//==============================================================================
//  Includes
//==============================================================================
#include "model/point3d.h"

#include "model/plane3d.h"
#include "model/util.h"

#include <math.h>

//==============================================================================
//  Implementation
//==============================================================================

model::Point3d::Point3d() {
    this->x = 0.0;
    this->y = 0.0;
    this->z = 0.0;
}

model::Point3d::Point3d(const double& x, const double& y, const double& z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

model::Point3d::Point3d(const Point3d& other) {
    this->x = other.x;
    this->y = other.y;
    this->z = other.z;
}

// rotate a point around a given point
void model::Point3d::rotate(const Point3d& ref, const Rotation3d& rot)
{
    // create rotation matrices
    double Rx[3][3] = {
            {1.0,    0.0,            0.0},
            {0.0,    cos(rot.x),     -sin(rot.x)},
            {0.0,    sin(rot.x),     cos(rot.x)}
    };

    double Ry[3][3] = {
            {cos(rot.y),    0.0,            sin(rot.y)},
            {0.0,           1.0,            0.0},
            {-sin(rot.y),   0.0,            cos(rot.y)}
    };

    double Rz[3][3] = {
            {cos(rot.z),    -sin(rot.z),    0.0},
            {sin(rot.z),    cos(rot.z),     0.0},
            {0.0,           0.0,            1.0}
    };

    // we need to translate the point in the origin, do rotation, and then translate back
    double point[3];
    point[0] = this->x - ref.x;
    point[1] = this->y - ref.y;
    point[2] = this->z - ref.z;

    // somewhere to save the result
    double temp[3];

    // do rotation around z axis
    for (int i = 0; i < 3; ++i) {
        // reset the sum
        double sum = 0.0;

        for (int j = 0; j < 3; ++j) {
            sum += (Rz[i][j] * point[j]);
        }

        // save the result
        temp[i] = sum;
    }

    // move results for next rotation
    for (int i = 0; i < 3; ++i) {
        point[i] = temp[i];
    }

    // do rotation around y axis
    for (int i = 0; i < 3; ++i) {
        // reset the sum
        double sum = 0.0;

        for (int j = 0; j < 3; ++j) {
            sum += (Ry[i][j] * point[j]);
        }

        // save the result
        temp[i] = sum;
    }

    // move results for next rotation
    for (int i = 0; i < 3; ++i) {
        point[i] = temp[i];
    }

    // do rotation around z axis
    for (int i = 0; i < 3; ++i) {
        // reset the sum
        double sum = 0.0;

        for (int j = 0; j < 3; ++j) {
            sum += (Rx[i][j] * point[j]);
        }

        // save the result
        temp[i] = sum;
    }

    // move results for next rotation
    for (int i = 0; i < 3; ++i) {
        point[i] = temp[i];
    }

    // now translate back to reference point
    this->x = point[0] + ref.x;
    this->y = point[1] + ref.y;
    this->z = point[2] + ref.z;
}

void model::Point3d::translate(const Vector3d& vec) {
    this->x += vec.x;
    this->y += vec.y;
    this->z += vec.z;
}

// the distance from this point to another one
double model::Point3d::distance(const Point3d& other) const {
    return sqrt(sqr(other.x - this->x)
            + sqr(other.y - this->y)
            + sqr(other.z - this->z));
}

// the distance from this point to the given plane
double model::Point3d::distance(const Plane3d& plane) const {
    double vect[3];
    vect[0] = plane.pt.x - this->x;
    vect[1] = plane.pt.y - this->y;
    vect[2] = plane.pt.z - this->z;

    return vect[0] * plane.normal.x +
            vect[1] * plane.normal.y +
            vect[2] * plane.normal.z;
}

model::Point3d& model::Point3d::operator=(const Point3d& other) {
    if (this != &other) {
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
    }

    return *this;
}

model::Point3d model::Point3d::operator-(const Point3d& other) {
    Point3d result;
    result.x = this->x - other.x;
    result.y = this->y - other.y;
    result.z = this->z - other.z;
    return result;
}

model::Point3d model::Point3d::operator*(const double& coef) {
    Point3d result;
    result = *this;
    result.x *= coef;
    result.y *= coef;
    result.z *= coef;
    return result;
}

model::Point3d model::Point3d::operator/(const double& div) {
    Point3d result;
    result.x = this->x / div;
    result.y = this->y / div;
    result.z = this->z / div;
    return result;
}

void model::Point3d::operator/=(const double& div) {
    this->x /= div;
    this->y /= div;
    this->z /= div;
}

void model::Point3d::operator+=(const Point3d& other) {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
}

void model::Point3d::operator-=(const Point3d& other) {
    this->x -= other.x;
    this->y -= other.y;
    this->z -= other.z;
}

void model::Point3d::operator*=(const double& coef) {
    this->x *= coef;
    this->y *= coef;
    this->z *= coef;
}


bool model::Point3d::operator==(const Point3d& other) const {
    return (this->x + _EPSILON >= other.x && this->x - _EPSILON <= other.x) &&
            (this->y + _EPSILON >= other.y && this->y - _EPSILON <= other.y) &&
            (this->z + _EPSILON >= other.z && this->z - _EPSILON <= other.z);
}

bool model::Point3d::operator!=(const Point3d& other) {
    if (*this == other) {
        return false;
    }
    return true;
}

std::ostream& model::operator<<(std::ostream& os, const model::Point3d& pt) {
    os << pt.x << " " << pt.y << " " << pt.z;
    return os;
}
