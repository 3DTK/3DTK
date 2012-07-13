/**
 * @file vector3d.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 16 Feb 2012
 *
 */

//==============================================================================
//  Includes
//==============================================================================
#include "model/vector3d.h"

#include "model/util.h"
#include <math.h>

using namespace model;

//==============================================================================
//  Implementation
//==============================================================================

model::Vector3d::Vector3d() {
    // nothing to do
}

model::Vector3d::Vector3d(const double& x, const double& y, const double& z) {
    this->x = x;
    this->y = y;
    this->z = z;
}

// constructor for vector b - a, where a and b are points
model::Vector3d::Vector3d(const Point3d& a, const Point3d& b) {
    this->x = b.x - a.x;
    this->y = b.y - a.y;
    this->z = b.z - a.z;
}

model::Vector3d::Vector3d(const Vector3d& other) {
    this->x = other.x;
    this->y = other.y;
    this->z = other.z;
}

model::Vector3d& model::Vector3d::operator=(const model::Vector3d& other) {
    if (this != &other) {
        this->x = other.x;
        this->y = other.y;
        this->z = other.z;
    }

    return *this;
}

// normalizes the vector
void model::Vector3d::normalize() {
    // squares
    double sx = sqr(this->x);
    double sy = sqr(this->y);
    double sz = sqr(this->z);

    double len = sqrt(sx + sy + sz);

    // avoid a division by zero
    if (len == 0.0) {
        len = 1.0;
    }

    // normalize the values
    this->x /= len;
    this->y /= len;
    this->z /= len;

    // increase precision, this accounts for the lost digits
    this->x = round(this->x * 10000) / 10000;
    this->y = round(this->y * 10000) / 10000;
    this->z = round(this->z * 10000) / 10000;
}

// inverses the vector
void model::Vector3d::invert() {
    this->x *= (-1);
    this->y *= (-1);
    this->z *= (-1);
}

// get the length
double model::Vector3d::length() const {
    return sqrt(sqr(this->x) + sqr(this->y) + sqr(this->z));
}

// rotation of a vector is like a point around 0 0 0
void model::Vector3d::rotate(const Rotation3d& rot) {
    // create rotation matrices
    double Rx[3][3] = {
            {1.0,   0.0,            0.0},
            {0.0,   +cos(rot.x),    -sin(rot.x)},
            {0.0,   +sin(rot.x),    +cos(rot.x)}
    };

    double Ry[3][3] = {
            {+cos(rot.y),   0.0,    +sin(rot.y)},
            {0.0,           1.0,    0.0},
            {-sin(rot.y),   0.0,    +cos(rot.y)}
    };

    double Rz[3][3] = {
            {+cos(rot.z),   -sin(rot.z),    0.0},
            {+sin(rot.z),   +cos(rot.z),    0.0},
            {0.0,           0.0,            1.0}
    };

    // convert to something easier to use
    double point[3];
    point[0] = this->x;
    point[1] = this->y;
    point[2] = this->z;

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

    // save the results
    this->x = point[0];
    this->y = point[1];
    this->z = point[2];
}

// computes the cross product between this vector and another given one
model::Vector3d model::Vector3d::crossProduct(const Vector3d& other) const {
    Vector3d ret;

    double a1 = this->x;
    double a2 = this->y;
    double a3 = this->z;

    double b1 = other.x;
    double b2 = other.y;
    double b3 = other.z;

    ret.x = a2 * b3 - a3 * b2;
    ret.y = a3 * b1 - a1 * b3;
    ret.z = a1 * b2 - a2 * b1;

    return ret;
}

// computes the dot product between this vector and another given one
double model::Vector3d::dotProduct(const Vector3d& other) const {
    return this->x * other.x +
            this->y * other.y +
            this->z * other.z;
}

// FIXME use atan2, computes the angle between two vectors
double model::Vector3d::angle(const Vector3d& other) {
    double len1 = this->length();
    double len2 = other.length();

    if (len1 == 0 || len2 == 0) {
        return 0;
    }

    return acos(
            this->dotProduct(other) / (len1 * len2)
    );
}

void model::Vector3d::operator+=(const Vector3d& other) {
    this->x += other.x;
    this->y += other.y;
    this->z += other.z;
}

model::Vector3d model::Vector3d::operator+(const Vector3d& other) {
    Vector3d result;

    result.x = this->x + other.x;
    result.y = this->y + other.y;
    result.z = this->z + other.z;

    return result;
}

model::Vector3d model::Vector3d::operator*(const double& val) {
    Vector3d result;

    result.x = this->x * val;
    result.y = this->y * val;
    result.z = this->z * val;

    return result;
}

void model::Vector3d::operator/=(const double& div) {
    this->x /= div;
    this->y /= div;
    this->z /= div;
}


std::ostream& model::operator<<(std::ostream& os, const model::Vector3d& vect) {
    os << vect.x << " " << vect.y << " " << vect.z;
    return os;
}
