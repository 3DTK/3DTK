/**
 * @file plane3d.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 16 Feb 2012
 *
 */

//==============================================================================
//  Includes
//==============================================================================
#include "model/plane3d.h"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/LU>
using namespace Eigen;

#include <math.h>

#include <iostream>
#include <stdexcept>
using namespace std;

model::Plane3d::Plane3d() {
    // nothing to do
}

model::Plane3d::Plane3d(const Point3d& pt, const Vector3d& normal) {
    this->pt     = pt;
    this->normal = normal;

    // normals must be normalized
    this->normal.normalize();
}


model::Plane3d::Plane3d(const Point3d& pt, const Vector3d& normal,
        const vector<Point3d>& hull)
{
    this->pt     = pt;
    this->normal = normal;
    this->hull   = hull;

    // normals must be normalized
    this->normal.normalize();
}

model::Plane3d::Plane3d(const Plane3d& other) {
    this->pt     = other.pt;
    this->normal = other.normal;
    this->hull   = other.hull;
}

model::Plane3d::~Plane3d() {
    // nothing to do
}

model::Point3d model::Plane3d::intersect(const Plane3d& second, const Plane3d& third) {
    // XXX http://mathworld.wolfram.com/Plane-PlaneIntersection.html
    Point3d result;
    Plane3d first = *this;

    MatrixXd mat(3, 3);
    mat << first.normal.x, second.normal.x, third.normal.x,
           first.normal.y, second.normal.y, third.normal.y,
           first.normal.z, second.normal.z, third.normal.z;

    Vector3d term[3];
    term[0] = second.normal.crossProduct(third.normal) *
            first.normal.dotProduct(Vector3d(first.pt.x, first.pt.y, first.pt.z));

    term[1] = third.normal.crossProduct(first.normal) *
            second.normal.dotProduct(Vector3d(second.pt.x, second.pt.y, second.pt.z));

    term[2] = first.normal.crossProduct(second.normal) *
            third.normal.dotProduct(Vector3d(third.pt.x, third.pt.y, third.pt.z));

    Vector3d tempResult = (term[0] + term[1] + term[2]) * (1.0 / mat.determinant());

    result.x = tempResult.x;
    result.y = tempResult.y;
    result.z = tempResult.z;

    return result;
}

model::Plane3d& model::Plane3d::operator=(const Plane3d& other) {
    this->pt     = other.pt;
    this->normal = other.normal;
    this->hull   = other.hull;

    return *this;
}

bool model::Plane3d::isVertical() {
    this->normal.normalize();
    return (fabs(this->normal.y) < _EPSILON_VERTICAL);
}

bool model::Plane3d::isHorizontal() {
    this->normal.normalize();
    return ((1 - fabs(this->normal.y / this->normal.length())) < _EPSILON_HORIZONTAL);
}

bool model::Plane3d::isSamePlane(model::Plane3d other) {
    // just to be safe
    this->normal.normalize();
    other.normal.normalize();

    double dist = this->pt.distance(other);
    double norm = this->normal.crossProduct(other.normal).length();

    // doing fabs of norm just to be safe, though it is always positive
    if (fabs(dist) < _EPSILON_DISTANCE && fabs(norm) < _EPSILON_NORMAL_CROSS_PRODUCT) {
        return true;
    }

    return false;
}

model::Vector3d model::Plane3d::computeAverageNormal() {
    if(this->hull.size() < 3) {
        throw runtime_error("hull contains less than 3 points, cannot compute normal");
    }

    // start from zero
    Vector3d normal(0.0, 0.0, 0.0);

    for (vector<Point3d>::iterator it = this->hull.begin() + 1; it != this->hull.end() - 1; ++it) {
        Point3d& left = *(it-1);
        Point3d& middle = *it;
        Point3d& right = *(it+1);

        normal += Vector3d(middle, left).crossProduct(Vector3d(middle, right));
    }

    // treat the boundary cases
    Point3d& first      = this->hull[0];
    Point3d& second     = this->hull[1];
    Point3d& secondLast = this->hull[this->hull.size() - 2];
    Point3d& last       = this->hull[this->hull.size() - 1];

    normal += Vector3d(first, last).crossProduct(Vector3d(first, second));
    normal += Vector3d(last, secondLast).crossProduct(Vector3d(last, first));

    normal.normalize();
    return normal;
}

vector<vector<model::Point3d> > model::Plane3d::getDiscretePoints(const double& dist) const {
    if (this->hull.size() < 3) {
        throw runtime_error("cannot retrieve discrete points on plane with hull containing less than 3 points");
    }

    // make a deep copy of the current plane
    Plane3d copy = *this;

    // this will eventually contain the result
    vector<vector<model::Point3d> > result;

    // the origin of the current system, take the first point in the hull
    Point3d origin = copy.hull[0];
    Point3d zero(0.0, 0.0, 0.0);

    // define the rotation around Z and X axis here
    Rotation3d rotZ(0.0, 0.0, atan2(copy.normal.x, copy.normal.y));
    copy.normal.rotate(rotZ);
    Rotation3d rotX(-atan2(copy.normal.z, copy.normal.y), 0.0, 0.0);

    // translate all points to origin
    for (vector<Point3d>::iterator it = copy.hull.begin(); it != copy.hull.end(); ++it) {
        // translate
        *it -= origin;

        // rotate around Z
        it->rotate(zero, rotZ);

        // rotate around X
        it->rotate(zero, rotX);
    }

    // figure out how to rotate around Y
    Vector3d firstSegment;

    Point3d first = copy.hull.front();
    Point3d second = copy.hull[1];
    Point3d third = copy.hull[2];

    if (first.distance(second) > second.distance(third)) {
        firstSegment = Vector3d(first, second);
    } else {
        firstSegment = Vector3d(second, third);
    }

    // define rotation around Y axis after we have moved the entire hull into the XZ plane
    copy.normal.rotate(rotX);
    Rotation3d rotY(0.0, atan2(firstSegment.z, firstSegment.x), 0.0);

    for (vector<Point3d>::iterator it = copy.hull.begin(); it != copy.hull.end(); ++it) {
        // rotate around Y
        it->rotate(zero, rotY);
    }

    // we need to compute the maximum and minimum offsets
    double minX = copy.hull[0].x;
    double maxX = copy.hull[0].x;
    double minZ = copy.hull[0].z;
    double maxZ = copy.hull[0].z;

    // we need to start from the second point
    for (vector<Point3d>::iterator it = copy.hull.begin() + 1;
            it != copy.hull.end(); ++it)
    {
        if (it->x < minX) {
            minX = it->x;
        }

        if (it->x > maxX) {
            maxX = it->x;
        }

        if (it->z < minZ) {
            minZ = it->z;
        }

        if (it->z > maxZ) {
            maxZ = it->z;
        }
    }

    // how many steps on each direction?
    int stepsX = round((maxX - minX) / dist) + 2;
    int stepsZ = round((maxZ - minZ) / dist) + 2;

    // resize the result vector
    result.resize(stepsZ);

    for (int j = 0; j < stepsZ; ++j) {
        for (int i = 0; i < stepsX; ++i) {
            Point3d toPush(minX + dist * (i - 0.5), 0.0, maxZ - dist * (j - 0.5));

            // rotate back around Y
            toPush.rotate(zero, rotY.getInverse());

            // rotate back around X
            toPush.rotate(zero, rotX.getInverse());;

            // rotate back around Z
            toPush.rotate(zero, rotZ.getInverse());

            //translate back to previous system
            toPush += origin;

            result[j].push_back(toPush);
        }
    }

    return result;
}
