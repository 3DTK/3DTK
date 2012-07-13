/**
 * @file util.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 13 Feb 2012
 *
 */

//==============================================================================
//  Includes
//==============================================================================
#include "model/util.h"

#include <sys/stat.h>       // stat()
#include <unistd.h>
#include <math.h>

#include <stdexcept>
using namespace std;

//==============================================================================
//  Implementation
//==============================================================================
bool model::sameSide(const Point3d& p1,
        const Point3d& p2, const Point3d& a, const Point3d& b)
{
    // compute the required vectors
    Vector3d ba;
    ba.x = b.x - a.x;
    ba.y = b.y - a.y;
    ba.z = b.z - a.z;

    Vector3d p1a;
    p1a.x = p1.x - a.x;
    p1a.y = p1.y - a.y;
    p1a.z = p1.z - a.z;

    Vector3d p2a;
    p2a.x = p2.x - a.x;
    p2a.y = p2.y - a.y;
    p2a.z = p2.z - a.z;

    // do the cross products
    Vector3d cp1 = ba.crossProduct(p1a);
    Vector3d cp2 = ba.crossProduct(p2a);

    // compute the dotproduct
    double dotProduct = cp1.x * cp2.x + cp1.y * cp2.y + cp1.z * cp2.z;

    if (dotProduct >= 0) {
        return true;
    }
    else {
        return false;
    }
}

bool model::insideHull(const Point3d& pt, const vector<Point3d>& hull) {
    if (hull.size() <= 0) {
        throw runtime_error("hull cannot be empty");
    }

    Point3d center(0.0, 0.0, 0.0);

    for (unsigned int i = 0; i < hull.size(); ++i) {
        center.x += hull[i].x;
        center.y += hull[i].y;
        center.z += hull[i].z;
    }
    center = center / hull.size();

    for (unsigned int i = 0; i < hull.size() - 1; ++i) {
        if (sameSide(pt, center, hull[i], hull[i+1]) &&
                sameSide(pt, hull[i], center, hull[i+1]) &&
                sameSide(pt, hull[i+1], center, hull[i]))
        {
            return true;
        }
    }

    if (sameSide(pt, center, hull[hull.size()-1], hull[0]) &&
            sameSide(pt, hull[hull.size()-1], center, hull[0]) &&
            sameSide(pt, hull[0], center, hull[hull.size()-1]))
    {
        return true;
    }

    return false;
}

vector<model::Point3d> model::getHorizontalConvexHull(vector<Point3d> points) {
    if (points.size() <= 3) {
        return points;
    }

    vector<Point3d> hull;
    vector<Point3d>::iterator leftmostIt = points.begin();

    // compute the lefmost point
    for (vector<Point3d>::iterator it = points.begin() + 1;
            it != points.end(); ++it)
    {
        if (it->x < leftmostIt->x) {
            leftmostIt = it;
        }
    }

    Point3d pointOnHull = *leftmostIt;
    Point3d endPoint;

    do {
        // add the candidate point to the hull
        hull.push_back(pointOnHull);
        endPoint = points[0];

        for (unsigned int j = 1; j < points.size(); ++j) {
            // define dummy points in the XOZ plane
            Point3d fakeCurPt(points[j].x, 0.0, points[j].z);
            Point3d fakePointOnHull(pointOnHull.x, 0.0, pointOnHull.z);
            Point3d fakeEndPoint(endPoint.x, 0.0, endPoint.z);

            // define a point to the left of the line
            Point3d farLeft = fakeEndPoint;
            farLeft.rotate(pointOnHull, Rotation3d(0.0, -M_PI/6, 0.0));

            // if ... or points[j] on the left side of line (pointOnHull, endPoint)
            if (pointOnHull != points[j] &&
                    (endPoint == pointOnHull || sameSide(fakeCurPt, farLeft, fakePointOnHull, fakeEndPoint)))
            {
                endPoint = points[j];
            }
        }

        pointOnHull = endPoint;

    } while (endPoint != hull[0]);

    return hull;
}

bool model::fileExists(const string& fileName) {
    struct stat buffer ;
    if (stat(fileName.c_str(), &buffer)) return false;   // ret 0 for found
    return true;
}

bool model::fileIsDir(const string& fileName) {
    struct stat buffer;
    int status = stat(fileName.c_str(), &buffer);

    if (status != 0 || S_ISREG(buffer.st_mode)) {
        return false;
    }

    if (S_ISDIR(buffer.st_mode)) {
        return true;
    }

    return false;
}

bool model::makeDir(const string& path) {
	if (fileExists(path) && fileIsDir(path)) {
		return true;
	}

	return (mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == 0 ? true : false);
}
