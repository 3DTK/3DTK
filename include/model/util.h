/**
 * @file util.h
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 13 Feb 2012
 *
 */

#ifndef UTIL_H_
#define UTIL_H_

//==============================================================================
//  Includes
//==============================================================================
#include "model/vector3d.h"

#include <stdlib.h>
#include <vector>

namespace model {

/**
 * Checks if a point is one one side of an edge.
 */
bool sameSide(const model::Point3d& p1, const model::Point3d& p2, const model::Point3d& a, const model::Point3d& b);

/**
 * Checks if this point lies inside a certain convex hull.
 */
bool insideHull(const model::Point3d& pt, const std::vector<model::Point3d>& hull);

/**
 * Computes the convex hull of 3D points, disregarding their height.
 */
std::vector<model::Point3d> getHorizontalConvexHull(std::vector<model::Point3d> points);

/**
 * Checks if a file exists.
 * @param fileName the name of the file to check for existance
 */
bool fileExists(const std::string& fileName);

/**
 * @return true if given file is a directory
 */
bool fileIsDir(const std::string& fileName);

/**
 * Creates a directory at the given path.
 */
bool makeDir(const std::string& path);

/**
 * definition of x^2
 */
template <class T>
static inline T sqr(const T& x) {
    return x * x;
}

/**
 * generates a random RGB color
 */
static inline void randomColor(const int& maxVal,
        int& r, int& g, int& b)
{
    r = rand() % maxVal;
    g = rand() % maxVal;
    b = rand() % maxVal;
}

} /* namespace model */

#endif /* UTIL_H_ */
