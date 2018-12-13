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
// C includes.
#include <stdlib.h>
#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#endif
#include <math.h>

// C++ includes.
#include <string>
#include <vector>

namespace floorplan {
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

static inline double normalizedRadAngle(double angle) {
    angle = fmod(angle, 2 * M_PI);

    while (angle < 0) {
        angle += 2 * M_PI;
    }

    // Redundant.
    while (angle > 2 * M_PI) {
        angle -= 2 * M_PI;
    }

    return angle;
}

static inline double normalizedDegAngle(double angle) {
    angle = fmod(angle, 360.0);

    while (angle < 0) {
        angle += 360.0;
    }

    // Redundant.
    while (angle > 360.0) {
        angle -= 360.0;
    }

    return angle;
}

static inline double rad2deg(double angle) {
    return angle * 180.0 / M_PI;
}

static inline double deg2rad(double angle) {
    return angle * M_PI / 180.0;
}

} // namespace model

#endif // UTIL_H_
