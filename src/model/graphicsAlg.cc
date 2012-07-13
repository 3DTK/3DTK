/**
 * @file graphicsAlg.cc
 *
 * @auhtor Remus Claudiu Dumitru <r.dumitru@jacobs-university.de>
 * @date 18 Feb 2012
 *
 */

//==============================================================================
//  Includes
//==============================================================================
#include "model/graphicsAlg.h"

#include "model/scene.h"

#include <math.h>
#include <algorithm>

using namespace std;

//==============================================================================
//  Global Variables
//==============================================================================
extern bool quiet;

template <class InputIterator, class OutputIterator>
void model::GraphicsAlg::alpha_edges(InputIterator begin, InputIterator end,
        const Type& Alpha,
        bool mode,
        OutputIterator out)
{
    Alpha_shape_2 shape(begin, end);

    if (mode) {
        shape.set_mode(Alpha_shape_2::GENERAL);
    }
    else {
        shape.set_mode(Alpha_shape_2::REGULARIZED);
    }

    shape.set_alpha(Alpha);

    for(Alpha_shape_edges_iterator it =  shape.alpha_shape_edges_begin();
            it != shape.alpha_shape_edges_end(); ++it)
    {
        *out++ = shape.segment(*it);
    }
}

// to be used to sort the pairs in the clockwiseSort method
bool compare(pair<double, unsigned int> left, pair<double, unsigned int> right) {
    // we want to order clockwise, so we should compare the other way around
    return left.first > right.first;
}

void model::GraphicsAlg::clockwiseSort(vector<Plane3d>& planes) {
    if (planes.size() <= 2) {
        return;
    }

    // to be returned at the end
    vector<Plane3d> result;

    // first compute the center of gravity of the given planes
    Point3d cog;
    for (unsigned int i = 0; i < planes.size(); ++i) {
        cog += planes[i].pt;
    }
    cog /= planes.size();

    // keep the angle of each plane wrt to the center of gravity
    // also keep and index with the position of the plane in the original vector
    vector<pair<double, unsigned int> > angles;

    // compute each angle and add it to the angles vector alongside the index
    for (unsigned int i = 0; i < planes.size(); ++i) {
        double angle = atan2(planes[i].pt.z - cog.z, planes[i].pt.x - cog.x);
        angles.push_back(make_pair(angle, i));
    }

    // use stl sort
    sort(angles.begin(), angles.end(), compare);

    // construct the result
    for (unsigned int i = 0; i < angles.size(); ++i) {
        result.push_back(planes[angles[i].second]);
    }

    // copy result into planes
    planes = result;
}

vector<model::Plane3d> model::GraphicsAlg::getConcaveHull(std::vector<model::Plane3d> planes) {
    if (planes.size() <= 3) {
        return planes;
    }

    const double eps = 0.1;
    vector<Plane3d> result;
    vector<Plane3d> verticalPlanes;

    // CGAL datatypes
    list<Point> points;
    vector<Segment> segments;

    // keep vertical planes
    for (vector<Plane3d>::iterator it = planes.begin();
            it < planes.end(); ++it)
    {
        if (it->isVertical()) {
            verticalPlanes.push_back(*it);
        }
    }

    // free a bit of memory
    planes.clear();

    // collect all the remaining planes as CGAL points
    for (vector<Plane3d>::iterator it = verticalPlanes.begin();
            it < verticalPlanes.end(); ++it)
    {
        Point toAdd(it->pt.x, it->pt.z);
        points.push_back(toAdd);
    }

    // XXX compute the concave hull, aka. the alpha edges
    alpha_edges(points.begin(), points.end(),
            Type(250000), Alpha_shape_2::GENERAL,
            std::back_inserter(segments));

    if (!quiet) cout << "** " << segments.size() << " concave hull segments found" << endl;

    // counts how many planes are rematched to make sure we find them again
    unsigned int counter = 0;

    // XXX naive retrieval of planes
    Point3d center;
    vector<Point> possiblePoints;
    for (vector<Segment>::iterator it = segments.begin();
            it < segments.end(); ++it)
    {
        Point source = it->source();
        Point target = it->target();

        center.x += source.x();
        center.z += source.y();
        center.x += target.x();
        center.z += target.y();

        possiblePoints.push_back(source);
        possiblePoints.push_back(target);
    }
    center /= possiblePoints.size();

    // match the points found using alpha shapes, back to their planes
    for (vector<Point>::iterator it = possiblePoints.begin(); it < possiblePoints.end(); ++it) {
        for (vector<Plane3d>::iterator jt = verticalPlanes.begin(); jt < verticalPlanes.end(); ++jt) {
            // second coordinate is on the z axis in the left handed coord system
            if ((it->x() >= jt->pt.x - eps) && (it->x() <= jt->pt.x + eps) &&
                    (it->y() >= jt->pt.z - eps) && (it->y() <= jt->pt.z + eps))
            {
                result.push_back(*jt);
                verticalPlanes.erase(jt);
                counter++;
                break;
            }
        }
    }

    // eliminate duplicate planes
    result = Scene::getSignificantPlanes(result);

    // sort them clockwise, it is in place
    clockwiseSort(result);

    // another threshold for deciding whether to keep a plane or not
    double eps2 = 0.2;

    // discard internal walls that are not part of the external walls of the room
    for (vector<Plane3d>::iterator it = result.begin() + 1;
            it < result.end(); ++it)
    {
        vector<Plane3d>::iterator prev = it - 1;
        vector<Plane3d>::iterator curr = it;

        // normalize normals just to be sure
        prev->normal.normalize();
        curr->normal.normalize();

        if ((1.0 - fabs(prev->normal.dotProduct(curr->normal))) < eps2) {
            if (fabs(center.distance(*prev)) < fabs(center.distance(*curr))) {
                if (!quiet) cout << "** Discarding previous internal plane centered at "
                        << prev->pt.x << " " << prev->pt.y << " " << prev->pt.z
                        << endl;
                result.erase(prev);
            } else {
                if (!quiet) cout << "** Discarding current internal plane centered at "
                        << curr->pt.x << " " << curr->pt.y << " " << curr->pt.z
                        << endl;
                result.erase(curr);
            }
        }
    }

    if (!quiet) cout << "** Added " << result.size() << " elements to concave hull result" << endl;

    return result;
}

void model::GraphicsAlg::getDiscreteLine(Point3d src, Point3d dest, double precision, const double& extraDist,
        vector<Point3d>& line)
{
    // clear the result just in case it contains something
    line.clear();

    // add the extra distance
    double len = src.distance(dest);
    double temp = (len + extraDist) / len;
    dest.x = src.x + (dest.x - src.x) * temp;
    dest.y = src.y + (dest.y - src.y) * temp;
    dest.z = src.z + (dest.z - src.z) * temp;

    // round up the values
    precision = round(precision);

    src.x = round(src.x);
    src.y = round(src.y);
    src.z = round(src.z);

    dest.x = round(dest.x);
    dest.y = round(dest.y);
    dest.z = round(dest.z);

    // adjust according to precision
    const double coef = pow(10.0, precision);
    src  *= coef;
    dest *= coef;

//    Point3d diff = dest - src;
//    diff.x = fabs(diff.x);
//    diff.y = fabs(diff.y);
//    diff.z = fabs(diff.z);
//    diff += Point3d(1.0, 1.0, 1.0);
//    double temp[] = {diff.x, diff.y, diff.z};
//
//    double d = *max_element(temp, temp + 3);

    double x1 = src.x;
    double y1 = src.y;
    double z1 = src.z;

    double x2 = dest.x;
    double y2 = dest.y;
    double z2 = dest.z;

    double dx = x2 - x1;
    double dy = y2 - y1;
    double dz = z2 - z1;

    double ax = 2 * fabs(dx);
    double ay = 2 * fabs(dy);
    double az = 2 * fabs(dz);

    double sx = dx > 0 ? +1.0 : -1.0;
    double sy = dy > 0 ? +1.0 : -1.0;
    double sz = dz > 0 ? +1.0 : -1.0;

    double x = x1;
    double y = y1;
    double z = z1;

    if (ax >= max(ay, az)) {
        // x dominant
        double yd = ay - ax / 2.0;
        double zd = az - ax / 2.0;

        while (1) {
            line.push_back(Point3d(x, y, z));

            if (x == x2) {
                break;
            }

            // move along y
            if (yd >= 0) {
                y  += sy;
                yd -= ax;
            }

            // move along z
            if (zd >= 0) {
                z  += sz;
                zd -= ax;
            }

            // move along x
            x  += sx;
            yd += ay;
            zd += az;
        }
    } else if (ay >= max(ax, az)) {
        // y dominant
        double xd = ax - ay / 2.0;
        double zd = az - ay / 2.0;

        while (1) {
            line.push_back(Point3d(x, y, z));

            if (y == y2) {
                break;
            }

            // move along x
            if (xd >= 0) {
                x  += sx;
                xd -= ay;
            }

            // move along z
            if (zd >= 0) {
                z  += sz;
                zd -= ay;
            }

            // move along y
            y  += sy;
            xd += ax;
            zd += az;
        }
    } else if (az >= max(ax, ay)) {
        // z dominant
        double xd = ax - az / 2.0;
        double yd = ay - az / 2.0;

        while (1) {
            line.push_back(Point3d(x, y, z));

            if (z == z2) {
                break;
            }

            // move along x
            if (xd >= 0) {
                x  += sx;
                xd -= az;
            }

            // move along y
            if (yd >= 0) {
                y  += sy;
                yd -= az;
            }

            // move along x
            z  += sz;
            xd += ax;
            yd += ay;
        }
    } else {
        throw logic_error("invalid branch taken while computing discrete line");
    }

    for (vector<Point3d>::iterator it = line.begin(); it != line.end(); ++it) {
        it->x /= coef;
        it->y /= coef;
        it->z /= coef;
    }
}
