#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE kdtree_indexed_random
#include <boost/test/unit_test.hpp>
#include <boost/random/variate_generator.hpp>
#include <boost/random/mersenne_twister.hpp>
#include <boost/random/uniform_real.hpp>
#include <cstdlib>
#include "slam6d/kdIndexed.h"
#include <limits>

using namespace std;

#define TEST BOOST_AUTO_TEST_CASE

size_t myFindClosest(double point[3], double** pa, double maxdist2, size_t num_points)
{
    size_t closest = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < num_points; ++i) {
        double newdist2 = Dist2(point, pa[i]);
        if (newdist2 < maxdist2) {
            maxdist2 = newdist2;
            closest = i;
        }
    }
    return closest;
}

vector<size_t> myFixedRangeSearch(double point[3], double** pa, double maxdist2, size_t num_points)
{
    vector<size_t> result;
    for (size_t i = 0; i < num_points; ++i) {
        double newdist2 = Dist2(point, pa[i]);
        if (newdist2 < maxdist2) {
            result.push_back(i);
        }
    }
    return result;
}

vector<size_t> myFixedRangeSearchBetween2Points(double point1[3], double point2[3], double** pa, double maxdist2, size_t num_points)
{
    vector<size_t> result;
    double dir[3];
    dir[0] = point2[0] - point1[0];
    dir[1] = point2[1] - point1[1];
    dir[2] = point2[2] - point1[2];
    Normalize3(dir);
    for (size_t i = 0; i < num_points; ++i) {
        double p2p[] = {point1[0] - pa[i][0],
            point1[1] - pa[i][1],
            point1[2] - pa[i][2]};
        double newdist2 = Len2(p2p) - sqr(Dot(p2p,dir));
        if (newdist2 < maxdist2) {
            result.push_back(i);
        }
    }
    return result;
}

size_t myFindClosestAlongDir(double point1[3], double point2[3], double** pa, double maxdist2, size_t num_points)
{
    size_t closest = std::numeric_limits<size_t>::max();
    for (size_t i = 0; i < num_points; ++i) {
        double p2p[] = {point1[0] - pa[i][0],
            point1[1] - pa[i][1],
            point1[2] - pa[i][2]};
        double newdist2 = Len2(p2p) - sqr(Dot(p2p,point2));
        if (newdist2 < maxdist2) {
            maxdist2 = newdist2;
            closest = i;
        }
    }
    return closest;
}

vector<size_t> myAABBSearch(double point1[3], double point2[3], double** pa, size_t num_points)
{
    vector<size_t> result;
    for (size_t i = 0; i < num_points; ++i) {
        if (pa[i][0] >= point1[0] && pa[i][0] <= point2[0]
                && pa[i][1] >= point1[1] && pa[i][1] <= point2[1]
                && pa[i][2] >= point1[2] && pa[i][2] <= point2[2]) {
            result.push_back(i);
        }
    }
    return result;
}

struct comp {
    double **pa;
    double *point;
    comp(double *point,double** pa) { this->point = point; this->pa = pa; }
    bool operator() (int a, int b) { return (Dist2(pa[a],point) < Dist2(pa[b], point)); }
};

vector<size_t> myKNearestNeighbors(double point[3], int k, double** pa, size_t num_points)
{
    vector<size_t> result;
    for (size_t i = 0; i < num_points; ++i) {
        result.push_back(i);
    }
    sort(result.begin(), result.end(), comp(point, pa));
    result.resize(k);
    return result;
}

size_t mySegmentSearch_1NearestPoint(double point1[3], double point2[3], double** pa, double maxdist2, size_t num_points)
{
    size_t closest = std::numeric_limits<size_t>::max();
    double dir[] = {
        point2[0] - point1[0],
        point2[1] - point1[1],
        point2[2] - point1[2]
    };
    double len2 = Len2(dir);
    double p2p[3];
    double proj[3];
    double t, newdist2;
    double mindist2 = numeric_limits<double>::infinity();
    for (size_t i = 0; i < num_points; ++i) {
        // find the length of the projection of the point onto the line segment
        p2p[0] = pa[i][0] - point1[0];
        p2p[1] = pa[i][1] - point1[1];
        p2p[2] = pa[i][2] - point1[2];
        t = Dot(p2p, dir)/len2;
        if (t < 0.0) {
            // point is beyond point1 of the segment
            if (Dist2(point1,pa[i]) >= maxdist2)
                continue;
        } else if (t > 1.0) {
            // point is beyond point2 of the segment
            if (Dist2(point2,pa[i]) >= maxdist2)
                continue;
        } else {
            // point is within the segment
            // calculate projection
            proj[0] = point1[0] + t*dir[0];
            proj[1] = point1[1] + t*dir[1];
            proj[2] = point1[2] + t*dir[2];
            if (Dist2(proj,pa[i]) >= maxdist2)
                continue;
        }
        newdist2 = Dist2(point1,pa[i]);
        if (newdist2 < mindist2) {
            mindist2 = newdist2;
            closest = i;
        }
    }
    return closest;
}

vector<size_t> mySegmentSearch_all(double point1[3], double point2[3], double** pa, double maxdist2, size_t num_points)
{
    vector<size_t> result;
    double dir[] = {
        point2[0] - point1[0],
        point2[1] - point1[1],
        point2[2] - point1[2]
    };
    double len2 = Len2(dir);
    double p2p[3];
    double proj[3];
    double t, *comp;
    for (size_t i = 0; i < num_points; ++i) {
        // find the length of the projection of the point onto the line segment
        p2p[0] = pa[i][0] - point1[0];
        p2p[1] = pa[i][1] - point1[1];
        p2p[2] = pa[i][2] - point1[2];
        t = Dot(p2p, dir)/len2;
        if (t < 0.0) {
            // point is beyond point1 of the segment
            comp = point1;
        } else if (t > 1.0) {
            // point is beyond point2 of the segment
            comp = point2;
        } else {
            // point is within the segment
            // calculate projection
            proj[0] = point1[0] + t*dir[0];
            proj[1] = point1[1] + t*dir[1];
            proj[2] = point1[2] + t*dir[2];
            comp = proj;
        }
        if (Dist2(comp,pa[i]) < maxdist2) {
            result.push_back(i);
        }
    }
    return result;
}

// make sure we always use the same seed to always generate the same
// random grid
// we generate a grid inside the axis aligned bounding box from
// (-10,-10,-10) to (10,10,10) with 10000 points inside
#define setup                                                                \
    boost::mt19937 generator(42u);                                           \
    boost::uniform_real<> uni_dist(-10,10);                                  \
    boost::variate_generator<boost::mt19937&, boost::uniform_real<> > uni(generator, uni_dist); \
    size_t num_points = 10000;                                               \
    double** pa = new double*[num_points];                                   \
    for (size_t i = 0; i < num_points; ++i) {                                \
        pa[i] = new double[3]{uni(), uni(), uni()};                          \
    }                                                                        \
    KDtreeIndexed t(pa, num_points);                                         \
    double point1[3];                                                        \

TEST(find_closest)
{
    setup;
    // check FindClosest
    for (double maxdist2 = 0.5; maxdist2 <= 5.0; maxdist2 += 0.5) {
        for (int i = 0; i < 100; ++i) {
            point1[0] = uni();
            point1[1] = uni();
            point1[2] = uni();
            size_t ret1 = myFindClosest(point1, pa, maxdist2, num_points);
            size_t ret2 = t.FindClosest(point1, maxdist2, 0);
            //cout << ret1 << " " << ret2 << endl;
            BOOST_CHECK(ret1 == ret2);
        }
    }
}

TEST(find_closest_along_dir)
{
    setup;
    double point2[3];
    // check FindClosestAlongDir
    for (double maxdist2 = 0.5; maxdist2 <= 5.0; maxdist2 += 0.5) {
        for (int i = 0; i < 100; ++i) {
            point1[0] = uni();
            point1[1] = uni();
            point1[2] = uni();
            point2[0] = uni();
            point2[1] = uni();
            point2[2] = uni();
            // we use point2 as direction vector
            size_t ret1 = myFindClosestAlongDir(point1, point2, pa, maxdist2, num_points);
            size_t ret2 = t.FindClosestAlongDir(point1, point2, maxdist2, 0);
            //cout << ret1 << " " << ret2 << endl;
            BOOST_CHECK(ret1 == ret2);
        }
    }
}

TEST(k_nearest_neighbors)
{
    setup;
    // check kNearestNeighbors
    for (int k = 1; k <= 10; ++k) {
        for (int i = 0; i < 100; ++i) {
            point1[0] = uni();
            point1[1] = uni();
            point1[2] = uni();
            /*for (auto it2 = res.begin(); it2 != res.end(); ++it2) {
                cout << *it2 << ",";
            }
            cout << endl;*/
            vector<size_t> res1 = t.kNearestNeighbors(point1, k, 0);
            vector<size_t> res2 = myKNearestNeighbors(point1, k, pa, num_points);
            sort(res1.begin(), res1.end());
            sort(res2.begin(), res2.end());
            BOOST_CHECK_EQUAL_COLLECTIONS(res1.begin(), res1.end(), res2.begin(), res2.end());
        }
    }
}

TEST(fixed_range_search_between_2_points)
{
    setup;
    double point2[3];
    // check fixedRangeSearchBetween2Points
    for (double maxdist2 = 0.5; maxdist2 <= 5.0; maxdist2 += 0.5) {
        for (int i = 0; i < 100; ++i) {
            point1[0] = uni();
            point1[1] = uni();
            point1[2] = uni();
            point2[0] = uni();
            point2[1] = uni();
            point2[2] = uni();
            /*for (auto it2 = res.begin(); it2 != res.end(); ++it2) {
                cout << *it2 << ",";
            }
            cout << endl;*/
            vector<size_t> res1 = t.fixedRangeSearchBetween2Points(point1, point2, maxdist2, 0);
            vector<size_t> res2 = myFixedRangeSearchBetween2Points(point1, point2, pa, maxdist2, num_points);
            sort(res1.begin(), res1.end());
            sort(res2.begin(), res2.end());
            BOOST_CHECK_EQUAL_COLLECTIONS(res1.begin(), res1.end(), res2.begin(), res2.end());
        }
    }
}

TEST(aabbsearch)
{
    setup;
    double point2[3];
    // check AABBSearch
    for (int i = 0; i < 1000; ++i) {
        point1[0] = uni();
        point1[1] = uni();
        point1[2] = uni();
        point2[0] = uni();
        point2[1] = uni();
        point2[2] = uni();
        // swap coordinates if necessary
        if (point2[0] < point1[0]) {
            double tmp;
            tmp = point1[0];
            point1[0] = point2[0];
            point2[0] = tmp;
        }
        if (point2[1] < point1[1]) {
            double tmp;
            tmp = point1[1];
            point1[1] = point2[1];
            point2[1] = tmp;
        }
        if (point2[2] < point1[2]) {
            double tmp;
            tmp = point1[2];
            point1[2] = point2[2];
            point2[2] = tmp;
        }
        vector<size_t> res1 = t.AABBSearch(point1, point2, 0);
        vector<size_t> res2 = myAABBSearch(point1, point2, pa, num_points);
        /*for (auto it2 = res.begin(); it2 != res.end(); ++it2) {
            cout << *it2 << ",";
        }
        cout << endl;*/
        sort(res1.begin(), res1.end());
        sort(res2.begin(), res2.end());
        BOOST_CHECK_EQUAL_COLLECTIONS(res1.begin(), res1.end(), res2.begin(), res2.end());
    }
}

TEST(fixed_range_search)
{
    setup;
    // check fixedRangeSearch
    for (double maxdist2 = 0.5; maxdist2 <= 5.0; maxdist2 += 0.5) {
        for (int i = 0; i < 100; ++i) {
            point1[0] = uni();
            point1[1] = uni();
            point1[2] = uni();
            /*for (auto it2 = res.begin(); it2 != res.end(); ++it2) {
                cout << *it2 << ",";
            }
            cout << endl;*/
            vector<size_t> res1 = myFixedRangeSearch(point1, pa, maxdist2, num_points);
            vector<size_t> res2 = t.fixedRangeSearch(point1, maxdist2, 0);
            sort(res1.begin(), res1.end());
            sort(res2.begin(), res2.end());
            BOOST_CHECK_EQUAL_COLLECTIONS(res1.begin(), res1.end(), res2.begin(), res2.end());
        }
    }
}

TEST(segmentsearch_all)
{
    setup;
    double point2[3];
    for (double maxdist2 = 0.5; maxdist2 <= 5.0; maxdist2 += 0.5) {
        for (int i = 0; i < 100; ++i) {
            point1[0] = uni();
            point1[1] = uni();
            point1[2] = uni();
            point2[0] = uni();
            point2[1] = uni();
            point2[2] = uni();
            vector<size_t> res1 = mySegmentSearch_all(point1, point2, pa, maxdist2, num_points);
            vector<size_t> res2 = t.segmentSearch_all(point1, point2, maxdist2, 0);
            sort(res1.begin(), res1.end());
            sort(res2.begin(), res2.end());
            BOOST_CHECK_EQUAL_COLLECTIONS(res1.begin(), res1.end(), res2.begin(), res2.end());
        }
    }
}

TEST(segmentsearch_1nearest)
{
    setup;
    double point2[3];
    for (double maxdist2 = 0.5; maxdist2 <= 5.0; maxdist2 += 0.5) {
        for (int i = 0; i < 100; ++i) {
            point1[0] = uni();
            point1[1] = uni();
            point1[2] = uni();
            point2[0] = uni();
            point2[1] = uni();
            point2[2] = uni();
            size_t res1 = mySegmentSearch_1NearestPoint(point1, point2, pa, maxdist2, num_points);
            size_t res2 = t.segmentSearch_1NearestPoint(point1, point2, maxdist2, 0);
            //cout << res1 << " " << res2 << endl;
            BOOST_CHECK(res1 == res2);
        }
    }
}
