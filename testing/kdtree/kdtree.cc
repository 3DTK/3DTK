#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE kdtree
#include <boost/test/unit_test.hpp>
#include "slam6d/kd.h"

using namespace std;

#define TEST BOOST_AUTO_TEST_CASE

#define closest_setup(N) \
    size_t num_points = N; \
    double maxdist2 = 4; \
    double** pa = new double*[num_points];
#define closest_run \
    KDtree t(pa, num_points); \
    double point[3] = {0.0, 0.0, 0.0}; \
    double* closest = t.FindClosest(point, maxdist2, 0);

// test a point that is exactly maxdist2 away
TEST(find_closest1)
{
    closest_setup(1);
    pa[0] = new double[3]{2.00000, 0.0, 0.0};
    closest_run;
    BOOST_CHECK(closest == NULL);
}

// test a point that is just a little closer
TEST(find_closest2)
{
    closest_setup(1);
    pa[0] = new double[3]{1.99999999999, 0.0, 0.0};
    closest_run;
    BOOST_CHECK(closest == pa[0]);
}

// if more than one point is within the search radius, test the right one is
// returned
TEST(find_closest3)
{
    closest_setup(2);
    pa[0] = new double[3]{1.5, 0.0, 0.0};
    pa[1] = new double[3]{1.0, 0.0, 0.0};
    closest_run;
    BOOST_CHECK(closest == pa[1]);
}

#define closest_along_dir_setup(N) \
    size_t num_points = N; \
    double maxdist2 = 4; \
    double** pa = new double*[num_points];
#define closest_along_dir_run \
    KDtree t(pa, num_points); \
    double point[3] = {0.0, 0.0, 0.0}; \
    double dir[3] = {1.0, 0.0, 0.0}; \
    double* closest = t.FindClosestAlongDir(point, dir, maxdist2, 0); \

// here, maxdist is the distance around the direction vector and not the
// maximum distance from the origin
// test a point that is exactly maxdist2 away from the ray
TEST(find_closest_along_dir1)
{
    closest_along_dir_setup(1);
    pa[0] = new double[3]{1.0, 2.0, 0.0};
    closest_along_dir_run;
    BOOST_CHECK(closest == NULL);
}

// test a point that is just a little closer
TEST(find_closest_along_dir2)
{
    closest_along_dir_setup(1);
    pa[0] = new double[3]{1.0, 1.99999999999, 0.0};
    closest_along_dir_run;
    BOOST_CHECK(closest == pa[0]);
}

// closest point here does not mean closest to the origin but closest to the
// ray that is cast
TEST(find_closest_along_dir3)
{
    closest_along_dir_setup(2);
    pa[0] = new double[3]{0.5, 0.1, 0.0}; // this point is closer to the origin
    pa[1] = new double[3]{1.0, 0.0, 0.0}; // but this one is chosen because
                                          // it's closer to the ray
    closest_along_dir_run;
    BOOST_CHECK(closest == pa[1]);
}

// points are not only found in the direction of the direction vector but also
// in the opposite direction
// test a point that is in the opposite direction
TEST(find_closest_along_dir4)
{
    closest_along_dir_setup(1);
    pa[0] = new double[3]{-1.0, 0.0, 0.0};
    closest_along_dir_run;
    BOOST_CHECK(closest == pa[0]);
}

// find 2 nearest neighbors
TEST(k_nearest_neighbors1)
{
    size_t num_points = 3;
    double** pa = new double*[num_points];
    pa[0] = new double[3]{1.0, 0.0, 0.0};
    pa[1] = new double[3]{2.0, 0.0, 0.0};
    pa[2] = new double[3]{3.0, 0.0, 0.0};
    KDtree t(pa, num_points);
    double point[3] = {0.0, 0.0, 0.0};
    vector<Point> result = t.kNearestNeighbors(point, 2, 0);
    vector<Point> trueresult = { Point(1.0, 0.0, 0.0), Point(2.0, 0.0, 0.0) };
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), trueresult.begin(), trueresult.end());
}

#define fixed_range_search_between_2_points_setup(N) \
    size_t num_points = N; \
    double maxdist2 = 4; \
    double** pa = new double*[num_points];
#define fixed_range_search_between_2_points_run \
    KDtree t(pa, num_points); \
    double point1[3] = {0.0, 0.0, 0.0}; \
    double point2[3] = {1.0, 0.0, 0.0}; \
    vector<Point> result = t.fixedRangeSearchBetween2Points(point1, point2, maxdist2, 0);

// test a point that is exactly one of the two
TEST(fixed_range_search_between_2_points1)
{
    fixed_range_search_between_2_points_setup(1);
    pa[0] = new double[3]{0.0, 0.0, 0.0};
    fixed_range_search_between_2_points_run;
    vector<Point> trueresult = { Point(0.0, 0.0, 0.0) };
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), trueresult.begin(), trueresult.end());
}

// test a point that is exactly one of the two (the other)
TEST(fixed_range_search_between_2_points2)
{
    fixed_range_search_between_2_points_setup(1);
    pa[0] = new double[3]{1.0, 0.0, 0.0};
    fixed_range_search_between_2_points_run;
    vector<Point> trueresult = { Point(1.0, 0.0, 0.0) };
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), trueresult.begin(), trueresult.end());
}

// test a point that is in the same direction but not on the segment
TEST(fixed_range_search_between_2_points3)
{
    fixed_range_search_between_2_points_setup(1);
    pa[0] = new double[3]{10.0, 0.0, 0.0};
    fixed_range_search_between_2_points_run;
    vector<Point> trueresult = { Point(10.0, 0.0, 0.0) };
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), trueresult.begin(), trueresult.end());
}

// test a point that is in the same direction but not on the segment (the
// other direction)
TEST(fixed_range_search_between_2_points4)
{
    fixed_range_search_between_2_points_setup(1);
    pa[0] = new double[3]{-10.0, 0.0, 0.0};
    fixed_range_search_between_2_points_run;
    vector<Point> trueresult = { Point(-10.0, 0.0, 0.0) };
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), trueresult.begin(), trueresult.end());
}

#define fixed_range_search_along_dir_setup(N) \
    size_t num_points = N; \
    double maxdist2 = 4; \
    double** pa = new double*[num_points];
#define fixed_range_search_along_dir_run \
    KDtree t(pa, num_points); \
    double point1[3] = {0.0, 0.0, 0.0}; \
    double point2[3] = {1.0, 0.0, 0.0}; \
    vector<Point> result = t.fixedRangeSearchAlongDir(point1, point2, maxdist2, 0);

// test a point that is exactly one of the two
TEST(fixed_range_search_along_dir1)
{
    fixed_range_search_along_dir_setup(1);
    pa[0] = new double[3]{0.0, 0.0, 0.0};
    fixed_range_search_along_dir_run;
    vector<Point> trueresult = { Point(0.0, 0.0, 0.0) };
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), trueresult.begin(), trueresult.end());
}

// test a point that is exactly one of the two (the other)
TEST(fixed_range_search_along_dir2)
{
    fixed_range_search_along_dir_setup(1);
    pa[0] = new double[3]{1.0, 0.0, 0.0};
    fixed_range_search_along_dir_run;
    vector<Point> trueresult = { Point(1.0, 0.0, 0.0) };
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), trueresult.begin(), trueresult.end());
}

// test a point that is in the same direction but not on the segment
TEST(fixed_range_search_along_dir3)
{
    fixed_range_search_along_dir_setup(1);
    pa[0] = new double[3]{10.0, 0.0, 0.0};
    fixed_range_search_along_dir_run;
    vector<Point> trueresult = { Point(10.0, 0.0, 0.0) };
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), trueresult.begin(), trueresult.end());
}

// test a point that is in the same direction but not on the segment (the
// other direction)
TEST(fixed_range_search_along_dir4)
{
    fixed_range_search_along_dir_setup(1);
    pa[0] = new double[3]{-10.0, 0.0, 0.0};
    fixed_range_search_along_dir_run;
    vector<Point> trueresult = { Point(-10.0, 0.0, 0.0) };
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), trueresult.begin(), trueresult.end());
}

TEST(abb_search1)
{
    size_t num_points = 3;
    double** pa = new double*[num_points];
    pa[0] = new double[3]{-1.0, 0.0, 0.0};
    pa[1] = new double[3]{0.0, 0.0, 0.0};
    pa[2] = new double[3]{1.0, 0.0, 0.0};
    KDtree t(pa, num_points);
    double point1[3] = {-1.5, -0.5, -0.5};
    double point2[3] = {1.5, 0.5, 0.5};
    vector<Point> result = t.AABBSearch(point1, point2, 0);
    vector<Point> trueresult = { Point(-1.0, 0.0, 0.0), Point(0.0, 0.0, 0.0), Point(1.0, 0.0, 0.0) };
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), trueresult.begin(), trueresult.end());
}
