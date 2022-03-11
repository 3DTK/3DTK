#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE bkdtree

#include <boost/test/unit_test.hpp>
#include <stdlib.h>
#include <time.h>
#include "slam6d/bkd.h"

using namespace std;

#define TEST BOOST_AUTO_TEST_CASE

#define setup_tree_with_pts(N) \
    BkdTree tree( 20 ); \
    srand(time(NULL)); \
    double *p2 = new double[3]; \
    p2[0] = 0.00; \
    p2[1] = 0.00; \
    p2[2] = 0.00; \
    tree.insert(p2); \
    for (int n = 0; n < N; ++n) \
    { \
        double *p = new double[3]; \
        for (int i = 0; i < 3; ++i) p[i] = drand(10.0, 500.0); \
        tree.insert(p); \
    } 

// Generates double numbers
double drand(double dmin, double dmax)
{
    double d = (double)rand() / RAND_MAX;
    return dmin + d * (dmax - dmin);
}

// Exactly the same point. 
bool valequals(double* p1, double *p2)
{
    return p1[0] == p2[0] && p1[1] == p2[1] && p1[2] == p2[2];
}

// Excactly same pointer adress.
bool ptrequals(double *p1, double *p2) {
    return (void*)p1 == (void*)p2;
}

TEST(simple_insertion_test)
{
    setup_tree_with_pts(10000);
    //cout << "Tree size: " << tree.size() << endl;
    int treesize = tree.size();
    BOOST_CHECK(treesize == 10001);
}

TEST(simple_closest_test)
{
    setup_tree_with_pts(10000);
    double *p1 = new double[3]; 
    p1[0] = 0.00; 
    p1[1] = 0.00; 
    p1[2] = 0.0001;
    double *closest = tree.FindClosest(p1, __DBL_MAX__);
    BOOST_CHECK(valequals(closest, p2) && ptrequals(closest, p2));
}

TEST(simple_remove_test)
{
    setup_tree_with_pts(10000);
    double *p1 = new double[3]; 
    p1[0] = 0.00; 
    p1[1] = 0.00; 
    p1[2] = 0.0001;
    double *closest = tree.FindClosest(p1, __DBL_MAX__);
    int removed1 = tree.remove(closest);
    int removed2 = tree.remove(closest);
    BOOST_CHECK(removed1 == 1 && removed2 == 0);
}

TEST(simple_consistency_check)
{
    BkdTree tree2(3);
    for (int i = 10; i <= 100; i+=10) {
        double *p = new double[3];
        p[0] = i;
        p[1] = i; // simple points
        p[2] = i;
        tree2.insert(p);
    }
    double p[3] = {20, 20, 20};
    tree2.remove(p);
    double *cls = tree2.FindClosest(p, __DBL_MAX__);
    BOOST_CHECK(!valequals(cls, p));
}

TEST(simple_knearest_check)
{
    // Sets up tree, includes (0, 0, 0)
    setup_tree_with_pts(100);
    
    // Include second reference point to compare
    double second[3] = {0, 0, 0.001};
    tree.insert( second );

    // Search two nearest to that point
    double point[3] = {0.00001, 0.0, 0.0};
    vector<Point> result = tree.kNearestNeighbors(point, 2);
    vector<Point> truth;
    truth.push_back( Point(0, 0, 0) );
    truth.push_back( Point(0, 0, 0.001) );
    BOOST_CHECK_EQUAL_COLLECTIONS(result.begin(), result.end(), truth.begin(), truth.end());
}

// FIXME : add more complex tests!