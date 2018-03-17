/** Unit test for rotation3d **/
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE model
#include <boost/test/unit_test.hpp>
#include <model/plane3d.h>
#include <model/point3d.h>
#include <cmath>

using namespace std;
using namespace model;


BOOST_AUTO_TEST_CASE(test_point3d) {

    double EPSILON = 0.02;

    // Test1 default constructor
    Point3d p; // 0, 0, 0
    BOOST_CHECK(p.x == 0.0 && p.y == 0.0 && p.z == 0.0);

    // Test2 constructor with parameters
    Point3d p1(1.0, 8.3, 10.4);
    BOOST_CHECK(p1.x == 1.0 && p1.y == 8.3 && p1.z == 10.4);
    
    // Test3 copy constructor
    Point3d p2(p1);
    BOOST_CHECK(p2.x == 1.0 && p2.y == 8.3 && p2.z == 10.4);
    
    // Test4 rotate()
    Point3d tempP(1.4, 1.5, 1.6);
    Rotation3d tempR(0.,0.,0.);
    p.rotate(tempP, tempR);
    BOOST_CHECK(p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
    Point3d tempP1(0.,0.,0.);
    Rotation3d tempR1(1.4, 3.3, 0.0);
    p.rotate(tempP1, tempR1);
    BOOST_CHECK(p.x == 0.0 && p.y == 0.0 && p.z == 0.0);
    
    // Test5 translate()
    Vector3d tempV2(2.0, 3.0, 4.0);
    p1.translate(tempV2);
    BOOST_CHECK(p1.x == 3.0 && p1.y == 11.3 && p1.z == 14.4);
    
    // Test6 distance() between 2 points
    Point3d tempP3(2.0, 0.0, 0.0);
    BOOST_CHECK(p.distance(tempP3) == 2.0);
    
    // Test7 distance() between point and plane
    Vector3d normal(7.0, 9.0, 8.0);
    Point3d origin(3.0, 4.0, 5.0);
    Plane3d plane(origin, normal);
    double d = p.distance(plane);
    //result should be 6.96419..
    BOOST_CHECK(fabs(d - 6.96) < EPSILON);
    
    // Test8 operator=
    p2 = p;
    BOOST_CHECK(p2.x == 0.0 && p2.y == 0.0 && p2.z == 0.0);

    Point3d result;
    // Test9 operator-
    result = p2 - p1;
    BOOST_CHECK(result.x == -3.0 && result.y == -11.3 && result.z == -14.4);

    // Test10 operator*
    result = p2*2;
    BOOST_CHECK(result.x == 0.0 && result.y == 0.0 && result.z == 0.0);

    // Test11 operator/
    result = p1/1;
    BOOST_CHECK(result.x == 3.0 && result.y == 11.3 && result.z == 14.4);

    // Test12 operator/=
    p1 /= 2;
    BOOST_CHECK(p1.x == 1.5 && p1.y == 5.65 && p1.z == 7.2);

    // Test13 operator+=
    p += p1;
    BOOST_CHECK(p.x == 1.5 && p.y == 5.65 && p.z == 7.2);

    // Test14 operator -=
    p1 -= p2;
    BOOST_CHECK(p1.x == 1.5 && p1.y == 5.65 && p1.z == 7.2);

    // Test15 operator*=
    p *= 2;
    BOOST_CHECK(p.x == 3.0 && p.y == 11.3 && p.z == 14.4);

    Point3d temp(3.0, 11.3, 14.4);
    // Test16 operator==
    BOOST_CHECK(temp == p);  
    BOOST_CHECK(!(p == p2));

    // Test17 operator!=
    BOOST_CHECK(!(temp != p));
    BOOST_CHECK(p != p2);

    // Tes18 operator<<
    std::stringstream buff;
    buff.precision(1);
    buff << fixed << p;
    BOOST_CHECK(buff.str() == "3.0 11.3 14.4");
}

