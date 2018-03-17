/** Unit test for rotation3d **/
#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE model
#include <boost/test/unit_test.hpp>
#include <model/rotation3d.h>

using namespace std;
using namespace model;


BOOST_AUTO_TEST_CASE(test_rotation3d) {

    //Test0 inline constructor with parameters
    Rotation3d rotation(0.1, 3.0, 1.5);
    BOOST_CHECK(rotation.x == 0.1 && rotation.y == 3.0 && rotation.z == 1.5);

    //Test1 inline constructor with no parameters
    Rotation3d rotation1;
    BOOST_CHECK(rotation1.x == 0.0 && rotation1.y == 0.0 && rotation1.z == 0.0);
    
    //Test2 copy constructor
    Rotation3d rotation2(rotation);
    BOOST_CHECK(rotation2.x == rotation.x && rotation2.y == rotation.y && rotation2.z == rotation.z);
    
    //Test3 = operator
    Rotation3d rotation3;
    rotation3 = rotation;
    BOOST_CHECK(rotation3.x == rotation.x && rotation3.y == rotation.y && rotation3.z == rotation.z);
    
    //Test4 getInverse()
    Rotation3d result = rotation3.getInverse();
    BOOST_CHECK(result.x == -0.1 && result.y == -3.0 && result.z == -1.5);
    
}
