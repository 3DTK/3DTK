#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE scanio
#include <boost/test/unit_test.hpp>
#include <boost/test/floating_point_comparison.hpp>

#include <slam6d/io_types.h>
#include <slam6d/globals.icc>
#include <slam6d/scan.h>

#define TOLERANCE 0.0000000000001

BOOST_AUTO_TEST_CASE(readscans) {
    auto argv = boost::unit_test::framework::master_test_suite().argv;
    std::string path = std::string(argv[1]) + "/dat";

    Scan::openDirectory(false, path.c_str(), UOS, 0, -1);
    // make sure that there are exactly three scans
    BOOST_CHECK(Scan::allScans.size() == 3);
    // make sure all scans have the correct number of points
    for(std::vector<Scan*>::iterator it = Scan::allScans.begin();it != Scan::allScans.end(); ++it) {
        Scan* scan = *it;
        DataXYZ xyz(scan->get("xyz"));
        BOOST_CHECK(xyz.size() == 81360);
    }
    // check whether the values of the first point of each scan are the ones
    // they should be
    Scan* scan1 = Scan::allScans[0];
    DataXYZ xyz1(scan1->get("xyz"));
    BOOST_CHECK(xyz1[0][0] == 10.1);
    BOOST_CHECK(xyz1[0][1] == 0);
    BOOST_CHECK(xyz1[0][2] == 0);
    Scan* scan2 = Scan::allScans[1];
    DataXYZ xyz2(scan2->get("xyz"));
    BOOST_CHECK(xyz2[0][0] == 10.2);
    BOOST_CHECK(xyz2[0][1] == 0);
    BOOST_CHECK(xyz2[0][2] == 2.84217e-14);
    Scan* scan3 = Scan::allScans[2];
    DataXYZ xyz3(scan3->get("xyz"));
    BOOST_CHECK(xyz3[0][0] == 10.1);
    BOOST_CHECK(xyz3[0][1] == 3.55271e-15);
    BOOST_CHECK(xyz3[0][2] == 1.13687e-13);

    Scan::closeDirectory();
}

BOOST_AUTO_TEST_CASE(readscans_uos) {
    Scan::openDirectory(false, "../data/icosphere/uos/", UOS, 0, -1);
    // make sure that there are exactly one scan
    BOOST_CHECK(Scan::allScans.size() == 1);
    // make sure all scans have the correct number of points
    Scan* scan = *(Scan::allScans.begin());
    DataXYZ xyz(scan->get("xyz"));
    BOOST_CHECK(xyz.size() == 20472);
    // check whether the values of the first point of each scan are the ones
    // they should be
    BOOST_CHECK(xyz[0][0] == -0.525731);
    BOOST_CHECK(xyz[0][1] == 0.850651);
    BOOST_CHECK(xyz[0][2] == 0);

    Scan::closeDirectory();
}

BOOST_AUTO_TEST_CASE(readscans_xyz) {
    Scan::openDirectory(false, "../data/icosphere/xyz/", XYZ, 0, -1);
    // make sure that there are exactly one scan
    BOOST_CHECK(Scan::allScans.size() == 1);
    // make sure all scans have the correct number of points
    Scan* scan = *(Scan::allScans.begin());
    DataXYZ xyz(scan->get("xyz"));
    BOOST_CHECK(xyz.size() == 20472);
    // check whether the values of the first point of each scan are the ones
    // they should be
    // the xyz scanio reader multiplies the input with 100 to convert from
    // meter to cm. This introduces small floating point inaccuracies.
    BOOST_CHECK_CLOSE(xyz[0][0], -0.850651, TOLERANCE);
    BOOST_CHECK_CLOSE(xyz[0][1], 0, TOLERANCE);
    BOOST_CHECK_CLOSE(xyz[0][2], -0.525731, TOLERANCE);

    Scan::closeDirectory();
}

/* vim: set ts=4 sw=4 et: */
