#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE scanio
#include <boost/test/unit_test.hpp>

#include <scanio/helper.h>
#include <scanio/scan_io.h>
#include "slam6d/io_types.h"

BOOST_AUTO_TEST_CASE(zipreader) {
	PointFilter filter;
	std::vector<float> reflectance;
	std::vector<float> temperature;
	std::vector<float> amplitude;
	std::vector<int> type;
	std::vector<float> deviation;
	std::vector<std::string> identifiers0 = { "000" };

	// read first directory
	ScanIO* sio1 = ScanIO::getScanIO(UOS_RGB);
	char *path1 = "../../testing/data/icosphere/uos/normals";
	std::list<std::string> identifiers1 = sio1->readDirectory(path1, 0, 0);
	BOOST_CHECK_EQUAL_COLLECTIONS(identifiers1.begin(), identifiers1.end(), identifiers0.begin(), identifiers0.end());
	std::vector<double> xyz1;
	std::vector<unsigned char> rgb1;
	sio1->readScan(path1, "000", filter, &xyz1, &rgb1, &reflectance, &temperature, &amplitude, &type, &deviation);

	// then read zip file
	ScanIO* sio2 = ScanIO::getScanIO(UOS_RGB);
	char *path2 = "../../testing/data/icosphere/uos/normals.zip/normals";
	std::list<std::string> identifiers2 = sio2->readDirectory(path2, 0, 0);
	BOOST_CHECK_EQUAL_COLLECTIONS(identifiers2.begin(), identifiers2.end(), identifiers0.begin(), identifiers0.end());
	std::vector<double> xyz2;
	std::vector<unsigned char> rgb2;
	sio2->readScan(path2, "000", filter, &xyz2, &rgb2, &reflectance, &temperature, &amplitude, &type, &deviation);
	std::cerr << xyz1.size() << std::endl;

	// the pointcloud is an icosphere created through five recursions
	//
	// the number of vertices in an icosphere created with N recursions is
	// equal to (5*4^N-2)/3 times 12 (the number of vertices in a regular
	// icosahedron). The sequence created by (5*4^N-2)/3 is known as A020989:
	// http://oeis.org/A020989
	// 1, 6, 26, 106, 426, 1706, ...
	//
	// since the xyz vector contains a flattened list of the coordinates, the
	// result has to be multiplied by three as every coordinate has three
	// degrees
	//
	// in our case, N=5
	BOOST_CHECK(xyz1.size() == ((5*pow(4, 5)-2)/3)*12*3);

	// finally compare
	BOOST_CHECK_EQUAL_COLLECTIONS(xyz1.begin(), xyz1.end(), xyz2.begin(), xyz2.end());
	BOOST_CHECK_EQUAL_COLLECTIONS(rgb1.begin(), rgb1.end(), rgb2.begin(), rgb2.end());
}

/* vim: set ts=4 sw=4 et: */
