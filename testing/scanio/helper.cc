#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE scanio
#include <boost/test/unit_test.hpp>
#include <sstream>
#include <slam6d/pointfilter.h>
#include <slam6d/io_types.h>
#include <scanio/helper.h>
#include <scanio/scan_io.h>

using namespace std;

#define TEST BOOST_AUTO_TEST_CASE

// test succeeds if three values can successfully be read
#define readASCII1 do { \
    vector<unsigned char> rgb; \
    IODataType spec[4] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR }; \
    vector<double> xyz, truexyz = { 1.0, 2.0, 3.0 }; \
    PointFilter filter; \
    ScanDataTransform_identity transform; \
    BOOST_CHECK(readASCII(inf, spec, transform, filter, &xyz) == true); \
    BOOST_CHECK_EQUAL_COLLECTIONS(xyz.begin(), xyz.end(), truexyz.begin(), truexyz.end()); } while(0)

// test succeeds if reading fails
#define readASCII2 do { \
    IODataType spec[4] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR }; \
    vector<double> xyz; \
    PointFilter filter; \
    ScanDataTransform_identity transform; \
    BOOST_CHECK(readASCII(inf, spec, transform, filter, &xyz) == false); } while(0)

// test succeeds if six values can successfully be read
#define readASCII3 do { \
    IODataType spec[4] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR }; \
    vector<double> xyz, truexyz = { 1.0, 2.0, 3.0, 4.0, 5.0, 6.0 }; \
    PointFilter filter; \
    ScanDataTransform_identity transform; \
    BOOST_CHECK(readASCII(inf, spec, transform, filter, &xyz) == true); \
    BOOST_CHECK_EQUAL_COLLECTIONS(xyz.begin(), xyz.end(), truexyz.begin(), truexyz.end()); } while(0)

// test succeeds if zero values can successfully be read
#define readASCII4 do { \
    IODataType spec[4] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR }; \
    vector<double> xyz; \
    PointFilter filter; \
    ScanDataTransform_identity transform; \
    BOOST_CHECK(readASCII(inf, spec, transform, filter, &xyz) == true); \
    BOOST_CHECK(xyz.size() == 0); } while(0)

TEST(normal1)         { istringstream inf("1.0 2.0 +3.0");       readASCII1; }
TEST(normal2)         { istringstream inf("1 2 +3");             readASCII1; }
TEST(normal3)         { istringstream inf("1.0E0 2e0 +3e+0");    readASCII1; }
TEST(normal4)         { istringstream inf("0x1 0X2 0x3");        readASCII1; }
TEST(normal5)         { istringstream inf("0x1 0X1p1 0x3p+0");   readASCII1; }
TEST(preceedingSpace) { istringstream inf(" 1 2 3");             readASCII1; }
TEST(preceedingSpace2) { istringstream inf("  1 2 3");             readASCII1; }
TEST(trailingSpace)   { istringstream inf("1 2 3 ");             readASCII1; }
TEST(multipleSpaces)  { istringstream inf("1   2 3");            readASCII1; }
TEST(mixedTabsSpaces) { istringstream inf("1  \t2 3");           readASCII1; }
TEST(beginningGarbage) { istringstream inf("blub\nbla\n1 2 3");  readASCII1; }
TEST(trailingGarbage) { istringstream inf("1aa 2 3");            readASCII4; }
TEST(garbage10)       { istringstream inf("a\nb\nc\nd\ne\nf\ng\nh\ni\nj\n1 2 3"); readASCII1; }
TEST(garbage11)       { istringstream inf("a\nb\nc\nd\ne\nf\ng\nh\ni\nj\nk\n1 2 3"); readASCII2; }
TEST(noNumber)        { istringstream inf("aa 2 3");             readASCII4; }
TEST(windowsLine)     { istringstream inf("1 2 3\r\n");          readASCII1; }
TEST(multipleLines)   { istringstream inf("1 2 3\n4 5 6\n");     readASCII3; }
TEST(emptyLines)      { istringstream inf("1 2 3\n\n\n4 5 6\n"); readASCII3; }
TEST(comments1)       { istringstream inf("1 2 3 # foobar");     readASCII1; }
TEST(comments2)       { istringstream inf("# foobar");           readASCII4; }
TEST(comments3)       { istringstream inf("#");                  readASCII4; }
TEST(comments4)       { istringstream inf(" # foobar");          readASCII4; }
TEST(comments5)       { istringstream inf("1 2 3# foobar");      readASCII1; }
TEST(comments6)       { istringstream inf("1 2 3#");             readASCII1; }
TEST(empty1)          { istringstream inf("");                   readASCII4; }
TEST(empty2)          { istringstream inf("\n");                 readASCII4; }
TEST(empty3)          { istringstream inf("\r\n");               readASCII4; }
TEST(empty4)          { istringstream inf("   ");                readASCII4; }
TEST(empty5)          { istringstream inf("\n\n\n\n");           readASCII4; }
TEST(empty6)          { istringstream inf("\r\n\r\n");           readASCII4; }


TEST(spec1) {
    IODataType spec[5] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR };
    vector<double> xyz; PointFilter filter; ScanDataTransform_identity transform;
    istringstream inf("1.0 2.0 3.0");
    BOOST_CHECK(readASCII(inf, spec, transform, filter, &xyz) == false);
}
TEST(spec2) {
    IODataType spec[3] = { DATA_XYZ, DATA_XYZ, DATA_TERMINATOR };
    vector<double> xyz; PointFilter filter; ScanDataTransform_identity transform;
    istringstream inf("1.0 2.0 3.0");
    BOOST_CHECK(readASCII(inf, spec, transform, filter, &xyz) == false);
}
TEST(spec3) {
    IODataType spec[5] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_REFLECTANCE, DATA_TERMINATOR };
    vector<double> xyz; PointFilter filter; ScanDataTransform_identity transform;
    istringstream inf("1.0 2.0 3.0 4.0");
    BOOST_CHECK(readASCII(inf, spec, transform, filter, &xyz) == false);
}
TEST(spec4) {
    IODataType spec[4] = { DATA_XYZ, DATA_XYZ, DATA_XYZ, DATA_TERMINATOR };
    vector<double> xyz; vector<float> refl; PointFilter filter; ScanDataTransform_identity transform;
    istringstream inf("1.0 2.0 3.0");
    BOOST_CHECK(readASCII(inf, spec, transform, filter, &xyz, 0, &refl) == false);
}
TEST(spec5) {
    IODataType spec[5] = { DATA_XYZ, DATA_XYZ, DATA_DUMMY, DATA_XYZ, DATA_TERMINATOR };
    vector<double> xyz; PointFilter filter; ScanDataTransform_identity transform;
    istringstream inf("1.0 2.0 foobar 3.0");
    BOOST_CHECK(readASCII(inf, spec, transform, filter, &xyz) == true);
}

/* vim: set ts=4 sw=4 et: */
