#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE segmentation
#include <boost/test/unit_test.hpp>
#include <segmentation/disjoint-set.h>

using namespace std;

BOOST_AUTO_TEST_CASE(disjoint_set) {
  universe uni(4);

  BOOST_CHECK(uni.num_sets() == 4);
  
  uni.join(1, 0);
  uni.join(0, 2);
  uni.join(3, 0);

  BOOST_CHECK(uni.size(0) == 4);
  BOOST_CHECK(uni.size(1) == 1);
  BOOST_CHECK(uni.size(2) == 1);
  BOOST_CHECK(uni.size(3) == 1);

  for(int i = 0; i < 4; i++) {
    BOOST_CHECK(uni.find(i) == 0);
  }
}
