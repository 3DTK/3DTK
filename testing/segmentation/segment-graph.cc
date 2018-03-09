#define BOOST_TEST_DYN_LINK
#define BOOST_TEST_MODULE segmentation
#include <boost/test/unit_test.hpp>
#include <segmentation/segment-graph.h>

using namespace std;

BOOST_AUTO_TEST_CASE(segmentgraph) {
  edge e[3];
  e[0].w = 3.0; e[0].a = 0; e[0].b = 2;
  e[1].w = 2.0; e[1].a = 1; e[1].b = 2;
  e[2].w = 1.0; e[2].a = 0; e[2].b = 1;
  universe *uni = segment_graph(3, 3, e, 2.0);

  BOOST_CHECK(uni->num_sets() == 1);
  
  BOOST_CHECK(uni->size(0) == 1);
  BOOST_CHECK(uni->size(1) == 3);
  BOOST_CHECK(uni->size(2) == 1);

  for(int i = 0; i < 3; i++) {
    BOOST_CHECK(uni->find(i) == 1);
  }
}
