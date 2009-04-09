#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <fstream>
using std::ofstream;

#include <boost/graph/graph_traits.hpp>
using boost::graph_traits;

#include "elch6Deuler.h"

void elch6Deuler::close_loop(const vector <Scan *> &allScans, int first, int last, graph_t &g)
{
  int n = num_vertices(g);
  graph_t grb[6];
  Matrix C(6, 6);
  graph_traits <graph_t>::edge_iterator ei, ei_end;
  for(tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
    int from = source(*ei, g);
    int to = target(*ei, g);
    my_icp6D->covarianceEuler(allScans[from], allScans[to], &C);
    for(int j = 0; j < 6; j++) {
      add_edge(from, to, C(j + 1, j + 1), grb[j]);
    }
  }

  double weights[6][n];
  for(int i = 0; i < 6; i++) {
    graph_balancer(grb[i], first, last, weights[i]);
  }

  vector <Scan *> meta_start;
  meta_start.push_back(allScans[first]);
  meta_start.push_back(allScans[first + 1]);
  meta_start.push_back(allScans[first + 2]);
  Scan *start = new Scan(meta_start, false);
  vector <Scan *> meta_end;
  meta_end.push_back(allScans[last - 2]);
  meta_end.push_back(allScans[last - 1]);
  meta_end.push_back(allScans[last]);
  Scan *end = new Scan(meta_end, false);

  for(int i = last - 2; i <= last; i++) {
    for(int j = 0; j < 6; j++) {
      weights[j][i] = 0.0;
    }
  }

  double delta[6];
  delta[0] = allScans[last]->get_rPos()[0];
  delta[1] = allScans[last]->get_rPos()[1];
  delta[2] = allScans[last]->get_rPos()[2];
  delta[3] = allScans[last]->get_rPosTheta()[0];
  delta[4] = allScans[last]->get_rPosTheta()[1];
  delta[5] = allScans[last]->get_rPosTheta()[2];

  my_icp6D->match(start, end);

  delete start;
  delete end;
  
  delta[0] = allScans[last]->get_rPos()[0] - delta[0];
  delta[1] = allScans[last]->get_rPos()[1] - delta[1];
  delta[2] = allScans[last]->get_rPos()[2] - delta[2];
  delta[3] = allScans[last]->get_rPosTheta()[0] - delta[3];
  delta[4] = allScans[last]->get_rPosTheta()[1] - delta[4];
  delta[5] = allScans[last]->get_rPosTheta()[2] - delta[5];

  if(!quiet) {
    cout << delta[0] << " " << delta[1] << " " << delta[2] << " " << delta[3] << " " << delta[4] << " " << delta[5] << endl;
  }

  double rPos[3], rPosTheta[3];
  for(int i = 1; i < n; i++) {
    rPos[0] = allScans[i]->get_rPos()[0] + delta[0] * (weights[0][i] - weights[0][0]);
    rPos[1] = allScans[i]->get_rPos()[1] + delta[1] * (weights[1][i] - weights[1][0]);
    rPos[2] = allScans[i]->get_rPos()[2] + delta[2] * (weights[2][i] - weights[2][0]);
    rPosTheta[0] = allScans[i]->get_rPosTheta()[0] + delta[3] * (weights[3][i] - weights[3][0]);
    rPosTheta[1] = allScans[i]->get_rPosTheta()[1] + delta[4] * (weights[4][i] - weights[4][0]);
    rPosTheta[2] = allScans[i]->get_rPosTheta()[2] + delta[5] * (weights[5][i] - weights[5][0]);

    allScans[i]->transformToEuler(rPos, rPosTheta, i == n-1 ? 2 : 1);
  }
}
