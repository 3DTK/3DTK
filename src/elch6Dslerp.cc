#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <fstream>
using std::ofstream;

#include <cstring>

#include <boost/graph/graph_traits.hpp>
using boost::graph_traits;

#include "elch6Dslerp.h"
#include "lum6Dquat.h"

void elch6Dslerp::close_loop(const vector <Scan *> &allScans, int first, int last, graph_t &g)
{
  int n = num_vertices(g);
  graph_t grb[4];
  Matrix C(7, 7);
  graph_traits <graph_t>::edge_iterator ei, ei_end;
  for(tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
    int from = source(*ei, g);
    int to = target(*ei, g);
    lum6DQuat::covarianceQuat(allScans[from], allScans[to], my_icp6D->get_use_cache(), my_icp6D->get_rnd(), my_icp6D->get_max_dist_match2(), &C);
    C = C.i();
    for(int j = 0; j < 3; j++) {
      add_edge(from, to, C(j + 1, j + 1), grb[j]);
    }
    add_edge(from, to, C(4, 4) + C(5, 5) + C(6, 6) + C(7, 7), grb[3]);
  }

  double *weights[4];
  for(int i = 0; i < 4; i++) {
    weights[i] = new double[n];
    graph_balancer(grb[i], first, last, weights[i]);
  }

  vector <Scan *> meta_start;
  for(int i = first - 2; i <= first + 2; i++) {
    if(i > 0) {
      meta_start.push_back(allScans[i]);
    }
  }
  Scan *start = new Scan(meta_start, false);

  int offset_last_start = 2;
  int offset_last_end = 0;
  vector <Scan *> meta_end;
  for(int i = last - offset_last_start; i <= last + offset_last_end && i < n; i++) {
    meta_end.push_back(allScans[i]);
  }
  Scan *end = new Scan(meta_end, false);

  double p1[16];
  memcpy(p1, allScans[last]->get_transMat(), 16 * sizeof(double));

  my_icp6D->match(start, end);

  delete start;
  delete end;

  // store ICP
  double p1P[16];
  memcpy(p1P, allScans[last]->get_transMat(), 16 * sizeof(double));

  // Move Origin
  double p1P_inv[16];
  M4inv(p1P, p1P_inv);
  for(int i = 0; i < n; i++) {
    allScans[i]->transform(p1P_inv, Scan::INVALID, -1);
  }

  //cout << "0 == " << allScans[last]->get_transMat() << endl;

  double p1N[16];
  MMult(p1P_inv, p1, p1N);
  double deltaM[16];
  M4inv(p1N, deltaM);

  double deltaT[3], deltaQ[4];
  Matrix4ToQuat(deltaM, deltaQ, deltaT);

  if(!quiet) {
    double axisangle[4];
    axisangle[0] = deltaQ[0];
    axisangle[1] = deltaQ[1];
    axisangle[2] = deltaQ[2];
    axisangle[3] = deltaQ[3];
    QuatToAA(axisangle);
    cout << deltaT[0] << " " << deltaT[1] << " " << deltaT[2] << " " << axisangle[0] << " " << axisangle[1] << " " << axisangle[2] << " " << axisangle[3] << endl;
  }

  double rPos[3], rPosQuat[4], idM[16], idQ[4] = {1, 0, 0, 0};
  M4identity(idM);

  for(int i = 0; i < n; i++) {

    rPos[0] = deltaT[0] * weights[0][i];
    rPos[1] = deltaT[1] * weights[1][i];
    rPos[2] = deltaT[2] * weights[2][i];
    slerp(idQ, deltaQ, weights[3][i], rPosQuat);

    //cout << weights[0][i] << " " << weights[1][i] << " " << weights[2][i] << " " << weights[3][i] << endl;

    if(i >= last - offset_last_start && i <= last + offset_last_end && i < n) {
      allScans[i]->transform(idM, Scan::INVALID, -1);
    } else {
      allScans[i]->transform(rPosQuat, rPos, Scan::INVALID, -1);
    }
  }

  // Move Origin
  allScans[0]->transform(p1P, Scan::INVALID, -1);
  for(int i = 1; i < n; i++) {
    allScans[i]->transform(p1P, Scan::LUM, i == n-1 ? 2 : 1);
  }

  for(int i = 0; i < 4; i++) {
    delete [] weights[i];
  }

}
