/*
 * elch6DunitQuat implementation
 *
 * Copyright (C) Jochen Sprickerhof
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file ELCH implementation using unit Quaternions
 * @author Jochen Sprickerhof. Inst. of CS, University of Osnabrueck, Germany.
 */


#include "slam6d/elch6DunitQuat.h"

#include "slam6d/metaScan.h"
#include "slam6d/lum6Dquat.h"
#include "slam6d/globals.icc"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <fstream>
using std::ofstream;

#include <boost/graph/graph_traits.hpp>
using boost::graph_traits;
using namespace NEWMAT;

/**
 * ELCH loop closing function using unit Quaternion
 * matches first and last scan of a loop with ICP
 * distributes the error
 *
 * @param allScans all laser scans
 * @param first index of first laser scan in the loop
 * @param last indes of last laser scan in the loop
 * @param g graph for loop optimization
 */
void elch6DunitQuat::close_loop(const vector <Scan *> &allScans,
                                int first,
                                int last,
                                graph_t &g)
{
  int n = num_vertices(g);
  graph_t grb[4];
  Matrix C(7, 7);
  graph_traits <graph_t>::edge_iterator ei, ei_end;
  for(boost::tuples::tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
    int from = source(*ei, g);
    int to = target(*ei, g);
    lum6DQuat::covarianceQuat(allScans[from], allScans[to],
                              my_icp6D->get_nns_method(),
                              my_icp6D->get_rnd(),
                              my_icp6D->get_max_dist_match2(),
                              &C);
    C = C.i();
    for(int j = 0; j < 3; j++) {
      add_edge(from, to, abs(C(j + 1, j + 1)), grb[j]);
    }
    add_edge(from, to,
             abs(C(4, 4)) + abs(C(5, 5)) + abs(C(6, 6)) + abs(C(7, 7)), grb[3]);
  }

  double *weights[4];
  for(int i = 0; i < 4; i++) {
    weights[i] = new double[n];
    graph_balancer(grb[i], first, last, weights[i]);
  }

  vector <Scan *> meta_start;
  meta_start.push_back(allScans[first]);
  meta_start.push_back(allScans[first + 1]);
  meta_start.push_back(allScans[first + 2]);
  MetaScan *start = new MetaScan(meta_start, false);
  vector <Scan *> meta_end;
  meta_end.push_back(allScans[last - 2]);
  meta_end.push_back(allScans[last - 1]);
  meta_end.push_back(allScans[last]);
  MetaScan *end = new MetaScan(meta_end, false);

  // save poses bevor ICP
  double pOld1[7], pOld2[7], pOld3[7];
  pOld1[0] = allScans[last]->get_rPos()[0];
  pOld1[1] = allScans[last]->get_rPos()[1];
  pOld1[2] = allScans[last]->get_rPos()[2];
  pOld1[3] = allScans[last]->get_rPosQuat()[0];
  pOld1[4] = allScans[last]->get_rPosQuat()[1];
  pOld1[5] = allScans[last]->get_rPosQuat()[2];
  pOld1[6] = allScans[last]->get_rPosQuat()[3];

  pOld2[0] = allScans[last - 1]->get_rPos()[0];
  pOld2[1] = allScans[last - 1]->get_rPos()[1];
  pOld2[2] = allScans[last - 1]->get_rPos()[2];
  pOld2[3] = allScans[last - 1]->get_rPosQuat()[0];
  pOld2[4] = allScans[last - 1]->get_rPosQuat()[1];
  pOld2[5] = allScans[last - 1]->get_rPosQuat()[2];
  pOld2[6] = allScans[last - 1]->get_rPosQuat()[3];

  pOld3[0] = allScans[last - 2]->get_rPos()[0];
  pOld3[1] = allScans[last - 2]->get_rPos()[1];
  pOld3[2] = allScans[last - 2]->get_rPos()[2];
  pOld3[3] = allScans[last - 2]->get_rPosQuat()[0];
  pOld3[4] = allScans[last - 2]->get_rPosQuat()[1];
  pOld3[5] = allScans[last - 2]->get_rPosQuat()[2];
  pOld3[6] = allScans[last - 2]->get_rPosQuat()[3];

  double delta[3];
  delta[0] = allScans[last]->get_rPos()[0];
  delta[1] = allScans[last]->get_rPos()[1];
  delta[2] = allScans[last]->get_rPos()[2];
  double q1[4];
  q1[0] = allScans[last]->get_rPosQuat()[0];
  q1[1] = -allScans[last]->get_rPosQuat()[1];
  q1[2] = -allScans[last]->get_rPosQuat()[2];
  q1[3] = -allScans[last]->get_rPosQuat()[3];

  my_icp6D->match(start, end);

  delete start;
  delete end;
  
  delta[0] = allScans[last]->get_rPos()[0] - delta[0];
  delta[1] = allScans[last]->get_rPos()[1] - delta[1];
  delta[2] = allScans[last]->get_rPos()[2] - delta[2];
  double q2[4];
  q2[0] = allScans[last]->get_rPosQuat()[0];
  q2[1] = allScans[last]->get_rPosQuat()[1];
  q2[2] = allScans[last]->get_rPosQuat()[2];
  q2[3] = allScans[last]->get_rPosQuat()[3];

  double deltaQ[4];
  QMult(q2, q1, deltaQ); // q3 = q2*q1^-1

  if(!quiet) {
    double axisangle[4];
    axisangle[0] = deltaQ[0];
    axisangle[1] = deltaQ[1];
    axisangle[2] = deltaQ[2];
    axisangle[3] = deltaQ[3];
    QuatToAA(axisangle);
    cout << "Delta: " << delta[0] << " " << delta[1] << " " << delta[2]
         << " " << axisangle[0] << " " << axisangle[1]
         << " " << axisangle[2] << " " << axisangle[3] << endl;
  }

  // restore poses after ICP matching
  allScans[last]->transformToQuat(pOld1, &pOld1[3], Scan::INVALID, -1);
  allScans[last - 1]->transformToQuat(pOld2, &pOld2[3], Scan::INVALID, -1);
  allScans[last - 2]->transformToQuat(pOld3, &pOld3[3], Scan::INVALID, -1);

  //compute inverse rotation of Scan 0
  double scan0Pdelta[4], scan0Q[4];
  QMult(deltaQ, allScans[0]->get_rPosQuat(), scan0Pdelta);
  scan0Q[0] = (1 - weights[3][0]) * allScans[0]->get_rPosQuat()[0] +
    scan0Pdelta[0] * weights[3][0];
  scan0Q[1] = -1 * ((1 - weights[3][0]) * allScans[0]->get_rPosQuat()[1] +
                    scan0Pdelta[1] * weights[3][0]);
  scan0Q[2] = -1 * ((1 - weights[3][0]) * allScans[0]->get_rPosQuat()[2] +
                    scan0Pdelta[2] * weights[3][0]);
  scan0Q[3] = -1 * ((1 - weights[3][0]) * allScans[0]->get_rPosQuat()[3] +
                    scan0Pdelta[3] * weights[3][0]);
  Normalize4(scan0Q);
  QMult(allScans[0]->get_rPosQuat(), scan0Q, scan0Pdelta);

  double rPos[3], rPosQuat[4], tmpquat[4];
  for(int i = 1; i < n; i++) {
    rPos[0] = allScans[i]->get_rPos()[0] +
      delta[0] * (weights[0][i] - weights[0][0]);
    rPos[1] = allScans[i]->get_rPos()[1] +
      delta[1] * (weights[1][i] - weights[1][0]);
    rPos[2] = allScans[i]->get_rPos()[2] +
      delta[2] * (weights[2][i] - weights[2][0]);
    QMult(deltaQ, allScans[i]->get_rPosQuat(), rPosQuat);
    tmpquat[0] = (1 - weights[3][i]) * allScans[i]->get_rPosQuat()[0] +
      rPosQuat[0] * weights[3][i];
    tmpquat[1] = (1 - weights[3][i]) * allScans[i]->get_rPosQuat()[1] +
      rPosQuat[1] * weights[3][i];
    tmpquat[2] = (1 - weights[3][i]) * allScans[i]->get_rPosQuat()[2] +
      rPosQuat[2] * weights[3][i];
    tmpquat[3] = (1 - weights[3][i]) * allScans[i]->get_rPosQuat()[3] +
      rPosQuat[3] * weights[3][i];
    Normalize4(tmpquat);
    QMult(scan0Pdelta, tmpquat, rPosQuat);

    Normalize4(rPosQuat);
    allScans[i]->transformToQuat(rPos, rPosQuat, Scan::ELCH, i == n-1 ? 2 : 1);
  }

  for(int i = 0; i < 4; i++) {
    delete [] weights[i];
  }

}
