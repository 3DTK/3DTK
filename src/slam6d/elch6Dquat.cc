/*
 * elch6Dquat implementation
 *
 * Copyright (C) Jochen Sprickerhof
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file ELCH implementation using Quaternions
 * @author Jochen Sprickerhof. Inst. CS, University of Osnabrueck, Germany.
 */

#include "slam6d/elch6Dquat.h"

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
 * ELCH loop closing function using Quaternions
 * matches first and last scan of a loop with ICP
 * distributes the error
 *
 * @param allScans all laser scans
 * @param first index of first laser scan in the loop
 * @param last indes of last laser scan in the loop
 * @param g graph for loop optimization
 */
void elch6Dquat::close_loop(const vector <Scan *> &allScans,
					   int first,
					   int last,
					   graph_t &g)
{
  int n = num_vertices(g);
  graph_t grb[7];
  Matrix C(7, 7);
  graph_traits <graph_t>::edge_iterator ei, ei_end;
  for(boost::tuples::tie(ei, ei_end) = edges(g); ei != ei_end; ei++) {
    int from = source(*ei, g);
    int to = target(*ei, g);
    lum6DQuat::covarianceQuat(allScans[from],
						allScans[to],
						my_icp6D->get_nns_method(),
						my_icp6D->get_rnd(),
						my_icp6D->get_max_dist_match2(), &C);
    C = C.i();
    for(int j = 0; j < 7; j++) {
      add_edge(from, to, fabs(C(j + 1, j + 1)), grb[j]);
    }
  }

  double *weights[7];
  for(int i = 0; i < 7; i++) {
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

  for(int i = last - 2; i <= last; i++) {
    for(int j = 0; j < 7; j++) {
      weights[j][i] = 0.0;
    }
  }

  double delta[7];
  delta[0] = allScans[last]->get_rPos()[0];
  delta[1] = allScans[last]->get_rPos()[1];
  delta[2] = allScans[last]->get_rPos()[2];
  delta[3] = allScans[last]->get_rPosQuat()[0];
  delta[4] = allScans[last]->get_rPosQuat()[1];
  delta[5] = allScans[last]->get_rPosQuat()[2];
  delta[6] = allScans[last]->get_rPosQuat()[3];

  my_icp6D->match(start, end);

  delete start;
  delete end;
  
  delta[0] = allScans[last]->get_rPos()[0] - delta[0];
  delta[1] = allScans[last]->get_rPos()[1] - delta[1];
  delta[2] = allScans[last]->get_rPos()[2] - delta[2];
  delta[3] = allScans[last]->get_rPosQuat()[0] - delta[3];
  delta[4] = allScans[last]->get_rPosQuat()[1] - delta[4];
  delta[5] = allScans[last]->get_rPosQuat()[2] - delta[5];
  delta[6] = allScans[last]->get_rPosQuat()[3] - delta[6];

  if(!quiet) {
    double axisangle[4];
    axisangle[0] = delta[3];
    axisangle[1] = delta[4];
    axisangle[2] = delta[5];
    axisangle[3] = delta[6];
    QuatToAA(axisangle);
    cout << "Delta: " << delta[0] << " " << delta[1] << " " << delta[2] << " "
	    << axisangle[0] << " " << axisangle[1] << " "
	    << axisangle[2] << " " << axisangle[3] << endl;
  }

  double rPos[3], rPosQuat[4];
  for(int i = 1; i < n; i++) {
    rPos[0] = allScans[i]->get_rPos()[0] +
	 delta[0] * (weights[0][i] - weights[0][0]);
    rPos[1] = allScans[i]->get_rPos()[1] +
	 delta[1] * (weights[1][i] - weights[1][0]);
    rPos[2] = allScans[i]->get_rPos()[2] +
	 delta[2] * (weights[2][i] - weights[2][0]);
    rPosQuat[0] = allScans[i]->get_rPosQuat()[0] +
	 delta[3] * (weights[3][i] - weights[3][0]);
    rPosQuat[1] = allScans[i]->get_rPosQuat()[1] +
	 delta[4] * (weights[4][i] - weights[4][0]);
    rPosQuat[2] = allScans[i]->get_rPosQuat()[2] +
	 delta[5] * (weights[5][i] - weights[5][0]);
    rPosQuat[3] = allScans[i]->get_rPosQuat()[3] +
	 delta[6] * (weights[6][i] - weights[6][0]);

    Normalize4(rPosQuat);
    allScans[i]->transformToQuat(rPos, rPosQuat, Scan::ELCH, i == n-1 ? 2 : 1);
  }

  for(int i = 0; i < 7; i++) {
    delete [] weights[i];
  }

}
