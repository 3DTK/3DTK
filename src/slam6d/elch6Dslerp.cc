/*
 * elch6Dslerp implementation
 *
 * Copyright (C) Jochen Sprickerhof
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file ELCH implementation using SLERP
 * @author Jochen Sprickerhof. Inst. of CS, University of Osnabrueck, Germany.
 */

#include "slam6d/elch6Dslerp.h"

#include "slam6d/metaScan.h"
#include "slam6d/lum6Dquat.h"
#include "slam6d/globals.icc"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <fstream>
using std::ofstream;

#include <cstring>

#include <boost/graph/graph_traits.hpp>
using boost::graph_traits;
using namespace NEWMAT;
/**
 * ELCH loop closing function using SLERP
 * matches first and last scan of a loop with ICP
 * distributes the error
 *
 * @param allScans all laser scans
 * @param first index of first laser scan in the loop
 * @param last indes of last laser scan in the loop
 * @param g graph for loop optimization
 */
void elch6Dslerp::close_loop(const vector <Scan *> &allScans,
                             int first,
                             int last,
                             graph_t &g)
{
  int n = num_vertices(g);
  graph_t grb[4];
  graph_traits <graph_t>::edge_iterator ei = edges(g).first;
  int num_arcs = num_edges(g);
  int li = 0;
#ifdef _OPENMP
#pragma omp parallel for firstprivate(li, ei)
#endif
  for(int i = 0; i < num_arcs; i++) {
    for(;i > li; li++, ei++) ;
    for(;i < li; li--, ei--) ;
    Matrix C(7, 7);
    int from = source(*ei, g);
    int to = target(*ei, g);
    lum6DQuat::covarianceQuat(allScans[from],
                              allScans[to],
                              my_icp6D->get_nns_method(),
                              my_icp6D->get_rnd(),
                              my_icp6D->get_max_dist_match2(),
                              &C);
    C = C.i();
    for(int j = 0; j < 3; j++) {
#ifdef _OPENMP
#pragma omp critical
#endif
      add_edge(from, to, fabs(C(j + 1, j + 1)), grb[j]);
    }
#ifdef _OPENMP
#pragma omp critical
#endif
    add_edge(from, to,
             fabs(C(4, 4)) + fabs(C(5, 5)) +
             fabs(C(6, 6)) + fabs(C(7, 7)),
             grb[3]);
    li++;
    ei++;
  }

  double *weights[4];
  for(int i = 0; i < 4; i++) {
    weights[i] = new double[n];
    graph_balancer(grb[i], first, last, weights[i]);
  }

  vector <Scan *> meta_start;
  for(int i = first - 2; i <= first + 2; i++) {
    if(i >= 0) {
      meta_start.push_back(allScans[i]);
    }
  }
  MetaScan *start = new MetaScan(meta_start, false);

  //static size of metascan
  int offset_last_start = 2;
  int offset_last_end = 0;

  vector <Scan *> meta_end;
  for(int i = last - offset_last_start;
      i <= last + offset_last_end && i < n;
      i++) {
    meta_end.push_back(allScans[i]);
  }
  MetaScan *end = new MetaScan(meta_end, false);

  double Pl0[16];
  memcpy(Pl0, allScans[last]->get_transMat(), 16 * sizeof(double));

  my_icp6D->match(start, end);

  delete start;
  delete end;

  // store ICP
  double Pp0[16];
  memcpy(Pp0, allScans[last]->get_transMat(), 16 * sizeof(double));

  //compute Delta
  double Pf0[16], Pf0_inv[16], tmp1[16], tmp2[16], deltaf[16];
  memcpy(Pf0, allScans[first]->get_transMat(), 16 * sizeof(double));
  M4inv(Pf0, Pf0_inv);

  MMult(Pf0_inv, Pl0, tmp1);
  M4inv(tmp1, tmp2);
  MMult(Pp0, tmp2, tmp1);
  MMult(Pf0_inv, tmp1, deltaf);

  double deltaT[3], deltaQ[4];
  Matrix4ToQuat(deltaf, deltaQ, deltaT);

  if(!quiet) {
    double axisangle[4];
    axisangle[0] = deltaQ[0];
    axisangle[1] = deltaQ[1];
    axisangle[2] = deltaQ[2];
    axisangle[3] = deltaQ[3];
    QuatToAA(axisangle);
    cout << "Delta: " << deltaT[0] << " " << deltaT[1] << " " << deltaT[2]
         << " " << axisangle[0] << " " << axisangle[1]
         << " " << axisangle[2] << " " << axisangle[3]
         << endl;
  }
  
  //transform scans
  double idQ[4] = {1, 0, 0, 0}, rPos[3], rPosQuat[4], delta0[16];

  rPos[0] = deltaT[0] * weights[0][0];
  rPos[1] = deltaT[1] * weights[1][0];
  rPos[2] = deltaT[2] * weights[2][0];
  slerp(idQ, deltaQ, weights[3][0], rPosQuat);
  QuatToMatrix4(rPosQuat, rPos, tmp1);
  M4inv(tmp1, tmp2);
  MMult(Pf0, tmp2, delta0);

#ifdef _OPENMP
#pragma omp parallel for private(rPos, rPosQuat, tmp1, tmp2)
#endif
  for(int i = 1; i < n; i++) {
    if(i >= last - offset_last_start && i <= last + offset_last_end) {
      MMult(delta0, Pf0_inv, tmp1);
    } else {
      rPos[0] = deltaT[0] * weights[0][i];
      rPos[1] = deltaT[1] * weights[1][i];
      rPos[2] = deltaT[2] * weights[2][i];
      slerp(idQ, deltaQ, weights[3][i], rPosQuat);
      QuatToMatrix4(rPosQuat, rPos, tmp1);
      MMult(delta0, tmp1, tmp2);
      MMult(tmp2, Pf0_inv, tmp1);
    }
    allScans[i]->transform(tmp1, Scan::ELCH, i == n-1 ? 2 : 1);
  }

  for(int i = 0; i < 4; i++) {
    delete [] weights[i];
  }

}
