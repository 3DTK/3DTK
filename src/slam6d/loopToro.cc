/**
 * @file TORO wrapper
 * @author Jochen Sprickerhof. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include <fstream>
using std::ofstream;
using std::ifstream;

#include <boost/graph/graph_traits.hpp>
using boost::graph_traits;

#include "slam6d/loopToro.h"
#include "slam6d/lum6Deuler.h"
#include "slam6d/globals.icc"
using namespace NEWMAT;
/**
 * @param allScans all laser scans
 * @param first index of first laser scan in the loop
 * @param last indes of last laser scan in the loop
 * @param g graph for loop optimization
 */
void loopToro::close_loop(const vector <Scan *> &allScans, int first, int last, graph_t &g)
{
  int n = num_vertices(g);
  Matrix C(6, 6);
  double invers[16], rela[16], rPos[3], rPosTheta[3], rPosQuat[4];
  ofstream outFile("toro.graph");

  for(int i = 0; i < n; i++) {
    QuatRPYEuler(allScans[i]->get_rPosQuat(), rPosTheta);
    outFile << "VERTEX3" << " " << i <<
      " " << (allScans[i]->get_rPos()[0]/100) <<
      " " << (allScans[i]->get_rPos()[1]/100) <<
      " " << (allScans[i]->get_rPos()[2]/100) <<
      " " << rPosTheta[0] <<
      " " << rPosTheta[1] <<
      " " << rPosTheta[2] << "\n";
  }

  graph_traits <graph_t>::edge_iterator ei = edges(g).first;
  int num_arcs = num_edges(g);
  int li = 0;
#ifdef _OPENMP
#pragma omp parallel for firstprivate(li, ei) private(invers, rela, rPos, rPosTheta, rPosQuat, C)
#endif
  for(int i = 0; i < num_arcs; i++) {
    for(;i > li; li++, ei++) ;
    for(;i < li; li--, ei--) ;
    int from = source(*ei, g);
    int to = target(*ei, g);

    M4inv(allScans[from]->get_transMat(), invers);
    MMult(invers, allScans[to]->get_transMat(), rela);
    Matrix4ToQuat(rela, rPosQuat, rPos);
    QuatRPYEuler(rPosQuat, rPosTheta);

    lum6DEuler::covarianceEuler(allScans[from], allScans[to], my_icp6D->get_nns_method(), my_icp6D->get_rnd(), my_icp6D->get_max_dist_match2(), &C);

#ifdef _OPENMP
#pragma omp critical
#endif
    {
    outFile << "EDGE3" << " " << from << " " << to << " " <<
      (rPos[0]/100) << " " <<
      (rPos[1]/100) << " " <<
      (rPos[2]/100) << " " <<
      rPosTheta[0] << " " <<
      rPosTheta[1] << " " <<
      rPosTheta[2] << " " <<
      C(1, 1) << " " << C(1, 2) << " " << C(1, 3) << " " << C(1, 4) << " " << C(1, 5) << " " << C(1, 6) << " " <<
      C(1, 1) << " " << C(1, 2) << " " << C(1, 3) << " " << C(1, 4) << " " << C(1, 5) << " " <<
      C(1, 1) << " " << C(1, 2) << " " << C(1, 3) << " " << C(1, 4) << " " <<
      C(1, 1) << " " << C(1, 2) << " " << C(1, 3) << " " <<
      C(1, 1) << " " << C(1, 2) << " " <<
      C(1, 1) << " " << "\n";
    }
  }

  vector <Scan *> meta_start;
  for(int i = first - 2; i <= first + 2; i++) {
    if(i >= 0) {
      meta_start.push_back(allScans[i]);
    }
  }
  Scan *start = new Scan(meta_start, false, false);

  //static size of metascan
  int offset_last_start = 2;
  int offset_last_end = 0;

  vector <Scan *> meta_end;
  for(int i = last - offset_last_start; i <= last + offset_last_end && i < n; i++) {
    meta_end.push_back(allScans[i]);
  }
  Scan *end = new Scan(meta_end, false, false);

  my_icp6D->match(start, end);

  delete start;
  delete end;

  M4inv(allScans[last]->get_transMat(), invers);
  MMult(invers, allScans[first]->get_transMat(), rela);
  Matrix4ToQuat(rela, rPosQuat, rPos);
  QuatRPYEuler(rPosQuat, rPosTheta);

  lum6DEuler::covarianceEuler(allScans[first], allScans[last], my_icp6D->get_nns_method(), my_icp6D->get_rnd(), my_icp6D->get_max_dist_match2(), &C);

  outFile << "EDGE3" << " " << last << " " << first << " " <<
    (rPos[0]/100) << " " <<
    (rPos[1]/100) << " " <<
    (rPos[2]/100) << " " <<
    rPosTheta[0] << " " <<
    rPosTheta[1] << " " <<
    rPosTheta[2] << " " <<
    C(1, 1) << " " << C(1, 2) << " " << C(1, 3) << " " << C(1, 4) << " " << C(1, 5) << " " << C(1, 6) << " " <<
    C(1, 1) << " " << C(1, 2) << " " << C(1, 3) << " " << C(1, 4) << " " << C(1, 5) << " " <<
    C(1, 1) << " " << C(1, 2) << " " << C(1, 3) << " " << C(1, 4) << " " <<
    C(1, 1) << " " << C(1, 2) << " " << C(1, 3) << " " <<
    C(1, 1) << " " << C(1, 2) << " " <<
    C(1, 1) << " " << "\n";
  outFile.close();

  system("sort toro.graph > toro2.graph && mv toro2.graph toro.graph && ./bin/toro3d -i 300 toro.graph");

  ifstream inFile("toro-treeopt-final.graph");
  string tag;
  int id;
  double dd;
  double rPosN[3], rPosThetaN[3];
  while(inFile) {
    inFile >> tag;
    if(tag == "VERTEX3") {
      inFile >> id;
      if(id == n-1) {
        inFile >> rPosN[0] >> rPosN[1] >> rPosN[2] >> rPosThetaN[0] >> rPosThetaN[1] >> rPosThetaN[2];
        rPosN[0] *= 100;
        rPosN[1] *= 100;
        rPosN[2] *= 100;
      } else {
        inFile >> rPos[0] >> rPos[1] >> rPos[2] >> rPosTheta[0] >> rPosTheta[1] >> rPosTheta[2];
        rPos[0] *= 100;
        rPos[1] *= 100;
        rPos[2] *= 100;
        RPYEulerQuat(rPosTheta, rPosQuat);
        if(id != 0) {
          allScans[id]->transformToQuat(rPos, rPosQuat, Scan::LOOPTORO, 1);
        }
      }
    }
    else if(tag == "EDGE3") {
      inFile >> id >> id;
      for(int i=0; i < 22; i++) {
        inFile >> dd;
      }
    }
  }
  RPYEulerQuat(rPosThetaN, rPosQuat);
  allScans[n-1]->transformToQuat(rPosN, rPosQuat, Scan::LOOPTORO, 2);
  inFile.close();
}
