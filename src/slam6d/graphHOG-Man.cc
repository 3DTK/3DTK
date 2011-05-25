/**
 * @file HOG-Man wrapper
 * @author Jochen Sprickerhof. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include <fstream>
using std::ofstream;
using std::ifstream;
#include <cfloat>
#include <cstring>
#include "slam6d/graphHOG-Man.h"
#include "slam6d/lum6Deuler.h"
#include "slam6d/globals.icc"
using namespace NEWMAT;
/**
 * This function is used to match a set of laser scans with any minimally
 * connected Graph.
 *
 * @param gr Some Graph with no real subgraphs except for itself
 * @param allScans Contains all laser scans
 * @param nrIt The number of iterations the LUM-algorithm will run
 * @return Euclidian distance of all pose shifts
 */
double graphHOGMan::doGraphSlam6D(Graph gr, vector <Scan *> allScans, int nrIt)
{
  Matrix C(6, 6);
  double invers[16], rela[16], rPos[3], rPosTheta[3], rPosQuat[4];
  double Pl0[16];

  ofstream outFile("hogman.graph");
  int n = gr.getNrScans();
  for(int i = 0; i < n; i++) {
    QuatRPYEuler(allScans[i]->get_rPosQuat(), rPosTheta);
    outFile << "VERTEX3" << " " << i <<
      " " << (allScans[i]->get_rPos()[0]/100) <<
      " " << (allScans[i]->get_rPos()[1]/100) <<
      " " << (allScans[i]->get_rPos()[2]/100) <<
      " " << rPosTheta[0] <<
      " " << rPosTheta[1] <<
      " " << rPosTheta[2] << endl;
  }

  for(int i = 0; i < gr.getNrLinks(); i++){
    int first = gr.getLink(i,0);
    int last = gr.getLink(i,1);

    if(first != last-1) {
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
        if(i >= 0) {
          meta_end.push_back(allScans[i]);
        }
      }
      Scan *end = new Scan(meta_end, false, false);

      memcpy(Pl0, allScans[last]->get_transMat(), 16 * sizeof(double));
      my_icp->match(start, end);

      delete start;
      delete end;
    }

    M4inv(allScans[last]->get_transMat(), invers);
    MMult(invers, allScans[first]->get_transMat(), rela);
    Matrix4ToQuat(rela, rPosQuat, rPos);
    QuatRPYEuler(rPosQuat, rPosTheta);

    lum6DEuler::covarianceEuler(allScans[first], allScans[last], my_icp->get_nns_method(), my_icp->get_rnd(), my_icp->get_max_dist_match2(), &C);

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
      C(1, 1) << " " << endl;

    if(first != last-1) {
      allScans[last]->transformToMatrix(Pl0,Scan::INVALID);
    }
  }
  outFile.close();

  system("LD_LIBRARY_PATH=./bin/ ./bin/hogman3d -update 1 -oc -o hogman-final.graph hogman.graph");

  ifstream inFile("hogman-final.graph");
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
          allScans[id]->transformToQuat(rPos, rPosQuat, Scan::GRAPHHOGMAN, 1);
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
  allScans[n-1]->transformToQuat(rPosN, rPosQuat, Scan::GRAPHHOGMAN, 2);
  inFile.close();

  return DBL_MAX;
}
