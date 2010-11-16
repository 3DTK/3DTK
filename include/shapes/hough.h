#ifndef __HOUGH_PLANE_H__
#define __HOUGH_PLANE_H__

#include "shapes/convexplane.h"
#include "shapes/ConfigFileHough.h"
#include "wykobi/wykobi_algorithm.hpp"
#include "slam6d/point.h"
#include "slam6d/scan.h"
#include "shapes/accumulator.h"
#include "newmat/newmatio.h"
#include <iostream>
using std::ofstream;
typedef vector <PtPair> vPtPair;  ///< just a typedef: vPtPair = vector of type PtPair

enum plane_alg { 
  RHT, SHT, PHT, PPHT, APHT, RANSAC
};

struct valuecompare {
  bool operator()(int* ip1, int* ip2) const {
    if(ip1[0] > ip2[0]) {
      return true;
    }else {
      return false;
    }
  }
};

class Hough {

public:
  ofstream out; 
  ConfigFileHough myConfigFileHough;
  Accumulator *acc;

  int nrEntries;
  vector <Point>* allPoints;
  bool maximum;  
  bool quiet;
  Scan *PlaneScan;
  Scan *PlaneScan2;
  int planeCounter;
  int scanCounter;
  vector<ConvexPlane> planes;

  Hough(Scan * GlobalScan, bool quiet = true );
  ~Hough();
  void RHT();
  void SHT();
  void PHT();
  void PPHT();
  void APHT();

  bool distanceOK(Point p1, Point p2, Point p3);
  bool calculatePlane(Point p1, Point p2, Point p3, double &theta, double &phi, double &rho); 

  int deletePoints(double * n, double rho);
  int deletePointsQuad(double * n, double rho);

  void writePlanes();
  int cluster(vPtPair &pairs, double minx, double max, double miny, double maxy);
  void writeAllPoints(int index, vector<Point> points);

};

double calcPlane(vector<Point> &ppoint, double plane[4]);

#endif
