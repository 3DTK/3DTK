#ifndef __HOUGH_PLANE_H__
#define __HOUGH_PLANE_H__

#include "shapes/convexplane.h"
#include "shapes/ConfigFileHough.h"
//#include "wykobi/wykobi_algorithm.hpp"
#include "slam6d/point.h"
#include "slam6d/scan.h"
#include "shapes/accumulator.h"
#include "newmat/newmatio.h"
#include <iostream>
using std::ofstream;
typedef vector <PtPair> vPtPair;  ///< just a typedef: vPtPair = vector of type PtPair

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
  // TODO delete planes in Constructor
  vector<ConvexPlane*> planes;
  vector<Point> coloredPoints;

  Hough(bool quiet = true, std::string configFile = ""); // this constructor allows the Scan to be set later
  Hough(Scan * GlobalScan, bool quiet = true, std::string configFile = "bin/hough.cfg" );
  void SetScan(Scan*);
  ~Hough();
  int  RHT();
  void SHT();
  void PHT();
  void PPHT();
  void APHT();

  //vector<ConvexPlane>& getPlanes();
  bool distanceOK(Point p1, Point p2, Point p3);
  bool calculatePlane(Point p1, Point p2, Point p3, double &theta, double &phi, double &rho); 

  double * const* deletePoints(vector<ConvexPlane*> &model, int &size); 
  double * const* getPoints(int &size);
  int deletePoints(double * n, double rho);
  int deletePointsQuad(double * n, double rho);

  int writePlanes(int startCount);
  void writePlanes(std::string);
  int cluster(vPtPair &pairs, double minx, double max, double miny, double maxy);
  void writePlanePoints(std::string);
  void writeAllPoints(int index, vector<Point> points);

};

double calcPlane(vector<Point> &ppoint, double plane[4]);

#endif
