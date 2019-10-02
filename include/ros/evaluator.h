#ifndef __EVALUATOR_H__
#define __EVALUATOR_H__


#include "slam6d/scan.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6D.h"
#include "slam6d/globals.icc"
#include "shapes/geom_math.h"
#include <omp.h>
#include <string>

#undef VERSION

class evaluator {

  public:

    virtual double evaluate(vector<Scan *> &scans) = 0;

};


class PPevaluator : public evaluator {
  double mdist;
  bool withICP;
  icp6Dminimizer *my_icp6Dminimizer;

protected:
  icp6D *icp;
  void doICP(vector<Scan *> &scans) {
    icp->doICP(scans); // align scans to minimize odometry related errors
  }

public:
  PPevaluator(double _mdist, bool _withICP = false) : mdist(_mdist), withICP(_withICP) {
    my_icp6Dminimizer = new icp6D_QUAT(true);
    icp = new icp6D(my_icp6Dminimizer);
  }

  double evaluate(vector<Scan *> &scans) {
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < (int)scans.size(); i++) {
      scans[i]->setSearchTreeParameter(BOCTree);
      scans[i]->createSearchTree();
    }
    double result = 0;

    if (icp) {
      if (withICP)
        doICP(scans);
      for (unsigned int i = 0; i < scans.size(); i++) {
        for (unsigned int j = 0; j < scans.size(); j++) {
          if (j == i) continue;
          unsigned int nrp2;
          double err2 = icp->Point_Point_Error((scans)[j], (scans)[i], mdist, &nrp2);//, 0.01);
          result += err2;
        }
      }
      /*
      for (unsigned int i = 1; i < scans.size(); i++) {
        unsigned int nrp2;
        double err2 = icp->Point_Point_Error((scans)[i-1], (scans)[i], mdist, &nrp2, 0.01);
        result += err2;
      }*/

    }
    return result;
  }
};


class PPevaluatorN : public evaluator {
  double mdist;
  bool withICP;
  icp6Dminimizer *my_icp6Dminimizer;

protected:
  icp6D *icp;

  void doICP(vector<Scan *> &scans) {
    icp->doICP(scans); // align scans to minimize odometry related errors
  }

public:
  PPevaluatorN(double _mdist, bool _withICP = false) : mdist(_mdist), withICP(_withICP) {
    my_icp6Dminimizer = new icp6D_QUAT(true);
    icp = new icp6D(my_icp6Dminimizer);
  }

  double evaluate(vector<Scan *> &scans) {
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < (int)scans.size(); i++) {
      scans[i]->setSearchTreeParameter(BOCTree);
      scans[i]->createSearchTree();
    }
    double result = 0;

    if (icp) {
      if (withICP)
        doICP(scans);

      for (unsigned int i = 1; i < scans.size(); i++) {
        unsigned int nrp2;
        icp->Point_Point_Error((scans)[i-1], (scans)[i], mdist, &nrp2);
        result += nrp2;
      }

    }
    return result * -(0.000001);   // use for number of points as error
  }
};

class PPevaluatorG : public evaluator {
  double mdist;
  bool withICP;
  icp6Dminimizer *my_icp6Dminimizer;

protected:
  icp6D *icp;

  void doICP(vector<Scan *> &scans) {
    icp->doICP(scans); // align scans to minimize odometry related errors
  }

public:
  PPevaluatorG(double _mdist, bool _withICP = false) : mdist(_mdist), withICP(_withICP) {
    my_icp6Dminimizer = new icp6D_QUAT(true);
    icp = new icp6D(my_icp6Dminimizer);
  }

  double evaluate(vector<Scan *> &scans) {
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < (int)scans.size(); i++) {
      scans[i]->setSearchTreeParameter(BOCTree);
      scans[i]->createSearchTree();
    }
    double result = 0;

    if (icp) {
      if (withICP)
        doICP(scans);

      unsigned int n = scans[0]->size<DataXYZ>("xyz reduced");
      unsigned int m = scans[1]->size<DataXYZ>("xyz reduced");
      cout << n << " " << m << "   ! ! ! ! ! " << endl;
      double sums[OPENMP_NUM_THREADS];
      for (unsigned int i=0; i < OPENMP_NUM_THREADS; i++)
        sums[i] = 0.0;
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
      for (unsigned int i = 0; i < n; i++) {
        unsigned int tn = omp_get_thread_num();
        for (unsigned int j = 0; j < m; j++) {
          DataXYZ ps = scans[0]->get("xyz reduced");
          DataXYZ qs = scans[1]->get("xyz reduced");
          const double*p = ps[i];
          const double*q = qs[j];
          double dist = Dist2(p,q);

          sums[tn] += 0.39894228 * exp(- dist/200 ); // decimeters work better than centimeters
        }
      }
      for (unsigned int i=0; i < OPENMP_NUM_THREADS; i++)
        result -= sums[i];

    }
    return result * 0.0001;        // use for n^2
  }
};


class Planeevaluator : public evaluator {

  vector<vector<int> > *plane2points;
  int nrscans;

//  vector<double *> planes;

public:
  Planeevaluator(string directory, int _nrscans, int start = 0, int end=0) : nrscans(_nrscans)  {
    plane2points = new vector<vector<int> >[nrscans];
    for (int i = 0; i < nrscans; i++) {
      for (int fileCounter = start; fileCounter <= end; fileCounter++) {
        plane2points[i].push_back( vector<int>() );
      }
    }
    // TODO read in information from the directory
    string scanFileName;

    ifstream scan_in;

    for (int fileCounter = start; fileCounter <= end; fileCounter++) {
      scanFileName = directory + "scan3d/selected" + to_string(fileCounter,3) + ".3d";
      cout << "opening file " << scanFileName << endl;

      scan_in.open(scanFileName.c_str());
      if (!scan_in.good()) continue;

      double x,y,z,r;
      while (scan_in.good()) {
        int index;
        int scanindex;
        scan_in >> x >> y >> z >> r;
        scan_in >> index;
        scan_in >> scanindex;
        plane2points[scanindex][fileCounter-start].push_back(index);
      }
      scan_in.close();
      scan_in.clear();
    }
  }

  ~Planeevaluator() { delete[]  plane2points;}

  double evaluate(vector<Scan *> &scans) {
    double error = 0;
    double planeerr;
    vector<double *> points;
    double plane[4];
    double center[3];

    if (nrscans <= 0 || (unsigned int)nrscans != scans.size()) return DBL_MAX;

    for (unsigned int j = 0; j < plane2points[0].size(); j++) {
      for (unsigned int i = 0; i < scans.size(); i++) {
        DataXYZ pts = scans[i]->get("xyz reduced");
        //      cout << "pts: " << scans[i]->get_points_red_size() << endl;

        planeerr = 0.0;
        points.clear();
        for (unsigned int k = 0; k < plane2points[i][j].size(); k++) {
          if (plane2points[i][j][k] < scans[i]->size<DataXYZ>("xyz reduced")) {
            points.push_back(pts[plane2points[i][j][k]]);
//            cout << pts[plane2points[i][j][k]][0] << " " << pts[plane2points[i][j][k]][1] << " " <<pts[i][plane2points[j][k]][2] << " 0 0" << endl;
          }
        }
//        exit(0);
        if(!fitPlane(points, plane, center)) {
          cout << "couldnt fit plane nr " << j << endl;
          continue;
        }
        for (unsigned int k = 0; k < points.size(); k++) {
          planeerr += fabs(planeDist(points[k], plane[0], plane[1], plane[2], plane[3]));
        }

//        error += planeerr;
	error += planeerr/scans[i]->size<DataXYZ>("xyz reduced");
      }

    }

    return error;
  }
};

#endif
