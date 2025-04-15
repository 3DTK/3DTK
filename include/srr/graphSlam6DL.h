#ifndef __GRAPHSLAML_H__
#define __GRAPHSLAML_H__

#include "slam6d/graphSlam6D.h"
#include "linescan.h"
#include "srr/lsegment.h"

class graphSlam6DL : public graphSlam6D {
  public:
  graphSlam6DL() : graphSlam6D() { };

  graphSlam6DL(icp6Dminimizer *my_icp6Dminimizer,
		    double mdm, double max_dist_match,
		    int max_num_iterations, bool quiet, bool meta, int rnd,
		    bool eP, int anim, double epsilonICP, bool use_cache, double epsilonLUM)
    : graphSlam6D(my_icp6Dminimizer, mdm, max_dist_match, max_num_iterations, quiet, meta, rnd,
        eP, anim, epsilonICP, use_cache, epsilonLUM) {
      odomweight = 1000000000.0;  // this is good
    };


  virtual double doGraphSlam6D(Graph gr, vector <LScan*> MetaScan, int nrIt) = 0;
  virtual double doGraphSlam6D(Graph gr, vector <LScan*> MetaScan, int nrIt, vector<LineScan*> allScans, vector<Scan*> *allSScans = 0) {return 0.0;};
  virtual double doGraphSlam6D(Graph gr, vector <LScan*> MetaScan, int nrIt, vector<LineScan*> allScans, vector<Scan*> *allSScans, vector<LSegment*> allSegments) {return 0.0;};

  void setOdomWeight(double weight, double gapweight = 0.0) {
    odomweight = weight;
    gapodomweight = gapweight;
  }

protected:
  double odomweight;
  double gapodomweight;


};

#endif
