/**
 * @file HOG-Man wrapper
 * @author Jochen Sprickerhof. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __GRAPH_HOGMAN_H__
#define __GRAPH_HOGMAN_H__

#include "graphSlam6D.h"

class graphHOGMan : public graphSlam6D {

public:
  /**
   * Constructor (default)
   */
  graphHOGMan() {};
  /**
   * Constructor
   *
   * @param my_icp6Dminimizer Pointer to ICP minimization functor
   * @param mdm Maximum PtoP distance to which point pairs are collected for ICP
   * @param max_dist_match Maximum PtoP distance to which point pairs are collected for LUM
   * @param max_num_iterations Maximal number of iterations for ICP
   * @param quiet Suspesses all output to std out
   * @param meta Indicates if metascan matching has to be used
   * @param rnd Indicates if randomization has to be used
   * @param eP Extrapolate odometry?
   * @param anim Animate which frames?
   * @param epsilonICP Termination criterion for ICP
   * @param nns_method Which nearest neigbor search method shall we use
   * @param epsilonLUM Termination criterion for LUM
   */
  graphHOGMan(icp6Dminimizer *my_icp6Dminimizer,
      double mdm = 25.0,
      double max_dist_match = 25.0,
      int max_num_iterations = 50,
      bool quiet = false,
      bool meta = false,
      int rnd = 1,
      bool eP = true,
      int anim = -1,
      double epsilonICP = 0.0000001,
      int nns_method = simpleKD,
      double epsilonLUM = 0.5)
    : graphSlam6D(my_icp6Dminimizer,
        mdm, max_dist_match,
        max_num_iterations, quiet, meta, rnd,
        eP, anim, epsilonICP, nns_method, epsilonLUM)
    { }

  virtual ~graphHOGMan()
  {
    delete my_icp;
  }

  double doGraphSlam6D(Graph gr, vector <Scan*> MetaScan, int nrIt);

};

#endif
