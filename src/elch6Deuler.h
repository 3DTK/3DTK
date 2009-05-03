/**
 * @file ELCH implementation using Euler angles
 * @author Jochen Sprickerhof
 */

#ifndef __ELCH6D_EULER_H__
#define __ELCH6D_EULER_H__

#include "elch6D.h"

class elch6Deuler : public elch6D {

  public:
    elch6Deuler(bool _quiet, icp6Dminimizer *my_icp6Dminimizer, double mdm, int max_num_iterations, int rnd, bool eP, int anim, double epsilonICP, bool use_cache)
     : elch6D(_quiet, my_icp6Dminimizer, mdm, max_num_iterations, rnd, eP, anim, epsilonICP, use_cache) {}

    virtual void close_loop(const vector <Scan *> &allScans, int first, int last, graph_t &g);
    
};

#endif
