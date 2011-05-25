/**
 * @file HOG-Man wrapper
 * @author Jochen Sprickerhof. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __LOOP_HOG_MAN_H__
#define __LOOP_HOG_MAN_H__

#include "loopSlam6D.h"

class loopHOGMan : public loopSlam6D {

  public:
    loopHOGMan(bool _quiet, icp6Dminimizer *my_icp6Dminimizer, double mdm, int max_num_iterations,
			int rnd, bool eP, int anim, double epsilonICP, int nns_method)
     : loopSlam6D(_quiet, my_icp6Dminimizer, mdm, max_num_iterations, rnd, eP, anim, epsilonICP, nns_method) {}

    virtual void close_loop(const vector <Scan *> &allScans, int first, int last, graph_t &g);
};

#endif
