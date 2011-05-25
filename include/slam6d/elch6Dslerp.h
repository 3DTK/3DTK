/**
 * @file ELCH implementation using SLERP
 * @author Jochen Sprickerhof. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __ELCH6D_SLERP_H__
#define __ELCH6D_SLERP_H__

#include "elch6D.h"

class elch6Dslerp : public elch6D {

  public:
    elch6Dslerp(bool _quiet, icp6Dminimizer *my_icp6Dminimizer, double mdm, int max_num_iterations,
			 int rnd, bool eP, int anim, double epsilonICP, int nns_method)
	 : elch6D(_quiet, my_icp6Dminimizer, mdm, max_num_iterations, rnd, eP, anim, epsilonICP, nns_method) {}

    virtual void close_loop(const vector <Scan *> &allScans, int first, int last, graph_t &g);
    
};

#endif
