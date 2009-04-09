/** @file 
 *  @brief Implementation of the virtual functor for a loop closing algorithm
 *  @author Jochen Sprickerhof
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __LOOPSLAM6D_H__
#define __LOOPSLAM6D_H__

#include <vector>
using std::vector;

#include "icp6D.h"
#include "icp6Dminimizer.h"
#include "scan.h"
#include "graph.h"

class loopSlam6D {

public:
  /** 
   * Constructor 
   */
  loopSlam6D(bool _quiet, icp6Dminimizer *my_icp6Dminimizer, double mdm, int max_num_iterations, int rnd, bool eP, int anim, double epsilonICP, bool use_cache)
  {
    this->quiet = _quiet;
    this->my_icp6D = new icp6D(my_icp6Dminimizer, mdm, max_num_iterations,
                    quiet, false, rnd, eP, anim, epsilonICP, use_cache);
  };

  /** 
   * Destructor 
   */
  virtual ~loopSlam6D() {
    delete my_icp6D;
  };

  virtual void close_loop(const vector <Scan *> &allScans, int first, int last, graph_t &g) = 0;

protected:
  bool quiet;

  /**
   * pointer to the ICP framework
   */
  icp6D *my_icp6D;

};

#endif 
