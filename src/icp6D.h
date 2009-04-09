/** @file 
 *  @brief Representation of 3D scan matching with ICP
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __ICP6D_H__
#define __ICP6D_H__

#include <vector>
using std::vector;
#include <ctime>

#include "newmat/newmat.h"

#include "ptpair.h"
#include "scan.h"
#include "icp6Dminimizer.h"

#include "globals.icc"

/**
 * @brief Representation of 3D scan matching with ICP.
 *  
 * Manages the matching of 3D scans.
 * Important values, such as maximal matching distance, 
 * maximal number of iterations, etc.
 * are specified in the constructor.
 */
class icp6D {

public:
  icp6D(icp6Dminimizer *my_icp6Dminimizer,
	   double max_dist_match = 25.0,
	   int max_num_iterations = 50,
	   bool quiet = false,
	   bool meta = false,
	   int rnd = 1,
	   bool eP = true,
	   int anim = -1,
	   double epsilonICP = 0.0000001,
	   bool use_cache = false);
  
  /**
   * Destructor (empty, but needed, because virtual)
   */
  virtual ~icp6D() {};
  
  void doICP(vector <Scan *> allScans);
  virtual int match(Scan* PreviousScan, Scan* CurrentScan);
  void covarianceEuler(Scan *scan1, Scan *scan2, Matrix *C);
  void covarianceQuat(Scan *scan1, Scan *scan2, Matrix *C);

  inline int  get_rnd();
  inline bool get_meta();
  inline int  get_anim();
  inline bool get_use_cache();
  inline void set_anim(int anim);
  inline double get_max_dist_match2();
  inline void set_max_dist_match2(double max_dist_match2);
  inline void set_max_num_iterations(int max_num_iterations);
  
private:

  /**
   * suppress output to cout
   */
  bool quiet;

  /**
   * take every rnd point for matching
   */
  int rnd;

  /**
   * extrapolate odometry
   */
  bool eP;

  /**
   * match against all scans (= meta scan), or against the last scan only
   */
  bool   meta;

  /**
   * specifies if cached kd-tree search shall be used
   */
  bool use_cache;

  /**
   * the maximal distance (^2 !!!) for matching
   */
  double max_dist_match2;

  /**
   * the maximal number of iterations
   */
  int    max_num_iterations;

  /**
   * write anim'th animation frame
   */
  int anim;

  /**
   * epsilon for stopping ICP algorithm ( convergence criterium )
   */
  double epsilonICP;

  /**
   * ptr to ICP error function minimizer functor
   */
  icp6Dminimizer *my_icp6Dminimizer;
};

#include "icp6D.icc"

#endif
