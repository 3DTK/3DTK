/** @file 
 *  @brief The 6D Lu Milios style SLAM in 6D
 *
 *  @author Dorit Borrman. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Jan Elseberg. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __LUM6D_QUAT_H__
#define __LUM6D_QUAT_H__

#include <vector>
using std::vector;

#include "graphSlam6D.h"

#include "newmat/newmatio.h"

typedef vector <PtPair> vPtPair;  ///< just a typedef: vPtPair = vector of type PtPair


/*
 * @brief Representation of 3D scan matching with Lu/Milios in 6D.
 *
 * Reference: D. Borrmann, et al., Globally consistent 3D mapping 
 *            with scan matching, Journal of Robotics and Autonomous 
 *            Systems (2007),
 */
class lum6DQuat : public graphSlam6D {

public:
  /**
   * Constructor (default)
   */
  lum6DQuat() {};
  lum6DQuat(icp6Dminimizer *my_icp6Dminimizer,
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
		  double epsilonLUM = 0.5);

  virtual ~lum6DQuat();
  
  double doGraphSlam6D(Graph gr, vector <Scan*> MetaScan, int nrIt);
  static void covarianceQuat(Scan *first, Scan *second,  int nns_method,
					    int rnd, double max_dist_match2, NEWMAT::Matrix *C, NEWMAT::ColumnVector *CD=0);
  
private:
  void FillGB3D(Graph *gr, NEWMAT::Matrix* G, NEWMAT::ColumnVector* B, vector<Scan*> allScans);
    
};

#endif
