/** @file 
 *  @brief The global correction based upon 6D Lu Milios style SLAM in 6D but with a helix correction step
 *
 *  @author Peter Schneider. Institute of Computer Science, University of Koblenz , Germany.
 */

#ifndef __GHELIX_Q2_H__
#define __GHELIX_Q2_H__

#include <vector>
using std::vector;

#include "graphSlam6D.h"
#include "icp6D.h"
#include "graph.h"

#include "newmat/newmatio.h"

//typedef vector <PtPair> vPtPair;  ///< just a typedef: vPtPair = vector of type PtPair


/*
 * @brief Representation of 3D scan matching based upon Lu/Milios in 6D but using helix correction step.
 *
 *            H. Pottman, et al., Simultaneous registration of multiple
 *            views of a 3D object, 
 */
class ghelix6DQ2 : public graphSlam6D {

public:
  /**
   * Constructor (default)
   */
  ghelix6DQ2() {};
  ghelix6DQ2(icp6Dminimizer *my_icp6Dminimizer,
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

  virtual ~ghelix6DQ2();
  
  virtual double doGraphSlam6D(Graph gr, vector <Scan*> MetaScan, int nrIt);
  
  static double LUM[4];
  
private:

  double genBBdForLinkedPair( int firstScanNum, int secondScanNum, vPtPair *ptpairs,
						NEWMAT::Matrix *B, NEWMAT::ColumnVector *Bd);
};

#endif
