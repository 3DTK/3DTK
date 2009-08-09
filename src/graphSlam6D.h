/** @file 
 *  @brief Implementation of the virtual functor for graphslam
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __GRAPHSLAM_6D_H__
#define __GRAPHSLAM_6D_H__

#include <vector>
using std::vector;

#include "icp6D.h"
#include "graph.h"
#include "newmat/newmatio.h"

typedef vector <PtPair> vPtPair;  ///< just a typedef: vPtPair = vector of type PtPair

class graphSlam6D {

public:
  /** 
   * Constructor 
   */
  graphSlam6D() { };

  graphSlam6D(icp6Dminimizer *my_icp6Dminimizer,
		    double mdm, double max_dist_match, double max_dist_match_last,
		    int max_num_iterations, bool quiet, bool meta, int rnd,
		    bool eP, int anim, double epsilonICP, bool use_cache, double epsilonLUM);

  /** 
   * Destructor 
   */
  virtual ~graphSlam6D() {};

  virtual double doGraphSlam6D(Graph gr, vector <Scan*> MetaScan, int nrIt) = 0;
  
  void matchGraph6Dautomatic(vector <Scan*> MetaScan, int nrIt, double cldist, int loopsize);
  void matchGraph6Dautomatic(vector <Scan*> MetaScan, int nrIt, int clpairs, int loopsize);

  ColumnVector solveSparseCholesky(const Matrix &G, const ColumnVector &B);
  ColumnVector solveSparseQR(const Matrix &G, const ColumnVector &B);
  ColumnVector solveCholesky(const Matrix &G, const ColumnVector &B);
  ColumnVector solve(const Matrix &G, const ColumnVector &B);

  void writeMatrixPGM(const Matrix &G);
  void set_mdmll(double mdmll);
  
protected:
  /**
   * pointer to the ICP framework
   */
  icp6D *my_icp;

  /**
   * the epsilon for LUM
   */
  double epsilonLUM;
  
  /**
   * the maximal distance (^2 !!!) for matching in LUM
   */
  double max_dist_match2_LUM;

  /**
   * the maximal distance (^2 !!!) for matching in LUM
   */
  double max_dist_match2_last_LUM;

  /**
   * indicates using the cache
   */
  bool use_cache;

  /**
   * be quiet
   */
  bool quiet;
};

#endif 
