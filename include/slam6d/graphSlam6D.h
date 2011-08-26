/** @file 
 *  @brief Implementation of the virtual functor for graphslam
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __GRAPHSLAM_6D_H__
#define __GRAPHSLAM_6D_H__

#include <vector>
using std::vector;
#include <map>
using std::map;
/*#include <pair>
using std::pair;*/

#include "icp6D.h"
#include "graph.h"
#include "newmat/newmatio.h"
#include "sparse/csparse.h"

typedef vector <PtPair> vPtPair;  ///< just a typedef: vPtPair = vector of type PtPair
using namespace NEWMAT;
typedef pair<unsigned int, unsigned int> uipair;
typedef pair< uipair, Matrix* > uimpair;


class GraphMatrix {
  public:
    void add(const unsigned int i, const unsigned int j, Matrix &Cij);
    void subtract(const unsigned int i, const unsigned int j, Matrix &Cij);
    void print() ;
    void convertToCS(cs* T);

    ~GraphMatrix();


  private:
  map< uipair, Matrix* > matrix;
  map< uipair, Matrix* >::iterator it;
};

class graphSlam6D {

public:
  /** 
   * Constructor 
   */
  graphSlam6D() { };

  graphSlam6D(icp6Dminimizer *my_icp6Dminimizer,
		    double mdm, double max_dist_match, 
		    int max_num_iterations, bool quiet, bool meta, int rnd,
		    bool eP, int anim, double epsilonICP, int nns_method, double epsilonLUM);

  /** 
   * Destructor 
   */
  virtual ~graphSlam6D();

  virtual double doGraphSlam6D(Graph gr, vector <Scan*> MetaScan, int nrIt) = 0;
  
  void matchGraph6Dautomatic(vector <Scan*> MetaScan, int nrIt, double cldist, int loopsize);
  void matchGraph6Dautomatic(vector <Scan*> MetaScan, int nrIt, int clpairs, int loopsize);
  Graph *computeGraph6Dautomatic(vector <Scan *> allScans, int clpairs); 

  NEWMAT::ColumnVector solveSparseCholesky(GraphMatrix *G, const NEWMAT::ColumnVector &B);
  NEWMAT::ColumnVector solveSparseCholesky(const NEWMAT::Matrix &G, const NEWMAT::ColumnVector &B);
  NEWMAT::ColumnVector solveSparseQR(const NEWMAT::Matrix &G, const NEWMAT::ColumnVector &B);
  NEWMAT::ColumnVector solveCholesky(const NEWMAT::Matrix &G, const NEWMAT::ColumnVector &B);
  NEWMAT::ColumnVector solve(const NEWMAT::Matrix &G, const NEWMAT::ColumnVector &B);

  void writeMatrixPGM(const NEWMAT::Matrix &G);
  void set_mdmll(double mdmll);
  inline void set_quiet(bool _quiet) { quiet = _quiet;};
  
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
   * indicates the NNS method being used
   */
  int nns_method;

  /**
   * be quiet
   */
  bool quiet;


  long ctime;
};

#endif 
