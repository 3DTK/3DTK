/** @file
 *  @brief The 6D Lu Milios style SLAM in 6D
 *
 *  @author Dorit Borrman. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Jan Elseberg. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 *  @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __LUM6D_EULER_L_H__
#define __LUM6D_EULER_L_H__

#include <vector>
using std::vector;

#include "slam6d/graphSlam6D.h"
#include "graphSlam6DL.h"
#include "linescan.h"
#include "srr/lsegment.h"

#include "newmat/newmatio.h"

typedef vector <PtPair> vPtPair;  ///< just a typedef: vPtPair = vector of type PtPair


/*
 * @brief Representation of 3D scan matching with Lu/Milios in 6D.
 *
 * Reference: D. Borrmann, et al., Globally consistent 3D mapping
 *            with scan matching, Journal of Robotics and Autonomous
 *            Systems (2007),
 */
class lum6DEulerL : public graphSlam6DL {

public:
  /**
   * Constructor (default)
   */
  lum6DEulerL() {};
  lum6DEulerL(icp6Dminimizer *my_icp6Dminimizer,
		   double mdm = 25.0,
		   double max_dist_match = 25.0,
		   int max_num_iterations = 50,
		   bool quiet = false,
		   bool meta = false,
		   int rnd = 1,
		   bool eP = true,
		   int anim = -1,
		   double epsilonICP = 0.0000001,
		   bool use_cache = false,
		   double epsilonLUM = 0.5);

  virtual ~lum6DEulerL();

  double doGraphSlam6D(Graph gr, vector <LScan*> MetaScan, int nrIt);
  double doGraphSlam6D(Graph gr, vector <Scan*> MetaScan, int nrIt) {return 0.0;}
  double doGraphSlam6D(Graph gr, vector <LScan*> MetaScan, int nrIt, vector<LineScan*> allScans, vector<Scan*> *_allSScans);
  double doGraphSlam6D(Graph gr, vector <LScan*> MetaScan, int nrIt,
      vector<LineScan*> allScans, vector<Scan*> *_allSScans, vector<LSegment*> _allSegments);


  // ptsource == 0  closest point pairs
  // ptsource == 1  odometry
  // ptsource == 2  ground_truth
  // ptsource == 3  rigid matching
  static int covarianceEuler(LScan *first, LScan *second, double &SUM,
      int rnd, double max_dist_match2, double odoweight,
      NEWMAT::Matrix *C, NEWMAT::ColumnVector *CD=0, int ptsource = 0);

  static void covarianceEuler(Scan *first, Scan *second, const double *fmat,
         const double *smat, int rnd, double max_dist_match2,
         NEWMAT::Matrix *C, NEWMAT::ColumnVector *CD=0);

  static void toTransMat(NEWMAT::Matrix *C, NEWMAT::ColumnVector *CD, double *transmat);


private:
  void FillGB3D(Graph *gr, GraphMatrix* G, NEWMAT::ColumnVector* B, vector <LScan *> allScans);
  void FillGB3D(Graph *gr, GraphMatrix* G, NEWMAT::ColumnVector* B, vector <LScan *> allLScans, vector<LineScan*> allScans);

  vector<Scan*> *allSScans;
  std::vector<LSegment*> allSegments;

  // scantype == 0 Scan
  // scantype == 1 LScan
  static void _covarianceEuler(vector<PtPair> &uk, int scantype, double &SUM, double odoweight,
        NEWMAT::Matrix *C, NEWMAT::ColumnVector *CD=0, double ptsource = 0);

  void _fillGB(GraphMatrix* G, ColumnVector* B, Matrix& Cab, ColumnVector& CDab, int a, int b);
  static NEWMAT::ColumnVector _calcPoseUpdate(const double* rPos, const double* rPosTheta, ColumnVector& Xtmp);

};

#endif
