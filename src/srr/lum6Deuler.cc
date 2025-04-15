/**
 * @file
 * @brief The implementation of globally consistent scan matching algorithm
 *
 * @author Dorit Borrman. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Jan Elseberg. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 *
 * A description of the algorithms implemented here can be found in the following paper
 * (ras2007.pdf):
 *
 * Dorit Borrmann, Jan Elseberg, Kai Lingemann, Andreas Nuechter, and Joachim Hertzberg.
 * Globally consistent 3D mapping with scan matching. Journal of Robotics and Autonomous
 * Systems (JRAS), Elsevier Science, Volume 56, Issue 2, ISSN 0921-8890, pages 130 - 142,
 * February 2008
 */

#ifdef _MSC_VER
#ifdef OPENMP
#define _OPENMP
#endif
#endif

#include "lum6Deuler.h"
#include "slam6d/lum6Deuler.h"
#include "suitesparse/cs.h"

#include <cfloat>
#include <fstream>
using std::ofstream;
using std::cerr;
#include "slam6d/globals.icc"

using namespace NEWMAT;
/**
 * Constructor
 *
 * @param my_icp6Dminimizer Pointer to ICP minimization functor
 * @param mdm Maximum PtoP distance to which point pairs are collected for ICP
 * @param max_dist_match Maximum PtoP distance to which point pairs are collected for LUM
 * @param max_num_iterations Maximal number of iterations for ICP
 * @param quiet Suspesses all output to std out
 * @param meta Indicates if metascan matching has to be used
 * @param rnd Indicates if randomization has to be used
 * @param eP Extrapolate odometry?
 * @param anim Animate which frames?
 * @param epsilonICP Termination criterion for ICP
 * @param use_cache Shall we used cached k-d tree search
 * @param epsilonLUM Termination criterion for LUM
 */
lum6DEulerL::lum6DEulerL(icp6Dminimizer *my_icp6Dminimizer,
				   double mdm, double max_dist_match,
				   int max_num_iterations, bool quiet, bool meta, int rnd,
				   bool eP, int anim, double epsilonICP, bool use_cache, double epsilonLUM)
  : graphSlam6DL(my_icp6Dminimizer,
			 mdm, max_dist_match,
			 max_num_iterations, quiet, meta, rnd,
			 eP, anim, epsilonICP, use_cache, epsilonLUM)
{ }


/**
 * Destructor
 */
lum6DEulerL::~lum6DEulerL()
{
  delete my_icp;
}


/**
 * A function to fill the linear system G X = B.
 *
 * @param gr the Graph is used to map the given covariances C and CD matrices to the correct link
 * @param CD A vector containing all covariances C multiplied with their respective estimations D
 * @param C A vector containing all covariances C of the pose difference estimations D
 * @param G The matrix G specifying the linear equation
 * @param B The vector B
 */
void lum6DEulerL::FillGB3D(Graph *gr, GraphMatrix* G, ColumnVector* B,vector<LScan *> allScans )
{
    double SUM = 0.0;
  for(int i = 0; i < gr->getNrLinks(); i++){
    int a = gr->getLink(i,0) - 1;
    int b = gr->getLink(i,1) - 1;
    LScan *FirstScan  = allScans[gr->getLink(i,0)];
    LScan *SecondScan = allScans[gr->getLink(i,1)];

    Matrix Cab(6,6);
    ColumnVector CDab(6);
    double S = 0.0;
    covarianceEuler(FirstScan, SecondScan, S, (int)my_icp->get_rnd(),
                    (int)max_dist_match2_LUM, odomweight, &Cab, &CDab);
    SUM += S;

#pragma omp critical
    {
      _fillGB(G,B,Cab,CDab,a,b);
    }
  }
  cout << "total sum of point differences: " << SUM << endl;
}


void lum6DEulerL::FillGB3D(Graph *gr, GraphMatrix* G, ColumnVector* B,vector<LScan *> allLScans, vector<LineScan*> allScans)
{
  double SUM = 0.0;
  for(int i = 0; i < gr->getNrLinks(); i++){
    LScan *FirstScan  = allLScans[gr->getLink(i,0)];
    LScan *SecondScan = allLScans[gr->getLink(i,1)];
    int a = FirstScan->getRepresentative() - 1;
    int b = SecondScan->getRepresentative() - 1;

    Matrix Cab(6,6);
    ColumnVector CDab(6);
    double S = 0.0;
  covarianceEuler( (*allSScans)[gr->getLink(i,0)], (*allSScans)[gr->getLink(i,1)], allScans[a+1]->transMat, allScans[b+1]->transMat,
      (int)my_icp->get_rnd(), (int)max_dist_match2_LUM, &Cab, &CDab);
    SUM += S;

#pragma omp critical
    {
      _fillGB(G,B,Cab,CDab,a,b);
    }
  }

  // Odometry
  if(allSegments.size() <= 1) {
    std::cout << "odom covariance in no segment mode" << std::endl;
    for(unsigned int i = 1; i < allScans.size(); i++) {
      int a = i - 2;
      int b = i - 1;
      LScan FirstScan(i-1,i-1, i-1);
      LScan SecondScan(i,i, i);

      Matrix Cab(6,6);
      ColumnVector CDab(6);
      double S = 0.0;
      covarianceEuler(&FirstScan, &SecondScan, S, (int)my_icp->get_rnd(),
          (int)max_dist_match2_LUM, odomweight,
          &Cab, &CDab, 1);
      SUM += S;

#pragma omp critical
      {
        _fillGB(G,B,Cab,CDab,a,b);
      }
    }
  } else {
    std::cout << "odom covariance in segment mode" << std::endl;
    for(size_t j= 0; j < allSegments.size(); j++) {
      int earliest = allSegments[j]->getBegin();
      int latest   = allSegments[j]->getEnd();
      for(int i = earliest; i <= latest; i++) {
        int a = i - 2;
        int b = i - 1;
        LScan FirstScan(i-1,i-1, i-1);
        LScan SecondScan(i,i, i);

        Matrix Cab(6,6);
        ColumnVector CDab(6);
        double S = 0.0;
        covarianceEuler(&FirstScan, &SecondScan, S, (int)my_icp->get_rnd(),
            (int)max_dist_match2_LUM, odomweight,
            &Cab, &CDab, 1);
        SUM += S;

#pragma omp critical
        {
          _fillGB(G,B,Cab,CDab,a,b);
        }
      }
      if(gapodomweight > 0) {
        for(size_t j=1; j < allSegments.size(); j++) { // handle segment borders
          int a = allSegments[j-1]->getEnd() - 1;
          int b = allSegments[j]->getBegin() - 1;
          LScan FirstScan(allSegments[j-1]->getEnd(),allSegments[j-1]->getEnd(),allSegments[j-1]->getEnd());
          LScan SecondScan(allSegments[j]->getBegin(),allSegments[j]->getBegin(),allSegments[j]->getBegin());

          Matrix Cab(6,6);
          ColumnVector CDab(6);
          double S = 0.0;
          covarianceEuler(&FirstScan, &SecondScan, S, (int)my_icp->get_rnd(),
              (int)max_dist_match2_LUM, gapodomweight,
              &Cab, &CDab, 1);
          SUM += S;

#pragma omp critical
          {
            _fillGB(G,B,Cab,CDab,a,b);
          }
        }
      }
    }

  }

  cout << "total sum of point differences: " << SUM << endl;
}

/**
 * This function is used to match a set of laser scans with any minimally
 * connected Graph, using the globally consistent LUM-algorithm in 3D.
 *
 * @param gr Some Graph with no real subgraphs except for itself
 * @param allScans Contains all laser scans
 * @param nrIt The number of iterations the LUM-algorithm will run
 * @return Euclidian distance of all pose shifts
 */
double lum6DEulerL::doGraphSlam6D(Graph gr, vector <LScan *> allScans, int nrIt)
{
#ifdef WRITE_GRAPH_NET
  // for debug only:
  static int d = 0;
  cout << "writing graph.dat ....................................." << endl;
  d++;
  string gfilename = "graph_" + to_string(d, 3) + ".net";
  ofstream out(gfilename.c_str());
  for (int i=0; i < gr.getNrLinks(); i++) {
    int from = gr.getLink(i,0);
    int to = gr.getLink(i,1);
    // shouldn't be necessary, just in case a (out of date) graph file is loaded:
    if (from < (int)allScans.size() && to < (int)allScans.size()) {
	 out << allScans[from]->get_rPos()[0] << " "
	     << allScans[from]->get_rPos()[1] << " "
	     << allScans[from]->get_rPos()[2] << endl
	     << allScans[to  ]->get_rPos()[0] << " "
	     << allScans[to  ]->get_rPos()[1] << " "
	     << allScans[to  ]->get_rPos()[2] << endl << endl;
    }
  }
  out.close();
  out.clear();
#endif

  // the IdentityMatrix to transform some Scans with
  double id[16];
  M4identity(id);

  double ret = DBL_MAX;

  for(int iteration = 0;
	 iteration < nrIt && ret > epsilonLUM;
	 iteration++) {

   if (nrIt > 1) cout << "Iteration " << iteration << endl;


    // * Calculate X and CX from all Dij and Cij
    int n = (gr.getNrScans() - 1);

    // Construct the linear equation system..
    GraphMatrix *G = new GraphMatrix();
    ColumnVector B(6*n);
    B = 0.0;
    // ...fill G and B...
    FillGB3D(&gr, G, &B, allScans);
    // ...and solve it
    ColumnVector X =  solveSparseCholesky(G, B);

    delete G;

    double sum_position_diff = 0.0;

    // Start with second Scan
    int loop_end = gr.getNrScans();
#ifdef _OPENMP
#pragma omp parallel for reduction(+:sum_position_diff)
#endif
    for(int i = 1; i < loop_end; i++){

      // Now update the Poses
      // Get pose estimate
      ColumnVector Xtmp = X.Rows((i-1)*6+1,(i-1)*6+6);

      // Correct pose estimate
      ColumnVector result = _calcPoseUpdate(allScans[i]->get_rPos(),allScans[i]->get_rPosTheta(),Xtmp);

      if(!quiet) {
        cout << "Old pose estimate, Scan " << i << endl;
        cout <<  "x: " << allScans[i]->get_rPos()[0]
       << " y: " << allScans[i]->get_rPos()[1]
       << " z: " << allScans[i]->get_rPos()[2]
       << " tx: " << allScans[i]->get_rPosTheta()[0]
       << " ty: " << allScans[i]->get_rPosTheta()[1]
       << " tz: " << allScans[i]->get_rPosTheta()[2]
       << endl;
      }

      double rPos[3];
      double rPosTheta[3];

      // calculate the updated Pose
      for (int k = 0; k < 3; k++) {
        rPos[k]      = allScans[i]->get_rPos()[k] - result.element(k);
        rPosTheta[k] = allScans[i]->get_rPosTheta()[k] - result.element(k+3);
      }

      // Update the Pose
      if (i != gr.getNrScans() - 1) {
        allScans[i]->transformToEuler(rPos, rPosTheta, Scan::LUM, 1);
      } else {
        allScans[i]->transformToEuler(rPos, rPosTheta, Scan::LUM, 2);
      }

      if(!quiet) {
        cout <<  "x: " << rPos[0]
       << " y: " <<  rPos[1]
       << " z: " << rPos[2]
       << " tx: " << rPosTheta[0]
       << " ty: " << rPosTheta[1]
       << " tz: " << rPosTheta[2] << endl << endl;
      }

      double x[3];
      x[0] = result.element(0);
      x[1] = result.element(1);
      x[2] = result.element(2);
      sum_position_diff += Len(x);
    }
    cout << "Sum of Position differences = " << sum_position_diff << endl;
    ret = (sum_position_diff / (double)gr.getNrScans());
  }

  return ret;
}

double lum6DEulerL::doGraphSlam6D(Graph gr, vector <LScan*> MetaScan, int nrIt,
    vector<LineScan*> allScans, vector<Scan*> *_allSScans,
    vector<LSegment*> _allSegments) {
  allSegments = _allSegments;
  return doGraphSlam6D(gr,MetaScan,nrIt,allScans,_allSScans);
};

double lum6DEulerL::doGraphSlam6D(Graph gr, vector <LScan *> allLScans, int nrIt, vector<LineScan*> allScans, vector<Scan*> *_allSScans)
{
  allSScans = _allSScans;

  // the IdentityMatrix to transform some Scans with
  double id[16];
  M4identity(id);

  double ret = DBL_MAX;

  for(int iteration = 0;
	 iteration < nrIt && ret > epsilonLUM;
	 iteration++) {

   if (nrIt > 1) cout << "Iteration " << iteration << endl;


    // * Calculate X and CX from all Dij and Cij
    //int n = (gr.getNrScans() - 1);
    int n = (allScans.size() - 1);

    // Construct the linear equation system..
    GraphMatrix *G = new GraphMatrix();
    ColumnVector B(6*n);
    B = 0.0;
    // ...fill G and B...
    FillGB3D(&gr, G, &B, allLScans, allScans);
    /*
    cout << "Filled matrix... solving it now"  << endl;
    cout << "B:" << endl << B << endl;
    cout << "G:" << endl << G << endl;
    */
    // ...and solve it
    ColumnVector X =  solveSparseCholesky(G, B);
    delete G;

/*    cout << "X done!" << endl;
    cout << X << endl;*/

    double sum_position_diff = 0.0;

    // Start with second Scan
    int loop_end = allScans.size();
#ifdef _OPENMP
#pragma omp parallel for reduction(+:sum_position_diff)
#endif
    for(int i = 1; i < loop_end; i++){

      // Now update the Poses
      // Get pose estimate
      ColumnVector Xtmp = X.Rows((i-1)*6+1,(i-1)*6+6);

      // Correct pose estimate
      ColumnVector result = _calcPoseUpdate(allScans[i]->rPos,allScans[i]->rPosTheta,Xtmp);

      if(!quiet) {
        cout << "Old pose estimate, Scan " << i << endl;
        cout <<  "x: " << allScans[i]->rPos[0]
       << " y: " << allScans[i]->rPos[1]
       << " z: " << allScans[i]->rPos[2]
       << " tx: " << allScans[i]->rPosTheta[0]
       << " ty: " << allScans[i]->rPosTheta[1]
       << " tz: " << allScans[i]->rPosTheta[2]
       << endl;
      }

      double rPos[3];
      double rPosTheta[3];

      // calculate the updated Pose
      for (int k = 0; k < 3; k++) {
        rPos[k]      = allScans[i]->rPos[k] - result.element(k);
        rPosTheta[k] = allScans[i]->rPosTheta[k] - result.element(k+3);
//        rPosTheta[k] = allScans[i]->rPosTheta[k];
      }

      // Update the Pose
      if (i != (int)(allScans.size() - 1)) {
        allScans[i]->transformToEuler(rPos, rPosTheta, Scan::LUM, 1);
      } else {
        allScans[i]->transformToEuler(rPos, rPosTheta, Scan::LUM, 2);
      }

      if(!quiet) {
        cout <<  "x: " << rPos[0]
       << " y: " <<  rPos[1]
       << " z: " << rPos[2]
       << " tx: " << rPosTheta[0]
       << " ty: " << rPosTheta[1]
       << " tz: " << rPosTheta[2] << endl << endl;
        /*
        cout <<  "x: " << allLScans[i]->get_rPos()[0]
       << " y: " << allLScans[i]->get_rPos()[1]
       << " z: " << allLScans[i]->get_rPos()[2]
       << " tx: " << allLScans[i]->get_rPosTheta()[0]
       << " ty: " << allLScans[i]->get_rPosTheta()[1]
       << " tz: " << allLScans[i]->get_rPosTheta()[2] << endl << endl;
       */
      }

      double x[3];
      x[0] = result.element(0);
      x[1] = result.element(1);
      x[2] = result.element(2);
      sum_position_diff += Len(x);
    }
    cout << "Sum of Position differences = " << sum_position_diff << endl;
    ret = (sum_position_diff / (double)gr.getNrScans());
  }

  return ret;
}



void lum6DEulerL::toTransMat(NEWMAT::Matrix *C, NEWMAT::ColumnVector *CD, double *transmat) {
  //Matrix Ha = IdentityMatrix(6);
  double rPos[3], rPosTheta[3];
  Matrix4ToEuler(transmat, rPosTheta, rPos);

  Matrix Ci = C->i();
  ColumnVector D = Ci * *CD;
  ColumnVector result = _calcPoseUpdate(rPos,rPosTheta,D);
//  cout << "C " << *C << endl;
//  cout << "result: " << result << endl;

  for (int k = 0; k < 3; k++) {
    rPos[k]      += result.element(k);
    rPosTheta[k] += result.element(k+3);
  }
  cout << rPos[0] << " " << rPos[1] << " " << rPos[2] << "  " << deg(rPosTheta[0]) << " " << deg(rPosTheta[1]) << " " << deg(rPosTheta[2]) << endl;


}


/**
 * This function calculates the inverse covariances Cij and the Vector Cij*Dij for
 * two LScans by finding pointpairs.
 *
 * @param first pointer to the first LScan of the link
 * @param second pointer to the second LScan of the link
 * @param SUM of point differences
 * @param rnd shall we use randomization for computing the point pairs?
 * @param max_dist_match2 maximal distance allowed for point pairs
 * @param odoweight weight for influence of odometry
 * @param C pointer to the inverse of the covariance matrix Cij
 * @param CD pointer to the vector Cij*Dij
 * @param ptsource which method to use for finding pointpairs, as well as structure of LScan (0 closest_point pairs, 1 odometry, 2 ground_truth, 3 rigid matching)
 */
int lum6DEulerL::covarianceEuler(LScan *first, LScan *second, double &SUM,
        int rnd, double max_dist_match2, double odoweight,
        Matrix *C, ColumnVector *CD, int ptsource)
{

  // A set of point pairs
  vector <PtPair> uk;

#ifdef _OPENMP
  int thread_num = omp_get_thread_num();
#else
  int thread_num = 0;
#endif

  double dummy_centroid_m[3];
  double dummy_centroid_d[3];

  if (ptsource==0) {
    LScan::getPtPairs(&uk, first, second, thread_num, rnd,
            max_dist_match2, dummy_centroid_m, dummy_centroid_d);
  } else if (ptsource == 1) {
    LScan::getOdomPairs(&uk, first, second, dummy_centroid_m, dummy_centroid_d, false);
  } else if (ptsource == 2) {
    LScan::getOdomPairs(&uk, first, second, dummy_centroid_m, dummy_centroid_d, true);
  } else if (ptsource == 3) {
  }

  _covarianceEuler(uk,1,SUM,odoweight,C,CD,ptsource);
  return uk.size();

}

/**
 * This function calculates the inverse covariances Cij and the Vector Cij*Dij for
 * two scans by finding pointpairs.
 *
 * @param first pointer to the first scan of the link
 * @param second pointer to the second scan of the link
 * @param fmat current transformation matrix of first scan
 * @param smat current transformation matrix of second scan
 * @param rnd shall we use randomization for computing the point pairs?
 * @param max_dist_match2 maximal distance allowed for point pairs
 * @param C pointer to the inverse of the covariance matrix Cij
 * @param CD pointer to the vector Cij*Dij
 */
void lum6DEulerL::covarianceEuler(Scan *first, Scan *second,
        const double *fmat, const double *smat,
        int rnd, double max_dist_match2,
        Matrix *C, ColumnVector *CD)
{

  // A set of point pairs
  vector <PtPair> uk;

#ifdef _OPENMP
  int thread_num = omp_get_thread_num();
#else
  int thread_num = 0;
#endif

  double dummy_centroid_m[3];
  double dummy_centroid_d[3];
  double dummy_sum;

	Scan::getPtPairs(&uk, first, second, thread_num,
				   rnd, max_dist_match2, dummy_sum, dummy_centroid_m, dummy_centroid_d);

  double finv[16], sinv[16];
  double ftmat[16], stmat[16];
  M4inv(first->get_transMat(), finv);
  M4inv(second->get_transMat(), sinv);

  MMult(fmat, finv, ftmat);
  MMult(smat, sinv, stmat);

  for (unsigned int j = 0; j < uk.size(); j++) {
      uk[j].p1.transform(ftmat);
      uk[j].p2.transform(stmat);
  }

  double SUM = 0.0;

  _covarianceEuler(uk,0,SUM,0.0,C,CD);
}


/**
 * This function calculates the inverse covariances Cij and the Vector Cij*Dij for
 * two scans on pointpairs.
 *
 * @param uk list of point pairs
 * @param scantype 0 for Scan, 1 for LScan
 * @param SUM
 * @param C pointer to the inverse of the covariance matrix Cij
 * @param CD pointer to the vector Cij*Dij
 */
void lum6DEulerL::_covarianceEuler(vector<PtPair> &uk, int scantype, double &SUM,
        double odoweight, Matrix *C, ColumnVector *CD, double ptsource)
{
  // x,y,z       denote the coordinates of uk (Here averaged over ak and bk)
  // sx,sy,sz    are the sums of their respective coordinates of uk over each paint pair
  // xpy,xpz,ypz are the sums over x*x + y*y ,x*x + z*z and y*y + z*z respectively over each point pair
  // xy,yz,xz    are the sums over each respective multiplication
  // dx,dy,dz    are the deltas in each coordinate of a point pair
  // ss          is the estimation of the covariance of sensing error
  double x, y, z, sx, sy, sz, xy, yz, xz, ypz, xpz, xpy, dx, dy, dz, ss;
  sx = sy = sz = xy = yz = xz = ypz = xpz = xpy = ss = 0.0;

  // D is needed to calculate the estimation of the covariance s
  ColumnVector D(6);
  // Almost Cij*Dij
  ColumnVector MZ(6);
  MZ = 0.0;
  // Almost the covarianve
  Matrix MM(6,6);
  MM = 0.0;
  // A point pair
  Point ak, bk;
  // number of pairs in a set
  const int m = uk.size();

  SUM=0.0;
  if (m > 2) {
    // for each point pair
    for(int j = 0; j < m; j++){
      ak = uk[j].p1;
      bk = uk[j].p2;

      // Some temporary values
      x = (ak.x + bk.x)/2.0;
      y = (ak.y + bk.y)/2.0;
      z = (ak.z + bk.z)/2.0;
      dx = ak.x - bk.x;
      dy = ak.y - bk.y;
      dz = ak.z - bk.z;

      SUM += sqrt( sqr(dx) + sqr(dy) + sqr(dz));

      // Sum up all necessary values to construct MM
      sx += x;
      sy += y;
      sz += z;

      xpy += x*x + y*y;
      xpz += x*x + z*z;
      ypz += y*y + z*z;

      xy += x*y;
      xz += x*z;
      yz += y*z;

      // Sum up each part of MZ
      MZ(1) += dx;
      MZ(2) += dy;
      MZ(3) += dz;
      MZ(4) += -z * dy + y * dz;
      MZ(5) += -y * dx + x * dy;
      MZ(6) +=  z * dx - x * dz;
    }
    // Now construct the symmetrical matrix MM
    MM(1,1) = MM(2,2) = MM(3,3) = m;
    MM(4,4) = ypz;
    MM(5,5) = xpy;
    MM(6,6) = xpz;

    MM(1,5) = MM(5,1) = -sy;
    MM(1,6) = MM(6,1) = sz;
    MM(2,4) = MM(4,2) = -sz;
    MM(2,5) = MM(5,2) = sx;
    MM(3,4) = MM(4,3) = sy;
    MM(3,6) = MM(6,3) = -sx;

    MM(4,5) = MM(5,4) = -xz;
    MM(4,6) = MM(6,4) = -xy;
    MM(5,6) = MM(6,5) = -yz;

    // Calculate the pose difference estimation
    D = MM.i() * MZ ;

    // Again going through all point pairs to faster calculate s.
    // This cannot be done earlier as we need D, and therefore MM and MZ to do this
    for(int j = 0; j < m; j++){
      ak = uk[j].p1;
      bk = uk[j].p2;

      x = (ak.x + bk.x) / 2.0;
      y = (ak.y + bk.y) / 2.0;
      z = (ak.z + bk.z) / 2.0;

      ss += sqr(ak.x - bk.x - (D(1) - y * D(5) + z * D(6)))
        + sqr(ak.y - bk.y - (D(2) - z * D(4) + x * D(5)))
        + sqr(ak.z - bk.z - (D(3) + y * D(4) - x * D(6)));
    }

    ss =  ss / (2*m - 3);

    // for dealing with numerical instabilities when identical point clouds are used in matching
    if (ss  < 0.0000000000001) {
      if(scantype==1) { // LScan
        // odometry case
        ss = odoweight;
        // ss = 400.0;
      } else { // scantype==0 (Scan)
        ss = 0.0;
        MM(1,1) = MM(1,2) = MM(1,3) = 0.0;
        MM(2,1) = MM(2,2) = MM(2,3) = 0.0;
        MM(3,1) = MM(3,2) = MM(3,3) = 0.0;
        MZ(6) = MZ(1) = MZ(2) = 0.0;
        MZ(3) = MZ(4) = MZ(5) = 0.0;
        *C = 0;
        if(CD)
          *CD = 0;
        return;
      }
    }

    ss = 1.0 / ss;

    if (CD) {
      *CD = MZ * ss;
    }

    // need to set a covariance weight, otherwise the evidence would be too strong
    if (ptsource != 0 && ptsource != 3 && scantype==1) {
      ss = odoweight;
    }

    *C = MM * ss;

  } else {
    // This case should not occur
    ss = 0.0;
    MM(1,1) = MM(1,2) = MM(1,3) = 0.0;
    MM(2,1) = MM(2,2) = MM(2,3) = 0.0;
    MM(3,1) = MM(3,2) = MM(3,3) = 0.0;
    MZ(6) = MZ(1) = MZ(2) = 0.0;
    MZ(3) = MZ(4) = MZ(5) = 0.0;
    *C = 0;
    if(CD)
      *CD = 0;
    cerr << "Error calculating covariance matrix from " << m  << " point pairs" << endl;
  }
}

void lum6DEulerL::_fillGB(GraphMatrix* G, ColumnVector* B,
    Matrix& Cab, ColumnVector& CDab, int a, int b) {

  if(a >= 0){
    B->Rows(a*6+1,a*6+6) += CDab;
    G->add(a, a, Cab);
  }
  if(b >= 0){
    B->Rows(b*6+1,b*6+6) -= CDab;
    G->add(b, b, Cab);
  }
  if(a >= 0 && b >= 0) {
    G->subtract(a, b, Cab);
    G->subtract(b, a, Cab);
  }

}

/**
 * @brief calculate the Pose update
 * @param rPos scan position
 * @param rPosTheta scan orientation
 * @param Xtmp pose estimate corresponding to xa,ya,za,tx,ty
 */
ColumnVector lum6DEulerL::_calcPoseUpdate(const double* rPos, const double* rPosTheta,
    ColumnVector& Xtmp) {

  // Now update the Poses
  Matrix Ha = IdentityMatrix(6);

  double xa = rPos[0];
  double ya = rPos[1];
  double za = rPos[2];

  double tx = rPosTheta[0];
  double ty = rPosTheta[1];

  double ctx = cos(tx);
  double stx = sin(tx);
  double cty = cos(ty);
  double sty = sin(ty);

  // Fill Ha
  Ha.element(0,4) = -za*ctx+ya*stx;
  Ha.element(0,5) = ya*cty*ctx+za*stx*cty;

  Ha.element(1,3) = za;
  Ha.element(1,4) = -xa*stx;
  Ha.element(1,5) = -xa*ctx*cty+za*sty;

  Ha.element(2,3) = -ya;
  Ha.element(2,4) = xa*ctx;
  Ha.element(2,5) = -xa*cty*stx-ya*sty;

  Ha.element(3,5) = sty;

  Ha.element(4,4) = stx;
  Ha.element(4,5) = ctx*cty;

  Ha.element(5,4) = ctx;
  Ha.element(5,5) = -stx*cty;
  // Invert it
  Ha = Ha.i();

  return Ha * Xtmp;

}
