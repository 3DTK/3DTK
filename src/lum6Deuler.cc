/**
 * @file 
 * @brief The implementation of globally consistent scan matching algorithm
 * @author Dorit Borrman. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Jan Elseberg. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "lum6Deuler.h"
#include "sparse/csparse.h"

#include <cfloat>
#include <fstream>
using std::ofstream;
#include "globals.icc"

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
lum6DEuler::lum6DEuler(icp6Dminimizer *my_icp6Dminimizer,
				   double mdm, double max_dist_match, double max_dist_match_last,
				   int max_num_iterations, bool quiet, bool meta, int rnd,
				   bool eP, int anim, double epsilonICP, bool use_cache, double epsilonLUM)
  : graphSlam6D(my_icp6Dminimizer,
			 mdm, max_dist_match, max_dist_match_last,
			 max_num_iterations, quiet, meta, rnd,
			 eP, anim, epsilonICP, use_cache, epsilonLUM)
{ }


/**
 * Destructor
 */
lum6DEuler::~lum6DEuler()
{
  delete my_icp;
}


/**
 * This function calculates the inverse covariances Cij and the Vector Cij*Dij for
 * two scans by finding pointpairs.
 * 
 * @param first pointer to the first scan of the link
 * @param second pointer to the second scan of the link
 * @param use_cache shall we use the cache?
 * @param rnd shall we use randomization for computing the point pairs?
 * @param max_dist_match2 maximal distance allowed for point pairs
 * @param C pointer to the inverse of the covariance matrix Cij
 * @param CD pointer to the vector Cij*Dij
 */
void lum6DEuler::covarianceEuler(Scan *first, Scan *second, 
                                 bool use_cache, int rnd, double max_dist_match2, 
                                 Matrix *C, ColumnVector *CD) 
{
  // x,y,z       denote the coordinates of uk (Here averaged over ak and bk)
  // sx,sy,sz    are the sums of their respective coordinates of uk over each paint pair
  // xpy,xpz,ypz are the sums over x*x + y*y ,x*x + z*z and y*y + z*z respectively over each point pair
  // xy,yz,xz    are the sums over each respective multiplication 
  // dx,dy,dz    are the deltas in each coordinate of a point pair
  // ss          is the estimation of the covariance of sensing error
  double x, y, z, sx, sy, sz, xy, yz, xz, ypz, xpz, xpy, dx, dy, dz, ss;

  // D is needed to calculate the estimation of the covariance s
  ColumnVector D(6);
  // Almost Cij*Dij
  ColumnVector MZ(6);
  // Almost the covarianve
  Matrix MM(6,6);
  // A set of point pairs
  vector <PtPair> uk;
  // A point pair
  Point ak, bk;
  // number of pairs in a set
  int m;

#ifdef _OPENMP
  int thread_num = omp_get_thread_num();
#else
  int thread_num = 0;
#endif

  double dummy_centroid_m[3];
  double dummy_centroid_d[3];

  if (use_cache) {
    KDCacheItem *closest = Scan::initCache(first, second);
    Scan::getPtPairsCache(&uk, closest, first, second, thread_num, rnd, max_dist_match2, dummy_centroid_m, dummy_centroid_d);
  } else {
    Scan::getPtPairs(&uk, first, second, thread_num, rnd, max_dist_match2, dummy_centroid_m, dummy_centroid_d);
  }

  m = uk.size();

  MZ = 0.0;
  MM = 0.0;
  sx = sy = sz = xy = yz = xz = ypz = xpz = xpy = ss = 0.0;

  if (m > 0) {
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
      MZ(6) += z * dx - x * dz;
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
	 ss = 1.0 / ss;

	 if (CD) {
	   *CD = MZ * ss;
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
    cerr << "Error calculating covariance matrix" << endl;

  }
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
void lum6DEuler::FillGB3D(Graph *gr, Matrix* G, ColumnVector* B,vector<Scan *> allScans )
{
#ifdef _OPENMP
#pragma omp parallel for
#endif
  for(int i = 0; i < gr->getNrLinks(); i++){
    int a = gr->getLink(i,0) - 1;
    int b = gr->getLink(i,1) - 1;
    Scan *FirstScan  = allScans[gr->getLink(i,0)];
    Scan *SecondScan = allScans[gr->getLink(i,1)];
  
    //    cout << "i " << i << " a: " << a << " b: " << b << endl; 

    Matrix Cab;
    ColumnVector CDab;
    covarianceEuler(FirstScan, SecondScan, use_cache, (int)my_icp->get_rnd(), 
                    (int)max_dist_match2_LUM, &Cab, &CDab); 

    if(a >= 0){
      B->Rows(a*6+1,a*6+6) += CDab;
      G->SubMatrix(a*6+1,a*6+6,a*6+1,a*6+6) += Cab;
    }
    if(b >= 0){
      B->Rows(b*6+1,b*6+6) -= CDab;
      G->SubMatrix(b*6+1,b*6+6,b*6+1,b*6+6) += Cab;
    }
    if(a >= 0 && b >= 0) { 
      G->SubMatrix(a*6+1,a*6+6,b*6+1,b*6+6) = -Cab;
      G->SubMatrix(b*6+1,b*6+6,a*6+1,a*6+6) = -Cab;
    }
  }
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
double lum6DEuler::doGraphSlam6D(Graph gr, vector <Scan *> allScans, int nrIt)
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
    Matrix G(6*n,6*n);
    ColumnVector B(6*n);
    G = 0.0;
    B = 0.0;
    // ...fill G and B...
    FillGB3D(&gr, &G, &B, allScans);
    // ...and solve it
    ColumnVector X =  solveSparseCholesky(G, B);

    //cout << "X done!" << endl;

    double sum_position_diff = 0.0;
    
    // Start with second Scan
    int loop_end = gr.getNrScans();
#ifdef _OPENMP
#pragma omp parallel for reduction(+:sum_position_diff)
#endif
    for(int i = 1; i < loop_end; i++){
	 
      // Now update the Poses
      Matrix Ha = IdentityMatrix(6);
      
      double xa = allScans[i]->get_rPos()[0];
      double ya = allScans[i]->get_rPos()[1];
      double za = allScans[i]->get_rPos()[2];

      double tx = allScans[i]->get_rPosTheta()[0];
      double ty = allScans[i]->get_rPosTheta()[1];

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

      // Get pose estimate
      ColumnVector Xtmp = X.Rows((i-1)*6+1,(i-1)*6+6);

      // Correct pose estimate
      ColumnVector result = Ha * Xtmp;

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
        cout <<  "x: " << allScans[i]->get_rPos()[0]
       << " y: " << allScans[i]->get_rPos()[1]
       << " z: " << allScans[i]->get_rPos()[2]
       << " tx: " << allScans[i]->get_rPosTheta()[0]
       << " ty: " << allScans[i]->get_rPosTheta()[1]
       << " tz: " << allScans[i]->get_rPosTheta()[2] << endl << endl;
      }

      double x[3];
      x[0] = result.element(0);
      x[1] = result.element(1);
      x[2] = result.element(2);
      sum_position_diff += Len(x);
    }
    cout << "Sum of Position differenzes = " << sum_position_diff << endl;
    ret = (sum_position_diff / (double)gr.getNrScans());
  }
  
  return ret;
}


