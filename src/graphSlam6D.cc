/**
 * @file 
 * @brief The implementation of globally consistent scan matching algorithm
 * @author Dorit Borrman. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Jan Elseberg. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "graphSlam6D.h"
#include "sparse/csparse.h"

#include <cfloat>
#include <fstream>
using std::ofstream;
using std::flush;
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
graphSlam6D::graphSlam6D(icp6Dminimizer *my_icp6Dminimizer,
					double mdm, double max_dist_match, double max_dist_match_last,
					int max_num_iterations, bool quiet, bool meta, int rnd,
					bool eP, int anim, double epsilonICP, bool use_cache, double epsilonLUM)
{
  this->use_cache = use_cache;
  this->quiet = quiet;
  this->epsilonLUM = epsilonLUM;
  this->max_dist_match2_LUM = sqr(max_dist_match);
  if (max_dist_match_last > 0) {
    this->max_dist_match2_last_LUM = sqr(max_dist_match_last);
  } else {
    this->max_dist_match2_last_LUM = -1;
  }
  this->my_icp = new icp6D(my_icp6Dminimizer, mdm, max_num_iterations,
					  quiet, meta, rnd, eP, anim, epsilonICP, use_cache);
}


/**
 * This function is used to match a set of laser scans with any minimally
 * connected Graph, using the globally consistent LUM-algorithm in 3D.
 *
 * @param allScans Contains all laser scans
 * @param nrIt The number of iterations the LUM-algorithm will run
 * @param clpairs minimal number of points aximal distance for closing loops
 * @param loopsize minimal loop size
 */
void graphSlam6D::matchGraph6Dautomatic(vector <Scan *> allScans, int nrIt, int clpairs, int loopsize)
{
  // the IdentityMatrix to transform some Scans with
  double id[16];
  M4identity(id);

  Graph *gr = 0;
  int i = 0;

  do {
    cout << "Generate graph ... " << flush;
    i++;
    if (gr) delete gr;
    gr = new Graph(0, false);
    int j, maxj = (int)allScans.size();
#ifdef _OPENMP
    omp_set_num_threads(OPENMP_NUM_THREADS);
#pragma omp parallel for schedule(dynamic)
#endif
    for (j = 0; j <  maxj; j++) {
#ifdef _OPENMP
	 int thread_num = omp_get_thread_num();
#else
	 int thread_num = 0;
#endif
	 for (int k = 0; k < (int)allScans.size(); k++) {
	   if (j == k) continue;
	   Scan * FirstScan  = allScans[j];
	   Scan * SecondScan = allScans[k];
	   double centroid_d[3] = {0.0, 0.0, 0.0};
	   double centroid_m[3] = {0.0, 0.0, 0.0};
	   vPtPair temp;
	   Scan::getPtPairs(&temp, FirstScan, SecondScan, thread_num,
					my_icp->get_rnd(), (int)max_dist_match2_LUM,
					centroid_m, centroid_d);
	   if ((int)temp.size() > clpairs) {
#ifdef _OPENMP
#pragma omp critical
#endif
		gr->addLink(j, k);  
	   } 
	 }
    }
    cout << "done" << endl;
  } while ((doGraphSlam6D(*gr, allScans, 1) > 0.001) && (i < nrIt));

  return;
}

/**
 * This function is used to solve the system of linear eq.
 *
 * @param G symmetric, positive definite Matrix, thus invertable
 * @param B column vector
 */
void graphSlam6D::writeMatrixPGM(const Matrix &G)
{
  int n = G.Ncols();
  static int matrixnum = 0;
  string mf = "matrix" + to_string(matrixnum,4) + ".pgm";
  ofstream matrixout(mf.c_str());
  matrixout << "P2" << endl
		  << "# CREATOR slam6D (c) Andreas Nuechter, 05/2007" << endl
		  << n << " " << n << endl
		  << 255 << endl;
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
	  if (G.element(i, j) > 0.001) {
	    matrixout << 0 << " ";
	  } else {
	    matrixout << 255 << " ";
	  }
    }
    matrixout << endl;
  }
  // matrixout << G << endl; 
  matrixout.close();
  matrixout.clear();
  matrixnum++;

}



/**
 * This function is used to solve the system of linear eq.
 *
 * @param G Matrix, invertable
 * @param B column vector
 */
ColumnVector graphSlam6D::solve(const Matrix &G, const ColumnVector &B)
{
  
#ifdef WRITE_MATRIX_PGM
  writeMatrixPGM(G);
#endif

  // ----------------------------------
  // solve eqn via inverting the matrix
  // ----------------------------------
  return  (ColumnVector)(G.i() * B);
}


/**
 * This function is used to solve the system of linear eq.
 * The implementation from the numerical recepies are used
 *
 * @param G symmetric, positive definite Matrix, thus invertable
 * @param B column vector
 */
ColumnVector graphSlam6D::solveCholesky(const Matrix &G, const ColumnVector &B)
{
  
#ifdef WRITE_MATRIX_PGM
  writeMatrixPGM(G);
#endif

  // We copy the newmat matrices and use our own
  // Cholesky decomposition code here. The Cholesky
  // decomosition is based on the Numerical Recipes in C.
  // This speed ups computation time

  // copy values
  int n = G.Ncols();
  double **A   = new double*[n];
  double *C    = new double[n];
  double *diag = new double[n];
  double *x    = new double[n];
  ColumnVector X(n);
  for (int i = 0; i < n; i++) {
    A[i] = new double[n];
    for (int j = 0; j < n; j++) {
	 A[i][j] = G.element(i, j);
    }
    C[i] = B.element(i);
  }
  // --------------------------------------------------
  // make cholesky dekomposition with numerical recipes
  // --------------------------------------------------
  if (!choldc(n, A, diag)) {
    cout << "cannot perfom cholesky decomposition" << endl;     
  }
  // solve A x = C
  cholsl(n, A, diag, C, x);
  // copy values back
  for (int i = 0; i < n; i++) {
    X.element(i) = x[i];
  }
  // clean up
  for (int i = 0; i < n; i++) {
    delete [] A[i];
  }
  delete [] x;
  delete [] diag;
  delete [] C;  
  delete [] A;

  return X;
}


/**
 * This function is used to solve the system of linear eq.
 *
 * @param G symmetric, positive definite Matrix, thus invertable
 * @param B column vector
 */
ColumnVector graphSlam6D::solveSparseCholesky(const Matrix &G, const ColumnVector &B)
{
  
#ifdef WRITE_MATRIX_PGM
  writeMatrixPGM(G);
#endif

  int n = G.Ncols();
  
  // ------------------------------
  // Sparse Cholsekey decomposition
  // ------------------------------
  ColumnVector X(n);
  cs *A, *T = cs_spalloc (0, 0, 1, 1, 1) ;
  double *x = new double[n];
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
	 if (fabs(G.element(i, j)) > 0.00001) {
	   cs_entry (T, i, j, G.element(i, j));
	 }
    }
    x[i] = B.element(i);
  }
  A = cs_triplet (T);
  cs_dropzeros (A) ;			// drop zero entries
  cs_cholsol (A, x, 1) ;
  // copy values back  
  for (int i = 0; i < n; i++) {
    X.element(i) = x[i];
  }

  cs_spfree(A);
  cs_spfree(T);
  delete [] x;

  return X;
}


/**
 * This function is used to solve the system of linear eq.
 *
 * @param G invertable Matrix
 * @param B column vector
 */
ColumnVector graphSlam6D::solveSparseQR(const Matrix &G, const ColumnVector &B)
{
  
#ifdef WRITE_MATRIX_PGM
  writeMatrixPGM(G);
#endif

  int n = B.Ncols();

  ColumnVector X(n);
  cs *A, *T = cs_spalloc (0, 0, 1, 1, 1) ;
  double *x = new double[n];
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < n; j++) {
	 if (fabs(G.element(i, j)) > 0.00001) {
	   cs_entry (T, i, j, G.element(i, j));
	 }
    }
    x[i] = B.element(i);
  }
  A = cs_triplet (T);
  cs_dropzeros (A) ;			// drop zero entries
  int order = 3;                   // for qr-ordering
  cs_qrsol ( A, x, order) ;
  // copy values back  
  for (int i = 0; i < n; i++) {
    X.element(i) = x[i];
  }

  cs_spfree(A);
  cs_spfree(T);
  delete [] x;

  return X;
}

void graphSlam6D::set_mdmll(double mdmll) {
  max_dist_match2_LUM = sqr(mdmll);
}

