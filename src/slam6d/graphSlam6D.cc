/*
 * graphSlam6D implementation
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */


/**
 * @file 
 * @brief The implementation of globally consistent scan matching algorithm
 * @author Dorit Borrman. Inst. of CS, University of Osnabrueck, Germany.
 * @author Jan Elseberg. Inst. of CS, University of Osnabrueck, Germany.
 * @author Kai Lingemann. Inst. of CS, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Inst. of CS, University of Osnabrueck, Germany.
 */

#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#include "slam6d/graphSlam6D.h"
#include "sparse/csparse.h"

#include <cfloat>
#include <fstream>
using std::ofstream;
using std::flush;
#include "slam6d/globals.icc"

using namespace NEWMAT;
/**
 * Constructor
 *
 * @param my_icp6Dminimizer Pointer to ICP minimization functor
 * @param mdm Maximum PtoP distance to which point pairs are collected for ICP
 * @param max_dist_match Maximum PtoP distance to which point pairs are
                         collected for LUM
 * @param max_num_iterations Maximal number of iterations for ICP
 * @param quiet Suspesses all output to std out
 * @param meta Indicates if metascan matching has to be used
 * @param rnd Indicates if randomization has to be used
 * @param eP Extrapolate odometry?
 * @param anim Animate which frames?
 * @param epsilonICP Termination criterion for ICP
 * @param nns_method Specifies which NNS method to use
 * @param epsilonLUM Termination criterion for LUM
 */
graphSlam6D::graphSlam6D(icp6Dminimizer *my_icp6Dminimizer,
                         double mdm, double max_dist_match, 
                         int max_num_iterations, bool quiet, bool meta, int rnd,
                         bool eP, int anim, double epsilonICP, int nns_method, double epsilonLUM)
{
  this->nns_method = nns_method;
  this->quiet = quiet;
  this->epsilonLUM = epsilonLUM;
  this->max_dist_match2_LUM = sqr(max_dist_match);

  ctime = 0;

  this->my_icp = new icp6D(my_icp6Dminimizer, mdm, max_num_iterations,
                           quiet, meta, rnd, eP, anim, epsilonICP, nns_method);
}

graphSlam6D::~graphSlam6D()
 {
   cout << "Time spent in the SLAM backend:" << ctime << endl;
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
void graphSlam6D::matchGraph6Dautomatic(vector <Scan *> allScans,
                                        int nrIt,
                                        int clpairs,
                                        int loopsize)
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
        double sum_dummy;
        Scan::getPtPairs(&temp, FirstScan, SecondScan, thread_num,
            my_icp->get_rnd(), max_dist_match2_LUM, sum_dummy,
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


Graph *graphSlam6D::computeGraph6Dautomatic(vector <Scan *> allScans,
                                            int clpairs) 
{
  // the IdentityMatrix to transform some Scans with
  double id[16];
  M4identity(id);

  int i = 0;

  cout << "Generate graph ... " << flush;
  i++;
  Graph *gr = new Graph(0, false);
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
      double sum_dummy;
      Scan::getPtPairs(&temp, FirstScan, SecondScan, thread_num,
          my_icp->get_rnd(), max_dist_match2_LUM, sum_dummy,
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

  return gr;
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
ColumnVector graphSlam6D::solveSparseCholesky(const Matrix &G,
                                              const ColumnVector &B)
{

  long starttime = GetCurrentTimeInMilliSec();
    
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
  cs_dropzeros (A) ;               // drop zero entries
  cs_cholsol (A, x, 1) ;
  // copy values back  
  for (int i = 0; i < n; i++) {
    X.element(i) = x[i];
  }

  cs_spfree(A);
  cs_spfree(T);
  delete [] x;

  ctime += GetCurrentTimeInMilliSec() - starttime;

  return X;
}

ColumnVector graphSlam6D::solveSparseCholesky(GraphMatrix *G,
                                              const ColumnVector &B)
{

  long starttime = GetCurrentTimeInMilliSec();

  int n = B.Nrows();
  ColumnVector X(n);
  
  // ------------------------------
  // Sparse Cholsekey decomposition
  // ------------------------------
  cs *A, *T = cs_spalloc (0, 0, 1, 1, 1) ;
  double *x = new double[n];
  for (int i = 0; i < n; i++) {
    x[i] = B.element(i);
  }
  G->convertToCS(T);
  A = cs_triplet (T);
  cs_dropzeros (A) ;               // drop zero entries
//  cs_print(T, 0);
  cs_cholsol (A, x, 1) ;
  // copy values back  
  for (int i = 0; i < n; i++) {
    X.element(i) = x[i];
  }

  cs_spfree(A);
  cs_spfree(T);
  delete [] x;

  ctime += GetCurrentTimeInMilliSec() - starttime;

  return X;
}


/**
 * This function is used to solve the system of linear eq.
 *
 * @param G invertable Matrix
 * @param B column vector
 */
ColumnVector graphSlam6D::solveSparseQR(const Matrix &G,
                                        const ColumnVector &B)
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
  cs_dropzeros (A) ;               // drop zero entries
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


void GraphMatrix::add(const unsigned int i,
                      const unsigned int j,
                      Matrix &Cij)
{
  uipair ui(i,j);
  it = matrix.find( ui );
  if (it != matrix.end()) {
    (*(it->second)) += Cij;
  } else {
    Matrix *C = new Matrix(6,6);
    *C = Cij;
    matrix.insert( uimpair( ui, C));
  }
}

void GraphMatrix::subtract(const unsigned int i,
                           const  unsigned int j,
                           Matrix &Cij) {
  uipair ui(i,j);
  it = matrix.find( ui );
  if (it != matrix.end()) {
    (*it->second) -= Cij;
  } else {
    Matrix *C = new Matrix(6,6);
    *C = Cij;
    *C *= -1.0;
    matrix.insert( uimpair( ui, C));
  }
}

void GraphMatrix::print() {
  for ( it = matrix.begin() ; it != matrix.end(); it++ ) {
    uimpair uim = *it;
    uipair ui = uim.first;
    cout << ui.first << " " << ui.second << " :"
         << endl << *uim.second << endl;
  }
}

GraphMatrix::~GraphMatrix() {
  for ( it = matrix.begin() ; it != matrix.end(); it++ ) {
    uimpair uim = *it;
    delete uim.second;
  }

}

void GraphMatrix::convertToCS(cs *T) {
  unsigned int a,b;
  int imin,imax,jmin,jmax;

  for ( it = matrix.begin() ; it != matrix.end(); it++ ) {
    Matrix *C = it->second;
    a = it->first.first;
    b = it->first.second;
    imin = a*6;
    jmin = b*6;

    imax = a*6 + 6;
    jmax = b*6 + 6;
    a = b = 0;

    for (int i = imin; i < imax; i++, a++) {
      b = 0;
      for (int j = jmin; j < jmax; j++, b++) {
        if (fabs(C->element(a, b)) > 0.00001) {
          cs_entry (T, i, j, C->element(a, b));
        }
      }
    }
  }
//  print();
//  cs_print(T, 0);
}
