/** @file 
 *  @brief Implementation of the virtual functor for ICP error function minimization
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#ifndef __ICP6DMINIMIZER__
#define __ICP6DMINIMIZER__

#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#include <vector>
using std::vector;

#ifdef _OPENMP
#include <omp.h>
#endif

#include "ptpair.h"

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <stdlib.h>

class icp6Dminimizer {

public:
  /** 
   * Constructor 
   */
  icp6Dminimizer(bool quiet = false) { this->quiet = quiet; };
  /** 
   * Destructor 
   */
  virtual ~icp6Dminimizer() {};                                

  /**
   * aligning the point pairs
   */
  // a detailed discussion of the minimization techniques used for this
  // function is given in:
  // Andreas Nuechter, Jan Elseberg, Peter Schneider, and Dietrich Paulus.
  // Study of Parameterizations for the Rigid Body Transformations of The
  // Scan Registration Problem, Journal Computer Vision and Image
  // Understanding (CVIU), Elsevier Science, Volume 114, Issue 8,
  // pp. 963-980, August 2010.
  virtual double Point_Point_Align(const vector<PtPair>& Pairs, double *alignxf,
							const double centroid_m[3], const double centroid_d[3]) = 0;
  
  /**
   * aligning the point pairs parallel algorithms
   */
  virtual double Point_Point_Align_Parallel(const int openmp_num_threads, 
								    const unsigned int n[OPENMP_NUM_THREADS],
								    const double sum[OPENMP_NUM_THREADS], 
								    const double centroid_m[OPENMP_NUM_THREADS][3],
								    const double centroid_d[OPENMP_NUM_THREADS][3], 
								    const double Si[OPENMP_NUM_THREADS][9], 
								    double *alignxf)
  {
    cout << "this function is not implemented!!!" << endl;
    exit(-1);
  }
  virtual double Point_Point_Align_Parallel(const int openmp_num_threads, 
								    const unsigned int n[OPENMP_NUM_THREADS],
								    const double sum[OPENMP_NUM_THREADS], 
								    const double centroid_m[OPENMP_NUM_THREADS][3],
								    const double centroid_d[OPENMP_NUM_THREADS][3], 
								    const vector<PtPair> pairs[OPENMP_NUM_THREADS],
								    double *alignxf)
  {
    cout << "this function is not implemented!!!" << endl;
    exit(-1);
  }

  virtual int getAlgorithmID() = 0; 

protected:
  bool quiet; ///< determines the verbosity
};

#endif 
