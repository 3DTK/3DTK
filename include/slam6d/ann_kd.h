/** @file 
 *  @brief Encapsules the implementation of ANN k-d trees. 
 *  @author Ulugbek Makhmudov, Jacobs University Bremen, Bremen, Germany.
 *  @author Andreas Nuechter, Jacobs University Bremen, Bremen, Germany.
 */

#ifndef __ANN_KD_H__
#define __ANN_KD_H__

#include "slam6d/kdparams.h"
#include "slam6d/searchTree.h"
#include "ANN/ANN.h"

/**
 * @brief Encapsulating class to create and store ANN KD Trees. 
 * 
 * A kD tree for points, with limited
 * capabilities (find nearest point to
 * a given point, or in a fixed radius distance).
 **/
class ANNtree : public SearchTree {

public:
  
  /**
   * default constructor
   */
  ANNtree();
  
  /**
   * Constructor using the point set pts and the num_of_pts n
   */
  ANNtree(PointerArray<double>&_pts, int n);
  
  /**
   * destructor
   */
  virtual ~ANNtree(); 
  
  
 /**
  * Finds the closest point within the tree,
  * wrt. the point given as first parameter.
  * @param _p point
  * @param maxdist2 maximal search distance.
  * @param threadNum Thread number, for parallelization
  * @return Pointer to the closest point
  */  
  double *FindClosest(double *_p, double maxdist2, int threadNum = 0) const;

private:

  /**
   * a pointer to ANNkd_tree instance
   */
  ANNkd_tree* annkd;
  ANNdistArray nn; //temporary ANNdistArray instance to use for storing the nearest neighbor
  ANNidxArray nn_idx; //temporary ANNdistArray instance to use for storing the nearest neighbor

  double** pts;
  
};

#endif


