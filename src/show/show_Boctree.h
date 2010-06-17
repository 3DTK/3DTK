/**
 * @file 
 * @brief Representation of an octree for show
 * @author Jan Elseberg. Jacobs University Bremen gGmbH, Germany
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#ifndef SHOWBOCTREE_H
#define SHOWBOCTREE_H


#include "../Boctree.h"

/**
 * @brief Octree for show
 * 
 * A cubic bounding box is calculated
 * from the given 3D points. Then it
 * is recusivly subdivided into smaller
 * subboxes
 *
 * It contains software culling functionalities
 */
class Show_BOctTree : public BOctTree  {

public:

  Show_BOctTree(double **pts, int n, double voxelSize);
  
  void displayOctTreeCulled(long targetpts);
  void displayOctTreeAllCulled();

protected:
  
  void displayOctTree(long targetpts, bitoct &node);
  void displayOctTreeAll( bitoct &node, double *center, double size );
  void displayOctTreeCulled(long targetpts, bitoct &node, double *center, double size );
  void displayOctTreeAllCulled( bitoct &node, double *center, double size );
  
};

#endif
