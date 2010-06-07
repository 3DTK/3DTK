/**
 * @file 
 * @brief Representation of an octree for show
 * @author Jan Elseberg. Jacobs University Bremen gGmbH, Germany
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#ifndef SHOWOCTREE_H
#define SHOWOCTREE_H

#include <stdbool.h>

#include "octree.h"

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
class Show_OctTree : public OctTree  {

public:

  Show_OctTree(double **pts, int n, double voxelSize);
  Show_OctTree(list<double*> &splitPoints, double center[3], 
			double x_size, double y_size, double z_size);
  
  void displayOctTree(long targetpts);
  void displayOctTreeAll();
  int  cullOctTree();

protected:

  void countPointsAndQueue(list<double*> &i_points,
					  double center[8][3], 
					  double x_size, double y_size, double z_size,
					  Show_OctTree **child);
  
  void countPointsAndQueue(double **pts, int n,
					  double center[8][3], 
					  double x_size, double y_size, double z_size,
					  Show_OctTree **child);
  void setVisible();

  bool culled;
  int nrpts;

};

#endif
