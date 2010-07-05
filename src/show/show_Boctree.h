/**
 * @file 
 * @brief Representation of an octree for show
 * @author Jan Elseberg. Jacobs University Bremen gGmbH, Germany
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
 */

#ifndef SHOWBOCTREE_H
#define SHOWBOCTREE_H


#include "../Boctree.h"
#include "colormanager.h"
#include "scancolormanager.h"


class ScanColorManager;

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

  Show_BOctTree(double **pts, int n, double voxelSize, unsigned int pointdim = 3, ScanColorManager *_cm = 0);

  void setColorManager(ColorManager *_cm);

  void displayOctTreeCulled(long targetpts);
  void displayOctTreeAllCulled();

protected:
  
  void displayOctTreeAll( bitoct &node, double *center, double size );
  void displayOctTreeAllCulled( bitoct &node, double *center, double size );
  void displayOctTreeCulledLOD(long targetpts, bitoct &node, double *center, double size );
  void displayOctTreeLOD(long targetpts, bitoct &node, double *center, double size );

  ColorManager *cm;
};

#endif
