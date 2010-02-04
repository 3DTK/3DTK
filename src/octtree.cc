/**
 * @file 
 * @brief Implementation of an octree
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
*/

#include "octtree.h"
#include "globals.icc"
double OctTree::voxelSize;

/**
 * Constructor
 * @param pts 2-dim array of the points to store in the octree
 * @param n number of points
 * @param voxelSize the size of the voxels
 */
OctTree::OctTree(double **pts, int n, double voxelSize) {
  this->voxelSize = voxelSize;

  double xmin = pts[0][0], xmax = pts[0][0];
  double ymin = pts[0][1], ymax = pts[0][1];
  double zmin = pts[0][2], zmax = pts[0][2];
  for (int i = 1; i < n; i++) {
    xmin = min(xmin, pts[i][0]);
    xmax = max(xmax, pts[i][0]);
    ymin = min(ymin, pts[i][1]);
    ymax = max(ymax, pts[i][1]);
    zmin = min(zmin, pts[i][2]);
    zmax = max(zmax, pts[i][2]); 
  }
  center[0] = 0.5 * (xmin+xmax);
  center[1] = 0.5 * (ymin+ymax);
  center[2] = 0.5 * (zmin+zmax);
  x_size = y_size = z_size = max(max(0.5 * (xmax-xmin), 0.5 * (ymax-ymin)), 0.5 * (zmax-zmin));

  // set up values
  child = new OctTree*[8];
  child[0] = child[1] = child[2] = child[3] = 0;
  child[4] = child[5] = child[6] = child[7] = 0; 
  leaf = false;

  // calculate new buckets
  double newcenter[8][3];

  double x_sizeNew = x_size / 2.0;
  double y_sizeNew = y_size / 2.0;
  double z_sizeNew = z_size / 2.0;

  newcenter[0][0] = center[0] - x_size / 2.0;
  newcenter[0][1] = center[1] - y_size / 2.0;
  newcenter[0][2] = center[2] - z_size / 2.0;

  newcenter[1][0] = center[0] + x_size / 2.0;
  newcenter[1][1] = center[1] - y_size / 2.0;
  newcenter[1][2] = center[2] - z_size / 2.0;

  newcenter[2][0] = center[0] - x_size / 2.0;
  newcenter[2][1] = center[1] + y_size / 2.0;
  newcenter[2][2] = center[2] - z_size / 2.0;

  newcenter[3][0] = center[0] - x_size / 2.0;
  newcenter[3][1] = center[1] - y_size / 2.0;
  newcenter[3][2] = center[2] + z_size / 2.0;
  
  newcenter[4][0] = center[0] + x_size / 2.0;
  newcenter[4][1] = center[1] + y_size / 2.0;
  newcenter[4][2] = center[2] - z_size / 2.0;

  newcenter[5][0] = center[0] + x_size / 2.0;
  newcenter[5][1] = center[1] - y_size / 2.0;
  newcenter[5][2] = center[2] + z_size / 2.0;

  newcenter[6][0] = center[0] - x_size / 2.0;
  newcenter[6][1] = center[1] + y_size / 2.0;
  newcenter[6][2] = center[2] + z_size / 2.0;

  newcenter[7][0] = center[0] + x_size / 2.0;
  newcenter[7][1] = center[1] + y_size / 2.0;
  newcenter[7][2] = center[2] + z_size / 2.0;

  countPointsAndQueue(pts, n, newcenter, x_sizeNew, y_sizeNew, z_sizeNew, child);
}

/**
 * Private constructor: Needed for recursive bulding of the octree
 */
OctTree::OctTree(vector<double*> &splitPoints, double _center[3], 
                 double _x_size, double _y_size, double _z_size)
{
  // set up values
  child = new OctTree*[8];
  child[0] = child[1] = child[2] = child[3] = 0;
  child[4] = child[5] = child[6] = child[7] = 0; 
  leaf = false;

  center[0] = _center[0];
  center[1] = _center[1];
  center[2] = _center[2];
  x_size    = _x_size;
  y_size    = _y_size;
  z_size    = _z_size;

  // if bucket is too small stop building tree
  // -----------------------------------------
  if ((z_size <= voxelSize)) {
    // copy points
    for (unsigned int i = 0; i < splitPoints.size(); i++) {
      points.push_back(splitPoints[i]);
    }
    leaf = true;
    return; 
  }  

  // calculate new buckets
  double newcenter[8][3];
  double x_sizeNew, y_sizeNew, z_sizeNew;

  x_sizeNew = x_size / 2.0;
  y_sizeNew = y_size / 2.0;
  z_sizeNew = z_size / 2.0;

  newcenter[0][0] = center[0] - x_size / 2.0;
  newcenter[0][1] = center[1] - y_size / 2.0;
  newcenter[0][2] = center[2] - z_size / 2.0;

  newcenter[1][0] = center[0] + x_size / 2.0;
  newcenter[1][1] = center[1] - y_size / 2.0;
  newcenter[1][2] = center[2] - z_size / 2.0;

  newcenter[2][0] = center[0] - x_size / 2.0;
  newcenter[2][1] = center[1] + y_size / 2.0;
  newcenter[2][2] = center[2] - z_size / 2.0;

  newcenter[3][0] = center[0] - x_size / 2.0;
  newcenter[3][1] = center[1] - y_size / 2.0;
  newcenter[3][2] = center[2] + z_size / 2.0;
  
  newcenter[4][0] = center[0] + x_size / 2.0;
  newcenter[4][1] = center[1] + y_size / 2.0;
  newcenter[4][2] = center[2] - z_size / 2.0;

  newcenter[5][0] = center[0] + x_size / 2.0;
  newcenter[5][1] = center[1] - y_size / 2.0;
  newcenter[5][2] = center[2] + z_size / 2.0;

  newcenter[6][0] = center[0] - x_size / 2.0;
  newcenter[6][1] = center[1] + y_size / 2.0;
  newcenter[6][2] = center[2] + z_size / 2.0;

  newcenter[7][0] = center[0] + x_size / 2.0;
  newcenter[7][1] = center[1] + y_size / 2.0;
  newcenter[7][2] = center[2] + z_size / 2.0;

  countPointsAndQueue(splitPoints, newcenter, x_sizeNew, y_sizeNew, z_sizeNew, child);
}

/**
 * Returns the number of represented points in the octree
 * (and stores them in the vector points, after checking the size constraints)
 * @param i_points the source vector
 * @param points the target vector
 * @param center the center
 * @param x_size maximal distance from the center (x direction)
 * @param y_size maximal distance from the center (y direction)
 * @param z_size maximal distance from the center (z direction)
 */
void OctTree::countPointsAndQueue(const vector<double*> &i_points,
                                 double center[8][3], 
                                 double x_size, double y_size, double z_size,
						   OctTree **child)
{
  vector<double*> points[8];
  int count[8] = {0, 0, 0, 0, 0, 0, 0, 0};

  for (unsigned int i = 0; i < i_points.size(); i++) {
    for (int j = 0; j < 8; j++) {
	 if (fabs(i_points[i][0] - center[j][0]) <= x_size) {
	   if (fabs(i_points[i][1] - center[j][1]) <= y_size) {
		if (fabs(i_points[i][2] - center[j][2]) <= z_size) {
		  count[j]++;
		  points[j].push_back(i_points[i]);
		  break;
		}
	   }
	 }
    }
  }
  for (int j = 0; j < 8; j++) {
    if (count[j] > 0) {
	 child[j] = new OctTree(points[j], center[j], x_size, y_size, z_size); 
    } else {
	 child[j] = 0;
    }
  }
}

/**
 * Returns the number of represented points in the octree
 * (and stores them in the vector points, after checking the size constraints)
 * @param pts the source points (2-dim array)
 * @param n number of points in the source array
 * @param points the target vector
 * @param center the center
 * @param x_size maximal distance from the center (x direction)
 * @param y_size maximal distance from the center (y direction)
 * @param z_size maximal distance from the center (z direction)
 */
void OctTree::countPointsAndQueue(double **pts, int n, 
                                 double center[8][3], 
                                 double x_size, double y_size, double z_size,
						   OctTree **child)
{
  vector<double*> points[8];
  int count[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < 8; j++) {
	 if (fabs(pts[i][0] - center[j][0]) <= x_size) {
	   if (fabs(pts[i][1] - center[j][1]) <= y_size) {
		if (fabs(pts[i][2] - center[j][2]) <= z_size) {
		  count[j]++;
		  points[j].push_back( pts[i] );
		  break;
		}
	   }
	 } 
    }
  }
  
  for (int j = 0; j < 8; j++) {
    if (count[j] > 0) {
	 child[j] = new OctTree(points[j], center[j], x_size, y_size, z_size); 
    } else {
	 child[j] = 0;
    }
  }
}


/**
 * returns the octree center in the parameter c
 */
void OctTree::GetOctTreeCenter(vector<double*> &c)
{
  if (leaf == true) {
    c.push_back(center);
    return;
  }

  for( int i = 0; i < 8; i++){ 
    if (child[i] != 0) {
      child[i]->GetOctTreeCenter(c);
    }
  }
}


/**
 * Destructor
 */
OctTree::~OctTree() {
  for(int i = 0; i < 8; i++) {
    if (child[i] != 0) {
      delete child[i];
      child[i] = 0;
    }
  }
  delete [] child;
  child = 0;
}
