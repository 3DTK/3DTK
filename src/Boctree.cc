/**
 * @file 
 * @brief Implementation of an octree
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
*/

#include "Boctree.h"
#include "globals.icc"
double BOctTree::voxelSize;

/**
 * Constructor
 * @param pts 2-dim array of the points to store in the octree
 * @param n number of points
 * @param voxelSize the size of the voxels
 */
BOctTree::BOctTree(double **pts, int n, double voxelSize) {
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
  size = max(max(0.5 * (xmax-xmin), 0.5 * (ymax-ymin)), 0.5 * (zmax-zmin));


  // calculate new buckets
  double newcenter[8][3];
  double sizeNew = size / 2.0;

  for (int i = 0; i < 8; i++) {
    childcenter(center, newcenter[i], size, i);
  }
  // set up values
  root = new bitoct();
 
  countPointsAndQueue(pts, n, newcenter, sizeNew, *root);
}

/**
 * Constructor
 * @param pts 2-dim array of the points to store in the octree
 * @param n number of points
 * @param voxelSize the size of the voxels
 */

BOctTree::BOctTree(deque<double*> pts, double voxelSize) {
  this->voxelSize = voxelSize;
    
  deque<double *>::iterator itr = pts.begin();  
  double xmin = (*itr)[0], xmax = (*itr)[0];
  double ymin = (*itr)[1], ymax = (*itr)[1];
  double zmin = (*itr)[2], zmax = (*itr)[2];
  
  while(itr != pts.end()) {
    xmin = min(xmin, (*itr)[0]);
    xmax = max(xmax, (*itr)[0]);
    ymin = min(ymin, (*itr)[1]);
    ymax = max(ymax, (*itr)[1]);
    zmin = min(zmin, (*itr)[2]);
    zmax = max(zmax, (*itr)[2]); 
    itr++;
  }

  center[0] = 0.5 * (xmin+xmax);
  center[1] = 0.5 * (ymin+ymax);
  center[2] = 0.5 * (zmin+zmax);
  size = max(max(0.5 * (xmax-xmin), 0.5 * (ymax-ymin)), 0.5 * (zmax-zmin));


  // calculate new buckets
  double newcenter[8][3];
  double sizeNew = size / 2.0;

  for (int i = 0; i < 8; i++) {
    childcenter(center, newcenter[i], size, i);
  }
  // set up values
  root = new bitoct();
 
  countPointsAndQueue(pts, newcenter, sizeNew, *root);
}


BOctTree::BOctTree(vector<double*> pts, double voxelSize) {
  this->voxelSize = voxelSize;
    
  vector<double *>::iterator itr = pts.begin();  
  double xmin = (*itr)[0], xmax = (*itr)[0];
  double ymin = (*itr)[1], ymax = (*itr)[1];
  double zmin = (*itr)[2], zmax = (*itr)[2];
  
  while(itr != pts.end()) {
    xmin = min(xmin, (*itr)[0]);
    xmax = max(xmax, (*itr)[0]);
    ymin = min(ymin, (*itr)[1]);
    ymax = max(ymax, (*itr)[1]);
    zmin = min(zmin, (*itr)[2]);
    zmax = max(zmax, (*itr)[2]); 
    itr++;
  }

  center[0] = 0.5 * (xmin+xmax);
  center[1] = 0.5 * (ymin+ymax);
  center[2] = 0.5 * (zmin+zmax);
  size = max(max(0.5 * (xmax-xmin), 0.5 * (ymax-ymin)), 0.5 * (zmax-zmin));


  // calculate new buckets
  double newcenter[8][3];
  double sizeNew = size / 2.0;

  for (int i = 0; i < 8; i++) {
    childcenter(center, newcenter[i], size, i);
  }
  // set up values
  root = new bitoct();
 
  countPointsAndQueue(pts, newcenter, sizeNew, *root);
}


/**
 * Needed for recursive bulding of the octree
 */
//list<double*>* BOctTree::branch(bitoct &node, list<double*> &splitPoints, 
pointrep* BOctTree::branch(bitoct &node, deque<double*> &splitPoints, 
              double _center[3], double _size)
{

  // if bucket is too small stop building tree
  // -----------------------------------------
  if ((_size <= voxelSize)) {
    // copy points
    /*
    list<double*> *points = new list<double*>();
    for (list<double *>::iterator itr = splitPoints.begin(); 
        itr != splitPoints.end(); itr++) {
      points->push_back(*itr);
    }*/
    pointrep *points = new pointrep[3*splitPoints.size() + 1];
    points[0].length = splitPoints.size();
    int i = 1;
    for (deque<double *>::iterator itr = splitPoints.begin(); 
        itr != splitPoints.end(); itr++) {
      points[i++].v = (*itr)[0];
      points[i++].v = (*itr)[1];
      points[i++].v = (*itr)[2];
    }
    return points; 
  }  

  // calculate new buckets
  double newcenter[8][3];
  double sizeNew;

  sizeNew = _size / 2.0;

  for (int i = 0; i < 8; i++) {
    childcenter(_center, newcenter[i], _size, i);
  }

  countPointsAndQueue(splitPoints, newcenter, sizeNew, node);
  return 0;
}

pointrep* BOctTree::branch(bitoct &node, vector<double*> &splitPoints, 
              double _center[3], double _size)
{

  // if bucket is too small stop building tree
  // -----------------------------------------
  if ((_size <= voxelSize)) {
    // copy points
    /*
    list<double*> *points = new list<double*>();
    for (list<double *>::iterator itr = splitPoints.begin(); 
        itr != splitPoints.end(); itr++) {
      points->push_back(*itr);
    }*/
    pointrep *points = new pointrep[3*splitPoints.size() + 1];
    points[0].length = splitPoints.size();
    int i = 1;
    for (vector<double *>::iterator itr = splitPoints.begin(); 
        itr != splitPoints.end(); itr++) {
      points[i++].v = (*itr)[0];
      points[i++].v = (*itr)[1];
      points[i++].v = (*itr)[2];
    }
    return points; 
  }  

  // calculate new buckets
  double newcenter[8][3];
  double sizeNew;

  sizeNew = _size / 2.0;

  for (int i = 0; i < 8; i++) {
    childcenter(_center, newcenter[i], _size, i);
  }

  countPointsAndQueue(splitPoints, newcenter, sizeNew, node);
  return 0;
}
  

void BOctTree::childcenter(double *pcenter, double *ccenter, double size, int i) {
  switch (i) {
    case 0:
      ccenter[0] = pcenter[0] - size / 2.0;
      ccenter[1] = pcenter[1] - size / 2.0;
      ccenter[2] = pcenter[2] - size / 2.0;
      break;
    case 1:
      ccenter[0] = pcenter[0] + size / 2.0;
      ccenter[1] = pcenter[1] - size / 2.0;
      ccenter[2] = pcenter[2] - size / 2.0;
      break;
    case 2:
      ccenter[0] = pcenter[0] - size / 2.0;
      ccenter[1] = pcenter[1] + size / 2.0;
      ccenter[2] = pcenter[2] - size / 2.0;
      break;
    case 3:
      ccenter[0] = pcenter[0] - size / 2.0;
      ccenter[1] = pcenter[1] - size / 2.0;
      ccenter[2] = pcenter[2] + size / 2.0;
      break;
    case 4:
      ccenter[0] = pcenter[0] + size / 2.0;
      ccenter[1] = pcenter[1] + size / 2.0;
      ccenter[2] = pcenter[2] - size / 2.0;
      break;
    case 5:
      ccenter[0] = pcenter[0] + size / 2.0;
      ccenter[1] = pcenter[1] - size / 2.0;
      ccenter[2] = pcenter[2] + size / 2.0;
      break;
    case 6:
      ccenter[0] = pcenter[0] - size / 2.0;
      ccenter[1] = pcenter[1] + size / 2.0;
      ccenter[2] = pcenter[2] + size / 2.0;
      break;
    case 7:
      ccenter[0] = pcenter[0] + size / 2.0;
      ccenter[1] = pcenter[1] + size / 2.0;
      ccenter[2] = pcenter[2] + size / 2.0;
      break;
    default:
      break;
  }

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
void BOctTree::countPointsAndQueue(deque<double*> &i_points,
                                 double center[8][3], 
                                 double size,
                                 bitoct &parent)
{
  deque<double*> points[8];
  int n_children = 0;
  

  for (int j = 0; j < 8; j++) {
    for ( deque<double *>::iterator itr = i_points.begin(); itr != i_points.end(); itr++) {
      if (fabs((*itr)[0] - center[j][0]) <= size) {
        if (fabs((*itr)[1] - center[j][1]) <= size) {
          if (fabs((*itr)[2] - center[j][2]) <= size) {
            points[j].push_back(*itr);
            continue;
          }
        }
      }
    }
  }
  i_points.clear();
  for (int j = 0; j < 8; j++) {
    if (!points[j].empty()) {
      parent.valid = ( 1 << j ) | parent.valid;
      ++n_children;
    }
  }

  // create children
  bitunion *children = new bitunion[n_children];
  bitoct::link(parent, children);

  int count = 0;
  for (int j = 0; j < 8; j++) {
    if (!points[j].empty()) {
      pointrep *c = branch(children[count].node, points[j], center[j], size);  // leaf node
      if (c) { 
        children[count].points = c; // set this child to deque of points
        parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
      }
      points[j].clear();
      ++count;
    }
  }

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
void BOctTree::countPointsAndQueue(vector<double*> &i_points,
                                 double center[8][3], 
                                 double size,
                                 bitoct &parent)
{
  vector<double*> points[8];
  int n_children = 0;

  for (int j = 0; j < 8; j++) {
    for ( vector<double *>::iterator itr = i_points.begin(); itr != i_points.end(); itr++) {
      if (fabs((*itr)[0] - center[j][0]) <= size) {
        if (fabs((*itr)[1] - center[j][1]) <= size) {
          if (fabs((*itr)[2] - center[j][2]) <= size) {
            points[j].push_back(*itr);
            continue;
          }
        }
      }
    }
  }
  
  i_points.clear();
  vector<double*>().swap(i_points);
  for (int j = 0; j < 8; j++) {
    if (!points[j].empty()) {
      parent.valid = ( 1 << j ) | parent.valid;
      ++n_children;
    }
  }

  // create children
  bitunion *children = new bitunion[n_children];
  bitoct::link(parent, children);

  int count = 0;
  for (int j = 0; j < 8; j++) {
    if (!points[j].empty()) {
      pointrep *c = branch(children[count].node, points[j], center[j], size);  // leaf node
      if (c) { 
        children[count].points = c; // set this child to deque of points
        parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
      }
      points[j].clear();
      vector<double*>().swap(points[j]);
      ++count;
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
void BOctTree::countPointsAndQueue(double **pts, int n, 
                                 double center[8][3], 
                                 double size,
				 bitoct &parent)
{
  vector<double*> points[8];
  int n_children = 0;
  for (int j = 0; j < 8; j++) {
    for (int i = 0; i < n; i++) {
      if (fabs(pts[i][0] - center[j][0]) <= size) {
        if (fabs(pts[i][1] - center[j][1]) <= size) {
          if (fabs(pts[i][2] - center[j][2]) <= size) {
            points[j].push_back( pts[i] );
          }
        }
      } 
    }
    // if non-empty set valid flag for this child
    if (!points[j].empty()) {
      parent.valid = ( 1 << j ) | parent.valid;
      ++n_children;
    }
  }

  // create children
  bitunion *children = new bitunion[n_children];
  bitoct::link(parent, children);
  int count = 0;
  for (int j = 0; j < 8; j++) {
    if (!points[j].empty()) {
      pointrep *c = branch(children[count].node, points[j], center[j], size);  // leaf node
      if (c) { 
        children[count].points = c; // set this child to vector of points
        parent.leaf = ( 1 << j ) | parent.leaf;  // remember this is a leaf
      }
      points[j].clear();
      vector<double*>().swap(points[j]);
      ++count;
    }
  }
}

long BOctTree::countNodes() {
  return 1 + countNodes(*root);
}

long BOctTree::countNodes(bitoct &node) {
  long result = 0;
  bitunion *children;
  bitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
        ++result;
      } else { // recurse
        result += countNodes(children->node) + 1;
      }
      ++children; // next child
    }
  }

  return result;
}

long BOctTree::countLeaves() {
  return 1 + countLeaves(*root);
}

long BOctTree::countLeaves(bitoct &node) {
  long result = 0;
  bitunion *children;
  bitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
        pointrep *points = children->points;
        long nrpts = points[0].length;
        result += 3*nrpts + 1;
      } else { // recurse
        result += countLeaves(children->node);
      }
      ++children; // next child
    }
  }

  return result;
}
void BOctTree::GetOctTreeRandom(vector<double*> &c, unsigned int ptspervoxel)
{
  GetOctTreeRandom(c, ptspervoxel, *root);
}

void BOctTree::GetOctTreeRandom(vector<double*>&c, unsigned int ptspervoxel, bitoct &node) {
  
  bitunion *children;
  bitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
        pointrep *points = children->points;
        unsigned int length = points[0].length;
        if (ptspervoxel >= length) {
          for (unsigned int j = 0; j < length; j++) 
            c.push_back(&(points[3*j+1].v));

          ++children; // next child
          continue;
        }
        set<int> indices;
        while(indices.size() < ptspervoxel) {
          int tmp = rand(length-1);
          indices.insert(tmp);
        }
        for(set<int>::iterator it = indices.begin(); it != indices.end(); it++) 
          c.push_back(&(points[3*(*it)+1].v));

      } else { // recurse
        GetOctTreeRandom(c, ptspervoxel, children->node);
      }
      ++children; // next child
    }
  }
} 

void BOctTree::GetOctTreeRandom(vector<double*> &c)
{
  GetOctTreeRandom(c, *root);
}

void BOctTree::GetOctTreeRandom(vector<double*>&c, bitoct &node) {
  
  bitunion *children;
  bitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
        pointrep *points = children->points;
        int tmp = rand(points[0].length);
        double *point = &(points[3*tmp+1].v);
        c.push_back(point);

      } else { // recurse
        GetOctTreeRandom(c, children->node);
      }
      ++children; // next child
    }
  }
} 

/**
 * returns the octree center in the parameter c
 */
void BOctTree::GetOctTreeCenter(vector<double*> &c)
{
  GetOctTreeCenter(c, *root, center, size);
}

void BOctTree::GetOctTreeCenter(vector<double*>&c, bitoct &node, double *center, double size) {
  double ccenter[3];
  bitunion *children;
  bitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        double * cp = new double[3];
        cp[0] = ccenter[0];
        cp[1] = ccenter[1];
        cp[2] = ccenter[2];
        c.push_back(cp);
      } else { // recurse
        GetOctTreeCenter(c, children->node, ccenter, size/2.0);
      }
      ++children; // next child
    }
  }


}



/**
 * Destructor
 */
BOctTree::~BOctTree() {
  deletetNodes(*root);
  delete root;

}
  
void BOctTree::deletetNodes(bitoct &node) {
  bitunion *children;
  bitoct::getChildren(node, children);
  bool haschildren = false;

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
        delete [] children->points;
      } else { // recurse
        deletetNodes(children->node);
      }
      ++children; // next child
      haschildren = true;
    }
  }
  // delete children
  if (haschildren) {
    bitoct::getChildren(node, children);
    delete[] children;
  }
  
}
