/**
 * @file 
 * @brief Implementation of an octree for show
 * @author Jan Elseberg. Jacobs University Bremen gGmbH, Germany
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
*/

#include "show_octree.h"
#include "../globals.icc"

#include "viewcull.h"
//#include "show/glui/glui.h"  /* Header File For The glui functions */


Show_OctTree::Show_OctTree(double **pts, int n, double voxelSize)
  : OctTree(pts, n, voxelSize)
{ }


Show_OctTree::Show_OctTree(list<double*> &splitPoints, double center[3], 
					  double x_size, double y_size, double z_size)
  : OctTree(splitPoints, center, x_size, y_size, z_size)
{ }  


/**
 * sets the culled flag to false in this and all child nodes. Needed for cullOctTree()
 */
void Show_OctTree::setVisible() {
  culled = false;
  for( int i = 0; i < 8; i++){
    if (child[i] ) {
      ((Show_OctTree*)child[i])->setVisible();
    }
  }
} 

/**
 * calculate which buckets are visible and count visible points.
 */
int Show_OctTree::cullOctTree() {
  nrpts = 0;
  int res = QuadInFrustrum2(center[0], center[1], center[2], x_size, y_size, z_size);
  culled = (res == 0);

  if (!culled && !leaf) {
    for( int i = 0; i < 8; i++){
      if (child[i] != 0 ) {
        if (res == 1) { // only recurse if partially overlapping the frustrum
          ((Show_OctTree*)child[i])->cullOctTree();
          if (!((Show_OctTree*)child[i])->culled) {
            nrpts += ((Show_OctTree*)child[i])->nrpts;
          }
        } else {
          // TODO compute number of points correctly
          nrpts += ((Show_OctTree*)child[i])->nrpts;
          setVisible();
        }        
      }
    }
  } else if (leaf) {
    nrpts = points.size(); 
  }
  return nrpts;
  
}

/**
 * Displays all non culled points
 */
void Show_OctTree::displayOctTreeAll() {
  if(leaf == true) {
    glBegin(GL_POINTS);
    for(list<double *>::iterator itl = points.begin(); itl != points.end(); itl++) {
      glVertex3f((*itl)[0],(*itl)[1],(*itl)[2]);
    }
    glEnd();
  } else {
    for( int i = 0; i < 8; i++){
      if (child[i] != 0 && !((Show_OctTree*)child[i])->culled) {
        ((Show_OctTree*)child[i])->displayOctTreeAll();
      }
    }
  }
}

/**
 * Displays non culled points. Tries to display only targetpts points, but will usually display four times as much
 *
 */
void Show_OctTree::displayOctTree(long targetpts) {
  if (culled) return;
  if(leaf == true && targetpts > 0) {
    glBegin(GL_POINTS);
    if (points.size() <= targetpts) {
      for(list<double *>::iterator itl = points.begin(); itl != points.end(); itl++) {
        glVertex3f((*itl)[0],(*itl)[1],(*itl)[2]);
      }
    } else {
      // some optimization, since this case occurs quite often
      if (targetpts == 1) {
        double *p = *(points.begin());
        glVertex3f(p[0],p[1],p[2]);
      } else {
        // TODO smarter subselection of points here
        int each = points.size()/targetpts;
        int i = 0;
        for(list<double *>::iterator itl = points.begin(); itl != points.end(); itl++) {
          i++;
          if ( i%each == 0 ) {
            glVertex3f((*itl)[0],(*itl)[1],(*itl)[2]);
          }
        }
      }
    }
    glEnd();
  } else {
    int count = 0;
    for( int i = 0; i < 8; i++){
      if (child[i] != 0 && !((Show_OctTree*)child[i])->culled) { // check wether child exists and if its displayed
        count++;
        //count += child[i]->nrpts;
      }
    }
    if (count == 0) return;
    for( int i = 0; i < 8; i++){
      if (child[i] != 0 && !((Show_OctTree*)child[i])->culled) {
        //double percentage = ((double)((Show_OctTree*)child[i])->nrpts)/((double)count);
        //child[i]->displayOctTree(percentage*targetpts +1);  // plus 1 results in many many more extra points
        ((Show_OctTree*)child[i])->displayOctTree(targetpts/count);  
      }
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
void Show_OctTree::countPointsAndQueue(list<double*> &i_points,
							    double center[8][3], 
							    double x_size, double y_size, double z_size,
							    Show_OctTree **child)
{
  list<double*> points;

  for (int j = 0; j < 7; j++) {
    for ( list<double *>::iterator itr = i_points.begin(); itr != i_points.end();) {
      if (fabs((*itr)[0] - center[j][0]) <= x_size) {
        if (fabs((*itr)[1] - center[j][1]) <= y_size) {
          if (fabs((*itr)[2] - center[j][2]) <= z_size) {
			 points.push_back(*itr);
             itr = i_points.erase(itr);
		     continue;
          }
        }
	  }
	  itr++;
	}
    if (points.size() > 0) {
      child[j] = new Show_OctTree(points, center[j], x_size, y_size, z_size); 
      points.clear();
    } else {
      child[j] = 0;
    }
  }
  // remaining child
  if (i_points.size() > 0) {
    child[7] = new Show_OctTree(i_points,center[7], x_size, y_size, z_size); 
    i_points.clear();
  } else {
    child[7] = 0;
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
void Show_OctTree::countPointsAndQueue(double **pts, int n, 
							    double center[8][3], 
							    double x_size, double y_size, double z_size,
							    Show_OctTree **child)
{
  list<double*> points;

  for (int j = 0; j < 8; j++) {
    for (int i = 0; i < n; i++) {
      if (fabs(pts[i][0] - center[j][0]) <= x_size) {
        if (fabs(pts[i][1] - center[j][1]) <= y_size) {
          if (fabs(pts[i][2] - center[j][2]) <= z_size) {
            points.push_back( pts[i] );
          }
        }
      } 
    }
    if (points.size() > 0) {
      child[j] = new Show_OctTree(points, center[j], x_size, y_size, z_size); 
      points.clear();
    } else {
      child[j] = 0;
    }
  }
}











