/**
 * @file 
 * @brief Implementation of an octree for show
 * @author Jan Elseberg. Jacobs University Bremen gGmbH, Germany
 * @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany
*/

#include "show_Boctree.h"
#include "../globals.icc"

#include "viewcull.h"

#if __GNUC__ > 3 || (__GNUC__ == 3 && __GNUC_MINOR__ >= 4)
  #define POPCOUNT(mask) __builtin_popcount(mask)
#else
//#define POPCOUNT(mask) 8

unsigned char _my_popcount_3(unsigned char x) {
  x -= (x >> 1) & 0x55;             //put count of each 2 bits into those 2 bits
  x = (x & 0x33) + ((x >> 2) & 0x33); //put count of each 4 bits into those 4 bits 
  x = (x + (x >> 4)) & 0x0f;        //put count of each 8 bits into those 8 bits 
  return x;
}
#define POPCOUNT(mask) _my_popcount_3(mask)

#endif


Show_BOctTree::Show_BOctTree(double **pts, int n, double voxelSize)
  : BOctTree(pts, n, voxelSize)
{ }

void Show_BOctTree::displayOctTreeAllCulled() {
  displayOctTreeAll(*root, center, size);
}

void Show_BOctTree::displayOctTreeCulled(long targetpts) {
  displayOctTreeCulled(targetpts, *root, center, size);
}

/**
 * Displays all points but does not perform culling
 */
void Show_BOctTree::displayOctTreeAll(bitoct &node, double *center, double size) {
  double ccenter[3];
  bitunion *children;
  bitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        pointrep *points = children->points;
        unsigned int length = points[0].length;
        double *point = &(points[1].v);  // first point
        glBegin(GL_POINTS);
        for(unsigned int iterator = 0; iterator < length; iterator++ ) {
          glVertex3f( point[0], point[1], point[2]);
          point+=3;
        }
        glEnd();
      } else { // recurse
        childcenter(center, ccenter, size, i);  // childrens center
        displayOctTreeAll( children->node, ccenter, size/2.0);
      }
      ++children; // next child
    }
  }
}

/**
 * Displays all non culled points
 */
void Show_BOctTree::displayOctTreeAllCulled(bitoct &node, double *center, double size) {
  int res = QuadInFrustrum2(center[0], center[1], center[2], size, size, size);
  if (res==0) return;  // culled do not continue with this branch of the tree
  
  if (res == 2) { // if entirely within frustrum discontinue culling
    displayOctTreeAllCulled(node, center, size);
  }

  double ccenter[3];
  bitunion *children;
  bitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        // check if leaf is visible
        if ( QuadInFrustrum(ccenter[0], ccenter[1], ccenter[2], size, size, size) ) {
          pointrep *points = children->points;
          unsigned int length = points[0].length;
          double *point = &(points[1].v);  // first point
          glBegin(GL_POINTS);
          for(unsigned int iterator = 0; iterator < length; iterator++ ) {
            glVertex3f( point[0], point[1], point[2]);
            point+=3;
          }
          glEnd();
        }
      } else { // recurse
        displayOctTreeAllCulled( children->node, ccenter, size/2.0);
      }
      ++children; // next child
    }
  }

}

/**
 * Displays octree. Tries to display only targetpts points, but will usually display four times as much
 *
 */
void Show_BOctTree::displayOctTree(long targetpts, bitoct &node, double *center, double size) {
  if (targetpts <= 0) return; // no need to display anything

  double ccenter[3];
  bitunion *children;
  bitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        // check if leaf is visible
        if ( QuadInFrustrum(ccenter[0], ccenter[1], ccenter[2], size, size, size) ) {
          pointrep *points = children->points;
          unsigned int length = points[0].length;
          double *point = &(points[1].v);  // first point
          glBegin(GL_POINTS);
          if (length <= targetpts) {        // more points requested than possible, plot all
            for(unsigned int iterator = 0; iterator < length; iterator++ ) {
              glVertex3f( point[0], point[1], point[2]);
              point+=3;
            }
          } else {                         // select points to show
            // TODO smarter subselection of points here
            int each = 3 * (length/targetpts);
            for(unsigned int iterator = 0; iterator < targetpts; iterator++ ) {
              glVertex3f( point[0], point[1], point[2]);
              point += each;
            }
          }
          glEnd();
        }

      } else { // recurse
        childcenter(center, ccenter, size, i);  // childrens center
        displayOctTree(targetpts/POPCOUNT(node.valid), children->node, ccenter, size/2.0);
      }
      ++children; // next child
    }
  }
}


void Show_BOctTree::displayOctTreeCulled(long targetpts, bitoct &node, double *center, double size) {
  if (targetpts <= 0) return; // no need to display anything
  
  int res = QuadInFrustrum2(center[0], center[1], center[2], size, size, size);
  if (res==0) return;  // culled do not continue with this branch of the tree
  
  if (res == 2) { // if entirely within frustrum discontinue culling
    displayOctTree(targetpts, node, center, size);
  }

  double ccenter[3];
  bitunion *children;
  bitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        pointrep *points = children->points;
        unsigned int length = points[0].length;
        double *point = &(points[1].v);  // first point
        glBegin(GL_POINTS);
        if (length <= targetpts) {        // more points requested than possible, plot all
          for(unsigned int iterator = 0; iterator < length; iterator++ ) {
            glVertex3f( point[0], point[1], point[2]);
            point+=3;
          }
        } else {                         // select points to show
          // TODO smarter subselection of points here
          int each = 3 * (length/targetpts);
          for(unsigned int iterator = 0; iterator < targetpts; iterator++ ) {
            glVertex3f( point[0], point[1], point[2]);
            point += each;
          }
        }
        glEnd();

      } else { // recurse
        displayOctTreeCulled(targetpts/POPCOUNT(node.valid), children->node, ccenter, size/2.0);
      }
      ++children; // next child
    }
  }
}
