/*
 * compacttree implementation
 *
 * Copyright (C) Jan Elseberg, Kai Lingemann, Jan Elseberg
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file 
 * @brief Efficient representation of an octree
 * @author Jan Elsberg. Automation Group, Jacobs University Bremen gGmbH, Germany. 
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include <stdio.h>

#include <vector>
using std::vector;
#include <deque>
using std::deque;
#include <set>
using std::set;
#include <list>
using std::list;
#include <iostream>
#include <fstream>
#include <string>

#include "slam6d/globals.icc"
#include "slam6d/point_type.h"

#include "show/scancolormanager.h"
#include "slam6d/Boctree.h"
#include "show/compacttree.h"
#include "show/colormanager.h"
#include "show/viewcull.h"

using namespace show;
compactTree::~compactTree(){
  delete alloc;

  delete[] mins;
  delete[] maxs;
} 


void compactTree::AllPoints( cbitoct &node, vector<double*> &vp, double center[3], double size) {
  double ccenter[3];
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        tshort *point = children->getPoints();
        lint length = children->getLength();

        for(unsigned int iterator = 0; iterator < length; iterator++ ) {
          double *p = new double[3];
          //cout << point[0] << " " << point[1] << " " << point[2] << endl; 
          for (unsigned int k = 0; k < 3; k++){
            p[k] = point[k] * precision + ccenter[k];
          }

          vp.push_back(p);
          point+=POINTDIM;
        }
      } else { // recurse
        AllPoints( children->node, vp, ccenter, size/2.0);
      }
      ++children; // next child
    }
  }
}



void compactTree::GetOctTreeCenter(vector<double*>&c, cbitoct &node, double *center, double size) {
  double ccenter[3];
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  for (unsigned char i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        double * cp = new double[POINTDIM];
        for (unsigned int iterator = 0; iterator < POINTDIM; iterator++) {
          cp[iterator] = ccenter[iterator];
        }
        c.push_back(cp);
      } else { // recurse
        GetOctTreeCenter(c, children->node, ccenter, size/2.0);
      }
      ++children; // next child
    }
  }
}
long compactTree::countNodes(cbitoct &node) {
  long result = 0;
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
        //   ++result;
      } else { // recurse
        result += countNodes(children->node) + 1;
      }
      ++children; // next child
    }
  }
  return result;
}

long compactTree::countLeaves(cbitoct &node) {
  long result = 0;
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
        lint nrpts = children->getLength();
        result += POINTDIM*nrpts + 1;
      } else { // recurse
        result += countLeaves(children->node);
      }
      ++children; // next child
    }
  }
  return result;
}


void compactTree::deletetNodes(cbitoct &node) {
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);
  bool haschildren = false;

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf
        tshort *points = children->getPoints();
        delete [] points;
        //          delete [] children->points;
      } else { // recurse
        deletetNodes(children->node);
      }
      ++children; // next child
      haschildren = true;
    }
  }
  // delete children
  if (haschildren) {
    cbitoct::getChildren(node, children);
    delete[] children;
  }
}


unsigned long compactTree::maxTargetPoints( cbitoct &node ) {
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  unsigned long max = 0;

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        lint length = children->getLength();
        if (length > max) max = length;
      } else { // recurse
        unsigned long tp = maxTargetPoints( children->node);
        if (tp > max) max = tp;
      }
      ++children; // next child
    }
  }

  return max*POPCOUNT(node.valid);
}

void compactTree::displayOctTreeAll( cbitoct &node, double *center, double size) {
  double ccenter[3];
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        tshort *point = children->getPoints();
        lint length = children->getLength();
        glBegin(GL_POINTS);
        for(unsigned int iterator = 0; iterator < length; iterator++ ) {
          if(cm) cm->setColor(point);
          //cout << "C " << point[1] << " " << cm << endl;
          //glVertex3f( point[0], point[1], point[2]);
          glVertex3f( point[0] * precision + ccenter[0], point[1] * precision + ccenter[1], point[2] * precision + ccenter[2]);
          point+=POINTDIM;
        }
        glEnd();
      } else { // recurse
        displayOctTreeAll( children->node, ccenter, size/2.0);
      }
      ++children; // next child
    }
  }
}

void compactTree::displayOctTreeAllCulled( cbitoct &node, double *center, double size ) {
  int res = CubeInFrustum2(center[0], center[1], center[2], size);
  if (res==0) return;  // culled do not continue with this branch of the tree

  if (res == 2) { // if entirely within frustrum discontinue culling
    displayOctTreeAll(node, center, size);
    return;
  }

  double ccenter[3];
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        // check if leaf is visible
        if ( CubeInFrustum(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {
          tshort *point = children->getPoints();
          lint length = children->getLength();
          glBegin(GL_POINTS);
          for(unsigned int iterator = 0; iterator < length; iterator++ ) {
            if(cm) cm->setColor(point);
            //glVertex3f( point[0], point[1], point[2]);
            glVertex3f( point[0] * precision + ccenter[0], point[1] * precision + ccenter[1], point[2] * precision + ccenter[2]);
            point+=POINTDIM;
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

void compactTree::displayOctTreeCulledLOD(long targetpts, cbitoct &node, double *center, double size ) {
  if (targetpts <= 0) return; // no need to display anything

  int res = CubeInFrustum2(center[0], center[1], center[2], size);
  if (res==0) return;  // culled do not continue with this branch of the tree

  if (res == 2) { // if entirely within frustrum discontinue culling
    displayOctTreeLOD(targetpts, node, center, size);
    return;
  }

  double ccenter[3];
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  unsigned short nc = POPCOUNT(node.valid);
  long newtargetpts = targetpts;
  if (nc > 0) {
    newtargetpts = newtargetpts/nc;
    if (newtargetpts <= 0 ) return;
  }

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        // check if leaf is visible
        if ( CubeInFrustum(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {
          tshort *point = children->getPoints();
          lint length = children->getLength();
          glBegin(GL_POINTS);
          if (length > 10 && !LOD(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {  // only a single pixel on screen only paint one point
            if(cm) cm->setColor(point);
            //glVertex3f( point[0], point[1], point[2]);
            glVertex3f( point[0] * precision + ccenter[0], point[1] * precision + ccenter[1], point[2] * precision + ccenter[2]);
          } else if (length <= newtargetpts) {        // more points requested than possible, plot all
            for(unsigned int iterator = 0; iterator < length; iterator++ ) {
              if(cm) cm->setColor(point);
              //glVertex3f( point[0], point[1], point[2]);
              glVertex3f( point[0] * precision + ccenter[0], point[1] * precision + ccenter[1], point[2] * precision + ccenter[2]);
              point+=POINTDIM;
            }
          } else {                         // select points to show
            // TODO smarter subselection of points here
            double each = (double)POINTDIM * (double)((double)length/(double)newtargetpts);
            tshort *p;
            int index;
            for(unsigned int iterator = 0; iterator < newtargetpts; iterator++ ) {
              index = (double)iterator * each;
              p = point + index - index%POINTDIM;
              if(cm) cm->setColor(p);
              //glVertex3f( p[0], p[1], p[2]);
              glVertex3f( p[0] * precision + ccenter[0], p[1] * precision + ccenter[1], p[2] * precision + ccenter[2]);
              //point += each;
            }
          }
          glEnd();
        }

      } else { // recurse
        displayOctTreeCulledLOD(newtargetpts, children->node, ccenter, size/2.0);
      }
      ++children; // next child
    }
  }
}

void compactTree::displayOctTreeLOD(long targetpts, cbitoct &node, double *center, double size ) {
  if (targetpts <= 0) return; // no need to display anything

  double ccenter[3];
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  unsigned short nc = POPCOUNT(node.valid);
  long newtargetpts = targetpts;
  if (nc > 0) {
    newtargetpts = newtargetpts/nc;
    if (newtargetpts <= 0 ) return;
  }


  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        tshort *point = children->getPoints();
        lint length = children->getLength();
        glBegin(GL_POINTS);
        /*          if (length > 10 && !LOD(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {  // only a single pixel on screen only paint one point
                    if(cm) cm->setColor(point);
        //glVertex3f( point[0], point[1], point[2]);
        glVertex3f( point[0] * precision + ccenter[0], point[1] * precision + ccenter[1], point[2] * precision + ccenter[2]);
        } else*/ if (length <= newtargetpts) {        // more points requested than possible, plot all
          for(unsigned int iterator = 0; iterator < length; iterator++ ) {
            if(cm) cm->setColor(point);
            //glVertex3f( point[0], point[1], point[2]);
            glVertex3f( point[0] * precision + ccenter[0], point[1] * precision + ccenter[1], point[2] * precision + ccenter[2]);
            point+=POINTDIM;
          }
        } else {                         // select points to show
          // TODO smarter subselection of points here
          double each = (double)POINTDIM * (double)((double)length/(double)newtargetpts);
          tshort *p;
          int index;
          for(unsigned int iterator = 0; iterator < newtargetpts; iterator++ ) {
            index = (double)iterator * each;
            p = point + index - index%POINTDIM;
            if(cm) cm->setColor(p);
            //glVertex3f( p[0], p[1], p[2]);
            glVertex3f( p[0] * precision + ccenter[0], p[1] * precision + ccenter[1], p[2] * precision + ccenter[2]);
            //point += each;
          }
        }
      glEnd();
      } else { // recurse
        displayOctTreeLOD(newtargetpts, children->node, ccenter, size/2.0);
      }
      ++children; // next child
    }
  }
}

void compactTree::displayOctTreeCulledLOD2(float ratio, cbitoct &node, double *center, double size ) {
  int res = CubeInFrustum2(center[0], center[1], center[2], size);
  if (res==0) return;  // culled do not continue with this branch of the tree

  if (res == 2) { // if entirely within frustrum discontinue culling
    displayOctTreeLOD2(ratio, node, center, size);
    return;
  }

  double ccenter[3];
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        // check if leaf is visible
        if ( CubeInFrustum(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {
          tshort *point = children->getPoints();
          lint length = children->getLength();
            
          int l = LOD2(ccenter[0], ccenter[1], ccenter[2], size/2.0);  // only a single pixel on screen only paint one point
          l = max((int)(l*l*ratio), 0);
          if (l != 0) {
            if ((int)length > l ) {
              double each = (double)POINTDIM * (double)((double)length/(double)l);
              tshort *p;
              int index;
              for(int iterator = 0; iterator < l; iterator++ ) {
                index = (double)iterator * each;
                p = point + index - index%POINTDIM;
                if(cm) cm->setColor(p);
                glVertex3f( p[0] * precision + ccenter[0], p[1] * precision + ccenter[1], p[2] * precision + ccenter[2]);
              }
            } else if ((int)length <= l) { 
              for(unsigned int iterator = 0; iterator < length; iterator++ ) {
                if(cm) cm->setColor(point);
                glVertex3f( point[0] * precision + ccenter[0], point[1] * precision + ccenter[1], point[2] * precision + ccenter[2]);
                point+=POINTDIM;
              }
            } else if (l == 1) {
                if(cm) cm->setColor(point);
                glVertex3f( point[0] * precision + ccenter[0], point[1] * precision + ccenter[1], point[2] * precision + ccenter[2]);
            }
          }
        }
      } else { // recurse
        int l = LOD2(ccenter[0], ccenter[1], ccenter[2], size/2.0);  // only a single pixel on screen only paint one point
        l = max((int)(l*l*ratio), 0);
        if (l > 0) {
          displayOctTreeCulledLOD2(ratio, children->node, ccenter, size/2.0);
        }
      }
      ++children; // next child
    }
  }
}

void compactTree::displayOctTreeLOD2(float ratio, cbitoct &node, double *center, double size ) {
  double ccenter[3];
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);


  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf get center
        tshort *point = children->getPoints();
        lint length = children->getLength();
            
        int l = LOD2(ccenter[0], ccenter[1], ccenter[2], size/2.0);  // only a single pixel on screen only paint one point
        l = max((int)(l*l*ratio), 0);
        if (l > 1) {
          if ((int)length > l ) {
            double each = (double)POINTDIM * (double)((double)length/(double)l);
            tshort *p;
            int index;
            for(int iterator = 0; iterator < l; iterator++ ) {
              index = (double)iterator * each;
              p = point + index - index%POINTDIM;
              if(cm) cm->setColor(p);
              glVertex3f( p[0] * precision + ccenter[0], p[1] * precision + ccenter[1], p[2] * precision + ccenter[2]);
            }
          } else if ((int)length <= l) { 
            for(unsigned int iterator = 0; iterator < length; iterator++ ) {
              if(cm) cm->setColor(point);
              glVertex3f( point[0] * precision + ccenter[0], point[1] * precision + ccenter[1], point[2] * precision + ccenter[2]);
              point+=POINTDIM;
            }
          }
        } else {
          if(cm) cm->setColor(point);
          glVertex3f( point[0] * precision + ccenter[0], point[1] * precision + ccenter[1], point[2] * precision + ccenter[2]);
        }
      } else { // recurse
        int l = LOD2(ccenter[0], ccenter[1], ccenter[2], size/2.0);  // only a single pixel on screen only paint one point
        l = max((int)(l*l*ratio), 0);
        if (l > 0) {
          displayOctTreeLOD2(ratio, children->node, ccenter, size/2.0);
        }
      }
      ++children; // next child
    }
  }
}


void compactTree::displayOctTreeCAllCulled( cbitoct &node, double *center, double size, double minsize ) {
  int res = CubeInFrustum2(center[0], center[1], center[2], size);
  if (res==0) return;  // culled do not continue with this branch of the tree

  if (res == 2) { // if entirely within frustrum discontinue culling
    displayOctTreeCAll(node, center, size, minsize);
    return;
  }

  double ccenter[3];
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf || minsize > size ) {   // if ith node is leaf get center
        // check if leaf is visible
        if ( CubeInFrustum(ccenter[0], ccenter[1], ccenter[2], size/2.0) ) {
          showCube(ccenter, size/2.0);
        }
      } else { // recurse
        displayOctTreeCAllCulled( children->node, ccenter, size/2.0, minsize);
      }
      ++children; // next child
    }
  }
}

void compactTree::displayOctTreeCAll( cbitoct &node, double *center, double size, double minsize ) {
  double ccenter[3];
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      childcenter(center, ccenter, size, i);  // childrens center
      if (  ( 1 << i ) & node.leaf || minsize > size ) {   // if ith node is leaf get center
        showCube(ccenter, size/2.0);
      } else { // recurse
        displayOctTreeCAll( children->node, ccenter, size/2.0, minsize);
      }
      ++children; // next child
    }
  }
}

void compactTree::showCube(double *center, double size) {
  glLineWidth(1.0);
  glBegin(GL_QUADS);      // draw a cube with 6 quads
  glColor3f(0.0f,1.0f,0.0f);      // Set The Color To Green
  glVertex3f(center[0] + size, center[1] + size, center[2] - size);
  glVertex3f(center[0] - size, center[1] + size, center[2] - size);
  glVertex3f(center[0] - size, center[1] + size, center[2] + size);
  glVertex3f(center[0] + size, center[1] + size, center[2] + size);
  glColor3f(1.0f,0.5f,0.0f);      // Set The Color To Orange

  glVertex3f(center[0] + size, center[1] - size, center[2] + size); 
  glVertex3f(center[0] - size, center[1] - size, center[2] + size);
  glVertex3f(center[0] - size, center[1] - size, center[2] - size);
  glVertex3f(center[0] + size, center[1] - size, center[2] - size);

  glColor3f(1.0f,0.0f,0.0f);      // Set The Color To Red
  glVertex3f(center[0] + size, center[1] + size, center[2] + size); 
  glVertex3f(center[0] - size, center[1] + size, center[2] + size);
  glVertex3f(center[0] - size, center[1] - size, center[2] + size);
  glVertex3f(center[0] + size, center[1] - size, center[2] + size);

  glColor3f(1.0f,1.0f,0.0f);      // Set The Color To Yellow
  glVertex3f(center[0] + size, center[1] - size, center[2] - size); 
  glVertex3f(center[0] - size, center[1] - size, center[2] - size);
  glVertex3f(center[0] - size, center[1] + size, center[2] - size);
  glVertex3f(center[0] + size, center[1] + size, center[2] - size);

  glColor3f(0.0f,0.0f,1.0f);      // Set The Color To Blue
  glVertex3f(center[0] - size, center[1] + size, center[2] + size); 
  glVertex3f(center[0] - size, center[1] + size, center[2] - size);
  glVertex3f(center[0] - size, center[1] - size, center[2] - size);
  glVertex3f(center[0] - size, center[1] - size, center[2] + size);

  glColor3f(1.0f,0.0f,1.0f);      // Set The Color To Violet
  glVertex3f(center[0] + size, center[1] + size, center[2] - size); 
  glVertex3f(center[0] + size, center[1] + size, center[2] + size);
  glVertex3f(center[0] + size, center[1] - size, center[2] + size);
  glVertex3f(center[0] + size, center[1] - size, center[2] - size);


  glEnd();

}




template <class T>
void compactTree::selectRay(vector<T *> &points) { 
  //selectRay(points, *root, center, size); 
}


void compactTree::childcenter(double *pcenter, double *ccenter, double size, unsigned char i) {
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
      ccenter[0] = pcenter[0] + size / 2.0;
      ccenter[1] = pcenter[1] + size / 2.0;
      ccenter[2] = pcenter[2] - size / 2.0;
      break;
    case 4:
      ccenter[0] = pcenter[0] - size / 2.0;
      ccenter[1] = pcenter[1] - size / 2.0;
      ccenter[2] = pcenter[2] + size / 2.0;
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


void compactTree::GetOctTreeCenter(vector<double*>&c) { GetOctTreeCenter(c, *root, center, size); }
void compactTree::AllPoints(vector<double *> &vp) { AllPoints(*compactTree::root, vp, center, size); }

long compactTree::countNodes() { return 1 + countNodes(*root); }
long compactTree::countLeaves() { return 1 + countLeaves(*root); }

void compactTree::setColorManager(ColorManager *_cm) { cm = _cm; }

void compactTree::drawLOD(float ratio) { 
    switch (current_lod_mode) {
      case 1:
        glBegin(GL_POINTS);
        displayOctTreeCulledLOD2(ratio , *root, center, size);
        glEnd();
        break;
      case 2:
        /*
#ifdef WITH_GLEE
        if (GLEE_ARB_point_parameters) {
          glPointParameterfARB(GL_POINT_SIZE_MIN_ARB, 1.0);
          glPointParameterfARB(GL_POINT_SIZE_MAX_ARB, 100000.0);
          GLfloat p[3] = {0.0, 0.0000, 0.0000005};
          glPointParameterfvARB(GL_POINT_DISTANCE_ATTENUATION_ARB, p);
          displayOctTreeCPAllCulled(*BOctTree<T>::root, BOctTree<T>::center, BOctTree<T>::size,  BOctTree<T>::size/  pow(2, min( (int)(ratio * BOctTree<T>::max_depth ), BOctTree<T>::max_depth - 3) ) ); 
          p[0] = 1.0;
          p[2] = 0.0;
          glPointParameterfvARB(GL_POINT_DISTANCE_ATTENUATION_ARB, p);
        }
#endif
*/
      //break;
      case 0:
        glBegin(GL_POINTS);
        displayOctTreeCulledLOD(maxtargetpoints * ratio, *root, center, size); 
        glEnd();
        break;
      default:
      break;
    }
}

void compactTree::draw() { 
  displayOctTreeAllCulled(*root, center, size); 
}

void compactTree::displayOctTree(double minsize ) { 
  displayOctTreeCAllCulled(*root, center, size, minsize); 
}

shortpointrep* compactTree::createPoints(lint length) {
  //shortpointrep *points = new shortpointrep[POINTDIM*length];
  shortpointrep *points = alloc->allocate<shortpointrep> (POINTDIM*length);
  return points;
}

void compactTree::deserialize(std::string filename)
{
  char buffer[sizeof(float) * 20];
  float *p = reinterpret_cast<float*>(buffer);

  std::ifstream file;
  file.open (filename.c_str(), std::ios::in | std::ios::binary);

  // read magic bits
  file.read(buffer, 2);
  if ( buffer[0] != 'X' || buffer[1] != 'T') {
    std::cerr << "Not an octree file!!" << endl;
    file.close();
    return;
  }

  // read header
  pointtype = PointType::deserialize(file);

  file.read(buffer, 5 * sizeof(float));
  voxelSize = p[0];
  center[0] = p[1];
  center[1] = p[2];
  center[2] = p[3];
  size = p[4];

  file.read(buffer, sizeof(int));
  int *ip = reinterpret_cast<int*>(buffer);
  POINTDIM = *ip;

  float *fmins = new float[POINTDIM];
  float *fmaxs = new float[POINTDIM];
  mins = new double[POINTDIM];
  maxs = new double[POINTDIM];

  file.read(reinterpret_cast<char*>(fmins), POINTDIM * sizeof(float));
  file.read(reinterpret_cast<char*>(fmaxs), POINTDIM * sizeof(float));

  for (unsigned int i = 0; i < POINTDIM; i++) {
    mins[i] = fmins[i];
    maxs[i] = fmaxs[i];
  }

  double vs = size;
  while (vs > voxelSize) {
    vs = vs * 0.5;
  }
//  precision = vs / 32768;  // 2^15
  precision = vs / TSHORT_MAXP1;  // 2^15


  // read root node
  //root = new cbitoct();
  root = alloc->allocate<cbitoct>();    
  deserialize(file, *root );
  file.close();
}




void compactTree::deserialize(std::ifstream &f, cbitoct &node) {
  char buffer[2];
  f.read(buffer, 2);
  node.valid = buffer[0];
  node.leaf = buffer[1];

  unsigned short n_children = POPCOUNT(node.valid);

  // create children
  //cbitunion<tshort> *children = new cbitunion<tshort>[n_children];
  cbitunion<tshort> *children = alloc->allocate<cbitunion<tshort> >(n_children);    
  cbitoct::link(node, children);

  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf read points 
        lint length;
        f.read(reinterpret_cast<char*>(&length), sizeof(lint));
        shortpointrep *points = createPoints(length); 

        f.read(reinterpret_cast<char*>(points), sizeof(shortpointrep) * length * POINTDIM); // read the points
        children->linkPoints(points, length); 
      } else {  // write child 
        deserialize(f, children->node);
      }
      ++children; // next child
    }
  }
}



void compactTree::serialize(std::string filename) {
  char buffer[sizeof(float) * 20];
  float *p = reinterpret_cast<float*>(buffer);

  std::ofstream file;
  file.open (filename.c_str(), std::ios::out | std::ios::binary);

  // write magic bits
  buffer[0] = 'X';
  buffer[1] = 'T';
  file.write(buffer, 2);

  // write header
  pointtype.serialize(file);

  p[0] = voxelSize;
  p[1] = center[0]; 
  p[2] = center[1]; 
  p[3] = center[2];
  p[4] = size;

  int *ip = reinterpret_cast<int*>(&(buffer[5 * sizeof(float)]));
  *ip = POINTDIM;

  file.write(buffer, 5 * sizeof(float) + sizeof(int));


  for (unsigned int i = 0; i < POINTDIM; i++) {
    p[i] = mins[i];
  }
  for (unsigned int i = 0; i < POINTDIM; i++) {
    p[i+POINTDIM] = maxs[i];
  }

  file.write(buffer, 2*POINTDIM * sizeof(float));

  // write root node
  serialize(file, *root);

  file.close();
}


void compactTree::serialize(std::ofstream &of, cbitoct &node) {
  char buffer[2];
  buffer[0] = node.valid;
  buffer[1] = node.leaf;
  of.write(buffer, 2);

  // write children
  cbitunion<tshort> *children;
  cbitoct::getChildren(node, children);
  for (short i = 0; i < 8; i++) {
    if (  ( 1 << i ) & node.valid ) {   // if ith node exists
      if (  ( 1 << i ) & node.leaf ) {   // if ith node is leaf write points 
        tshort *points = children->getPoints();
        lint length = children->getLength();

        of.write(reinterpret_cast<char*>(&length), sizeof(lint) );

        of.write(reinterpret_cast<char*>(points), POINTDIM*length*sizeof(tshort) );

      } else {  // write child 
        serialize(of, children->node);
      }
      ++children; // next child
    }
  }
}
