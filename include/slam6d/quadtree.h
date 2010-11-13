#ifndef QUADTREE_H
#define QUADTREE_H

#include <fstream>
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
using std::ios;
using std::ofstream;
#include <vector>
using std::vector;
#include <set>
using std::set;

#include "slam6d/globals.icc"            // globals constant definitions
//#include "defs.h"
//#include "plane.h"

const int    MIN_PTS_IN_BUCKET = -1;


/* +++++++++-------------++++++++++++
 * CLASS NAME
 *   QuadTree
 * DESCRIPTION
 *   A quadtree is used to convert an
 *   unbounded 3D plane to a 3D surface,
 *   such that it can be drawn to the
 *   screen.
 *   The code is similar to the octree
 *   class.
 * PARAMETERS
 *   --
 * +++++++++-------------++++++++++++ */

class QuadTree {
 public:

   //QuadTree(double **pts, int n, double, Plane *);
   QuadTree(double **pts, int n, double, double min_angle);
   ~QuadTree();

   vector<set<double*> >& getClusters(); 

//   int  outputQuad(char *);
//   void getQuadTreePoints(vector<double> &quad_x,  vector<double> &quad_y,  vector<double> &quad_z);
//   void getQuadTreeColor(double &colorR,  double &colorG, double &colorB);
  
 private:

   QuadTree(vector<double*> &splitPoints, double center[2], 
            double x_size, double y_size,set<double*> &n,
            set<double*> &e, set<double*> &s, set<double*> &w);
 
   int countPoints(double **pts, int n, double center[2], 
                   double x_size, double y_size);

   int countPointsAndQueue(const vector<double*> &i_points,
               		   vector<double*> &points,
                           double center[2], 
                           double x_size, double y_size,
                           set<double*> &n,
                           set<double*> &e,
                           set<double*> &s,
                           set<double*> &w);

   int countPointsAndQueue(double **pts, int ,
                           vector<double*> &points, 
                           double center[2], 
                           double x_size, double y_size,
                           set<double*> &no,
                           set<double*> &e,
                           set<double*> &s,
                           set<double*> &w);

  // int _OutputQuad(ofstream& quadout);
   vector<set<double * > > clusters;
   set<double *> north;
   set<double *> south;
   set<double *> east;
   set<double *> west;

   QuadTree *child[4];

   vector<double*> points;
   double center[2];
   double x_size, y_size;
   double size2;

   int leaf; 

   static int anz;
   static double qSize;
   static double dist;

   static bool close(double *p, set<double*> &cluster);
   static bool close(double *p1, double *p2);
   static int where(double *p1, vector<set<double*> >& clusters);
   static vector<set<double*> >& clusterCells(vector<set<double*> > &clusters1, 
                                              set<double*> border1,
                                              vector<set<double*> > &clusters2,
                                              set<double*> border2, vector<set<double*> >&cl);
//   static Plane *basePlane;
};

#endif









