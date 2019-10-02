#ifndef QUADTREE_H
#define QUADTREE_H

#include <fstream>
#include <iostream>
#include <vector>
#include <set>

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

   std::vector<std::set<double*> >& getClusters();

//   int  outputQuad(char *);
//   void getQuadTreePoints(vector<double> &quad_x,  vector<double> &quad_y,  vector<double> &quad_z);
//   void getQuadTreeColor(double &colorR,  double &colorG, double &colorB);

 private:

   QuadTree(std::vector<double*> &splitPoints, double center[2],
            double x_size, double y_size,std::set<double*> &n,
            std::set<double*> &e, std::set<double*> &s, std::set<double*> &w);

   int countPoints(double **pts, int n, double center[2],
                   double x_size, double y_size);

   int countPointsAndQueue(const std::vector<double*> &i_points,
               		   std::vector<double*> &points,
                           double center[2],
                           double x_size, double y_size,
                           std::set<double*> &n,
                           std::set<double*> &e,
                           std::set<double*> &s,
                           std::set<double*> &w);

   int countPointsAndQueue(double **pts, int ,
                           std::vector<double*> &points,
                           double center[2],
                           double x_size, double y_size,
                           std::set<double*> &no,
                           std::set<double*> &e,
                           std::set<double*> &s,
                           std::set<double*> &w);

  // int _OutputQuad(ofstream& quadout);
   std::vector<std::set<double * > > clusters;
   std::set<double *> north;
   std::set<double *> south;
   std::set<double *> east;
   std::set<double *> west;

   QuadTree *child[4];

   std::vector<double*> points;
   double center[2];
   double x_size, y_size;
   double size2;

   int leaf;

   static int anz;
   static double qSize;
   static double dist;

   static bool close(double *p, std::set<double*> &cluster);
   static bool close(double *p1, double *p2);
   static int where(double *p1, std::vector<std::set<double*> >& clusters);
   static std::vector<std::set<double*> >& clusterCells(std::vector<std::set<double*> > &clusters1,
                                              std::set<double*> border1,
                                              std::vector<std::set<double*> > &clusters2,
                                              std::set<double*> border2, std::vector<std::set<double*> >&cl);
//   static Plane *basePlane;
};

#endif









