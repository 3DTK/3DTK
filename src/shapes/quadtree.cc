/*
 * quadtree implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#include "shapes/quadtree.h"

int QuadTree::anz;
double QuadTree::qSize;
double QuadTree::dist = 10.0;
//Plane *QuadTree::basePlane;
/*
QuadTree::QuadTree(double **pts, int n, double _qSize, Plane *_basePlane){
  basePlane = _basePlane;

  anz = 0;
  qSize = _qSize;
  double xmin = pts[0][0], xmax = pts[0][0];
  double ymin = pts[0][1], ymax = pts[0][1];
  for (int i=1; i < n; i++) {
    xmin = min(xmin, pts[i][0]);
    xmax = max(xmax, pts[i][0]);
    ymin = min(ymin, pts[i][1]);
    ymax = max(ymax, pts[i][1]);
  }
  center[0] = 0.5f * (xmin+xmax);
  center[1] = 0.5f * (ymin+ymax);

  double m = max(0.5f * (xmax-xmin), 0.5f * (ymax-ymin));
  int count;

  //set up values
  child[0] = child[1] = child[2] = child[3] = 0;
  leaf = 0;

  x_size    = m;
  y_size    = m;

  // calculate new buckets
  double newcenter[4][2];
  double x_sizeNew, y_sizeNew;

  x_sizeNew = x_size / 2.0;
  y_sizeNew = y_size / 2.0;

  newcenter[0][0] = center[0] - x_size / 2.0;
  newcenter[0][1] = center[1] - y_size / 2.0;

  newcenter[1][0] = center[0] + x_size / 2.0;
  newcenter[1][1] = center[1] - y_size / 2.0;

  newcenter[2][0] = center[0] - x_size / 2.0;
  newcenter[2][1] = center[1] + y_size / 2.0;

  newcenter[3][0] = center[0] + x_size / 2.0;
  newcenter[3][1] = center[1] + y_size / 2.0;
  
  // if bucket is _not_ approximately (nearly) empty don't build tree further
  // -----------------------------------------------------------------------
  for (int i = 0; i < 4; i++) {
    vector<double*> splitPoints;
    count = countPointsAndQueue(pts, n, splitPoints, 
                                newcenter[i], x_sizeNew, y_sizeNew);
    if (count > MIN_PTS_IN_BUCKET) 
      child[i] = new QuadTree(splitPoints,
                             newcenter[i], x_sizeNew, y_sizeNew);
    splitPoints.clear();
  }
}*/

bool QuadTree::close(double* p, double *p2) {
  double stheta = sin(p[0]) * sin(p2[0]);
  double myd2 = acos( 
      stheta * cos(p[1]) * cos(p2[1])
      + stheta * sin(p[1]) * sin(p2[1])
      + cos(p[0]) * cos(p2[0]));
  if (myd2 < dist) {
    return true;
  }
  return false;
/*
  double dx = p2[0] - p[0];
  double dy = p2[1] - p[1];
  if ( sqrt(dx*dx + dy*dy) < dist ) {
    return true;
  }
  return false;*/
}
 
bool QuadTree::close(double* p, set<double*> &cluster) {
  for(set<double*>::iterator it = cluster.begin(); it != cluster.end(); it++) {
    if ( close(p, (*it)) ) {
      return true;
    }
  }
  return false;
}
/*
bool QuadTree::close(double* p, set<double*> &cluster) {
  double dx,dy;
  for(set<double*>::iterator it = cluster.begin(); it != cluster.end(); it++) {
    dx = (*it)[0] - p[0];
    dy = (*it)[1] - p[1];
    if ( sqrt(dx*dx + dy*dy) < dist ) {
      return true;
    }
  }
  return false;
}*/
int QuadTree::where(double *p1, vector<set<double*> >& clusters) {
  for (unsigned int i = 0; i < clusters.size(); i++) {
    if (clusters[i].count(p1) > 0) return i;
  }
  // this shouldnt happen
  //cout << "VERY BAD THINGS HAPPENED!" <<endl;
  return -1;
}

vector<set<double*> >& QuadTree::clusterCells(vector<set<double*> > &clusters1, set<double*> border1, 
                           vector<set<double*> > &clusters2, set<double*> border2, vector< set<double*> > &clusters) {
  if (border1.size() <= 0 || border2.size() <= 0) {
    clusters.insert( clusters.end(), clusters1.begin(), clusters1.end());
    clusters.insert( clusters.end(), clusters2.begin(), clusters2.end());
    return clusters;
  }
  // some clusters might not have a border connection, remember those?
  set<int> non_used_clusters;
  for (unsigned int i = 0; i < clusters1.size(); i++) {
    non_used_clusters.insert( -1 -i);
  }
  for (unsigned int i = 0; i < clusters2.size(); i++) {
    non_used_clusters.insert( i);
  }

  // contains the edges of the correspondence graph
  // indices of cluster 1 are stored negated
  vector<int*> cgraph;
//cout << "bordersizes: " << border1.size() << " " <<  border2.size() << endl;
  for (set<double*>::iterator bit1 = border1.begin(); bit1 != border1.end(); bit1++) {
    for (set<double*>::iterator bit2 = border2.begin(); bit2 != border2.end(); bit2++) {
      if (close( *bit1, *bit2) ) {
        int *edge = new int[3];
        int w1,w2; 
        w1 = where(*bit1, clusters1);
        if (w1 == -1) {
          continue;
        }
        w2 = where(*bit2, clusters2);
        if (w2 == -1) {
          continue;
        }

        edge[0] = -1 - w1;
        edge[1] = w2; 
        // label the edge
        edge[2] = -1;
        cgraph.push_back(edge);
//        cout << "point to point " << edge[0] << " " << edge[1] << endl; 
      }
    }
  }
  //cout << "borders connected" << endl;
  vector< set<int> > index_clusters;
  int nr_cluster = 0;
  for (unsigned int i = 0; i < cgraph.size(); i++) {
    if (cgraph[i][2] != -1) {continue;}
    // unlabeled edge
    cgraph[i][2] = nr_cluster;
    set<int> index_cluster;
    index_cluster.insert(cgraph[i][0]);
    index_cluster.insert(cgraph[i][1]);
        non_used_clusters.erase(cgraph[i][0]);
        non_used_clusters.erase(cgraph[i][1]);
    for (unsigned int j = i + 1; j < cgraph.size(); j++) {
//    cout << j << " j " << cgraph.size() << endl;
      if (cgraph[j][2] != -1) {continue;}
      // if either of the nodes is already in my cluster, label edge and add
      // nodes
      if (index_cluster.count(cgraph[j][0]) > 0 
          || index_cluster.count(cgraph[j][1]) > 0 ) {
        index_cluster.insert(cgraph[j][0]);
        index_cluster.insert(cgraph[j][1]);

        non_used_clusters.erase(cgraph[j][0]);
        non_used_clusters.erase(cgraph[j][1]);
        cgraph[j][2] = nr_cluster;
      }
    }
    //non_used_clusters.erase(index_cluster.begin(), index_cluster.end());
    index_clusters.push_back(index_cluster);
    nr_cluster++;
  }
//cout << "cluster connections resolved" << endl;
  for (unsigned int i = 0; i < index_clusters.size(); i++) {
    set<double*> cluster;
    for (set<int>::iterator it = index_clusters[i].begin(); 
         it != index_clusters[i].end(); it++) {
      if (*it < 0 ) {
        int index = -1 - (*it);
        cluster.insert( clusters1[index].begin(), clusters1[index].end() );
      } else {
        cluster.insert( clusters2[*it].begin(), clusters2[*it].end() );
      }
    }
    clusters.push_back(cluster);
  }
//cout << "clusters joined. add remaining " << non_used_clusters.size() << endl;
  for (set<int>::iterator it = non_used_clusters.begin(); it != non_used_clusters.end(); it++) {
    set<double*> cluster;
    if (*it < 0 ) {
      int index = -1 - (*it);
      cluster.insert( clusters1[index].begin(), clusters1[index].end() );
    } else {
      cluster.insert( clusters2[*it].begin(), clusters2[*it].end() );
    }
    clusters.push_back(cluster);
  }
//cout << "done." << endl;
  // cleanup
  for (unsigned int i = 0; i < cgraph.size(); i++) {
    delete[] cgraph[i];
  }
  cgraph.clear();
//cout << "   cleanup done returning" << endl;
  return clusters;
//  return clusters1;
}

vector<set<double*> >& QuadTree::getClusters() {
  // if leaf node
  if (leaf == 1) {
    if (points.size() <= 0) return clusters;
//    cout << "leaf clustering ";
//    cout << center[0] << " " << center[1] << " " << points.size() << " " <<endl; 
    clusters.push_back(set<double*> ());
    clusters[0].insert(points[0]);

    for (unsigned int j = 1; j < points.size(); j++) {
      bool found_cluster = false;
      int oldindex = -1;
      for (unsigned int i = 0; i < clusters.size(); i++) {
        if ( close(points[j], clusters[i]) ) {
          if (!found_cluster) {
            clusters[i].insert(points[j]);
            found_cluster = true;
            oldindex = i;
            //break;
          } else {
//            cout << "    join cluster " << oldindex << " and " << i << endl;
            clusters[oldindex].insert(clusters[i].begin(), clusters[i].end());
            clusters.erase(clusters.begin() + i);
            i--;
            //clusters[i]
          }
        }
      }
      if (!found_cluster) { // havent found a cose enough cluster -> new cluster
        set<double*> cluster;
        cluster.insert(points[j]);
        clusters.push_back(cluster);
      }
    }
//    cout << "    this many cluster found in leaf  " << clusters.size() << endl;
    return clusters;
  }

 //cout << "my leafs: "<< child[0] << " " << child[1] << " " << child[2] << " " << child[3] << endl;
  vector< set<double *> > southclusters; 
            clusterCells(child[0]->getClusters(), child[0]->east,
                         child[1]->getClusters(), child[1]->west, southclusters);
//  cout << "northern cluste: " << endl; 
  vector< set<double *> > northclusters;
            clusterCells(child[2]->getClusters(), child[2]->east,
                         child[3]->getClusters(), child[3]->west, northclusters);
//  cout << "south: " << southclusters.size() << endl;
//  cout << "north: " << northclusters.size() << endl;

  // join borders
  set<double*> nborders01;
  nborders01.insert(child[0]->north.begin(), child[0]->north.end());
  nborders01.insert(child[1]->north.begin(), child[1]->north.end());
  
  set<double*> sborders23;
  sborders23.insert(child[2]->south.begin(), child[2]->south.end());
  sborders23.insert(child[3]->south.begin(), child[3]->south.end());

  clusters = clusterCells(northclusters, sborders23, southclusters, nborders01, clusters);
  if (clusters.size() > 0) {
  //cout << "node clustered " << clusters.size() << endl;
  //cout << center[0] << " " << center[1] << " " << points.size() << endl; 
  }

  return clusters;
}

QuadTree::QuadTree(double **pts, int n, double _qSize, double min_d){
  dist = min_d;
  anz = 0;
  qSize = _qSize;
  double xmin = pts[0][0], xmax = pts[0][0];
  double ymin = pts[0][1], ymax = pts[0][1];
  for (int i=1; i < n; i++) {
    xmin = min(xmin, pts[i][0]);
    xmax = max(xmax, pts[i][0]);
    ymin = min(ymin, pts[i][1]);
    ymax = max(ymax, pts[i][1]);
  }
  double eps = 0.00001;
  xmin -= eps;
  ymin -= eps;
  xmax += eps;
  ymax += eps;
  center[0] = 0.5 * (xmin+xmax);
  center[1] = 0.5 * (ymin+ymax);

  double m = max(0.5 * (xmax-xmin), 0.5 * (ymax-ymin));
  int count;

  //set up values
  child[0] = child[1] = child[2] = child[3] = 0;
  leaf = 0;

  x_size    = m;
  y_size    = m;

  // calculate new buckets
  double newcenter[4][2];
  double x_sizeNew, y_sizeNew;

  x_sizeNew = x_size / 2.0;
  y_sizeNew = y_size / 2.0;

  newcenter[0][0] = center[0] - x_size / 2.0;
  newcenter[0][1] = center[1] - y_size / 2.0;

  newcenter[1][0] = center[0] + x_size / 2.0;
  newcenter[1][1] = center[1] - y_size / 2.0;

  newcenter[2][0] = center[0] - x_size / 2.0;
  newcenter[2][1] = center[1] + y_size / 2.0;

  newcenter[3][0] = center[0] + x_size / 2.0;
  newcenter[3][1] = center[1] + y_size / 2.0;
  
  // if bucket is _not_ approximately (nearly) empty don't build tree further
  // -----------------------------------------------------------------------
  for (int i = 0; i < 4; i++) {
    vector<double*> splitPoints;
    set<double *> no,e,s,w;
    count = countPointsAndQueue(pts, n, splitPoints, 
                                newcenter[i], x_sizeNew, y_sizeNew, 
                                no, e, s, w);
    if (count > MIN_PTS_IN_BUCKET) 
      child[i] = new QuadTree(splitPoints,
                             newcenter[i], x_sizeNew, y_sizeNew, no,e,s,w);
    splitPoints.clear();
  }
}

QuadTree::QuadTree(vector<double*> &splitPoints, double _center[2], 
                 double _x_size, double _y_size, set<double*> &n,
                 set<double*> &e, set<double*> &s, set<double*> &w) {
  int count;
  /*
  this->north = n;
  this->east = e;
  this->south = s;
  this->west = w;
  */
  this->north.insert(n.begin(), n.end());
  this->south.insert(s.begin(), s.end());
  this->east.insert(e.begin(), e.end());
  this->west.insert(w.begin(), w.end());

  //set up values
  child[0] = child[1] = child[2] = child[3] = 0;
  leaf = 0;

  center[0] = _center[0];
  center[1] = _center[1];
  x_size    = _x_size;
  y_size    = _y_size;

  // if bucket is too small stop building tree
  // -----------------------------------------
  if ((y_size <= qSize)) {
    // copy points
    for (unsigned int i = 0; i < splitPoints.size(); i++) {
      points.push_back(splitPoints[i]);
    }
    leaf = 1;
    anz++;
    return; 
  }  

  // calculate new buckets
  double newcenter[4][3];
  double x_sizeNew, y_sizeNew;

  x_sizeNew = x_size / 2.0;
  y_sizeNew = y_size / 2.0;

  newcenter[0][0] = center[0] - x_size / 2.0;
  newcenter[0][1] = center[1] - y_size / 2.0;

  newcenter[1][0] = center[0] + x_size / 2.0;
  newcenter[1][1] = center[1] - y_size / 2.0;

  newcenter[2][0] = center[0] - x_size / 2.0;
  newcenter[2][1] = center[1] + y_size / 2.0;

  newcenter[3][0] = center[0] + x_size / 2.0;
  newcenter[3][1] = center[1] + y_size / 2.0;
  
  // if bucket is _not_ approximately (nearly) empty don't build tree further
  // ------------------------------------------------------------------------
  for (int i = 0; i < 4; i++) {
    vector<double*> new_splitPoints;
    set<double *> n,e,s,w;
    count = countPointsAndQueue(splitPoints, new_splitPoints, 
                                newcenter[i], x_sizeNew, y_sizeNew,
                                n, e, s, w);
    if (count > MIN_PTS_IN_BUCKET) {
      child[i] = new QuadTree(new_splitPoints,
                             newcenter[i], x_sizeNew, y_sizeNew, n,e,s,w);
    }
    new_splitPoints.clear();
  }
}

int QuadTree::countPointsAndQueue(const vector<double*> &i_points,
                                 vector<double*> &points,
                                 double center[2], 
                                 double x_size, double y_size,
                                 set<double*> &n,
                                 set<double*> &e,
                                 set<double*> &s,
                                 set<double*> &w)
{
  int count = 0;
  for (unsigned int i = 0; i < i_points.size(); i++) {
    if (fabs(i_points[i][0] - center[0]) <= x_size) {
      if (fabs(i_points[i][1] - center[1]) <= y_size) {
        count++;
        points.push_back(i_points[i]);
        double top = asin(sin(i_points[i][0]) * sin( fabs( (center[1] + y_size) - i_points[i][1]) ) );
        double bottom = asin(sin(i_points[i][0]) * sin( fabs( i_points[i][1] - (center[1] - y_size) ) ) );

        if ( fabs( (center[0] + x_size) - i_points[i][0]) < dist ) {
          e.insert(i_points[i]);
        }
        if ( fabs( i_points[i][0] - (center[0] - x_size) ) < dist ) {
          w.insert(i_points[i]);
        }
        if ( top < dist ) {
          n.insert(i_points[i]);
        }
        if ( bottom < dist ) {
          s.insert(i_points[i]);
        }
      }
    } 
  } 
  return count;
}

int QuadTree::countPointsAndQueue(double **pts, int n, 
                                 vector<double*> &points,
                                 double center[2], 
                                 double x_size,double y_size,
                                 set<double*> &no,
                                 set<double*> &e,
                                 set<double*> &s,
                                 set<double*> &w)
{

  int count = 0;
  for (int i=0; i < n; i++) {
    if (fabs(pts[i][0] - center[0]) <= x_size) {
      if (fabs(pts[i][1] - center[1]) <= y_size) {
        count++;
        double *p = pts[i];
        points.push_back(p);

        double top = asin(sin(p[0]) * sin( fabs( (center[1] + y_size) - p[1]) ) );
        double bottom = asin(sin(p[0]) * sin( fabs( p[1] - (center[1] - y_size) ) ) );
        
        if ( fabs( (center[0] + x_size) - p[0]) < dist ) {
          e.insert(p);
        }
        if ( fabs( p[0] - (center[0] - x_size) ) < dist ) {
          w.insert(p);
        }
        if ( top < dist ) {
          no.insert(p);
        }
        if ( bottom < dist ) {
          s.insert(p);
        }
      }
    } 
  } 
  return count;
}

int QuadTree::countPoints(double **pts, int n, double center[2], 
                         double x_size, double y_size)
{
  int count = 0;
  for (int i=0; i < n; i++) {
    if (fabs(pts[i][0] - center[0]) <= x_size) {
      if (fabs(pts[i][1] - center[1]) <= y_size) {
        count++;
      }
    } 
  } 
  return count;
}
/*
void QuadTree::getQuadTreePoints(vector<double> &quad_x,  vector<double> &quad_y,  vector<double> &quad_z)
{
  double x,y,z;
  
  if (leaf == 1) {
    basePlane->projTo3D(center[0] - x_size,
				    center[1] - y_size,
				    x, y, z);
    quad_x.push_back(x); 
    quad_y.push_back(y); 
    quad_z.push_back(z);
    basePlane->projTo3D(center[0] + x_size,
				    center[1] - y_size,
				    x, y, z);
    quad_x.push_back(x); 
    quad_y.push_back(y); 
    quad_z.push_back(z);
    basePlane->projTo3D(center[0] + x_size,
				    center[1] + y_size,
				    x, y, z);
    quad_x.push_back(x); 
    quad_y.push_back(y); 
    quad_z.push_back(z);
    basePlane->projTo3D(center[0] - x_size,
				    center[1] + y_size,
				    x, y, z);
    quad_x.push_back(x); 
    quad_y.push_back(y); 
    quad_z.push_back(z);
  } else {
    for( int i = 0; i < 4; i++){ 
	 if (child[i] != 0) {
	   child[i]->getQuadTreePoints(quad_x,  quad_y,  quad_z);
	 }
    }
  }
}
*/
/*
void QuadTree::getQuadTreeColor(double &colorR,  double &colorG, double &colorB)
{
    switch (basePlane->getColor()) {
	 case  1: colorR = 0.6; colorG = 0.0; colorB = 0.0; break;
      case  2: colorR = 0.0; colorG = 0.6; colorB = 0.0; break;
      case  3: colorR = 0.0; colorG = 0.0; colorB = 0.6; break;
      case  4: colorR = 0.6; colorG = 0.6; colorB = 0.0; break;
      case  5: colorR = 0.6; colorG = 0.0; colorB = 0.6; break;
      case  6: colorR = 0.0; colorG = 0.6; colorB = 0.6; break;
      case  7: colorR = 0.6; colorG = 0.3; colorB = 0.3; break;
      case  8: colorR = 0.3; colorG = 0.3; colorB = 0.3; break;
      case  9: colorR = 0.3; colorG = 0.3; colorB = 0.6; break;
      case 10: colorR = 0.6; colorG = 0.6; colorB = 0.6; break;
	 case 11: colorR = 0.1; colorG = 0.1; colorB = 0.6; break;
	 case 12: colorR = 0.1; colorG = 0.1; colorB = 0.1; break;
	 case 13: colorR = 0.1; colorG = 0.3; colorB = 0.1; break;
      case 14: colorR = 0.1; colorG = 0.1; colorB = 0.3; break;
      case 15: colorR = 0.3; colorG = 0.6; colorB = 0.3; break;
      case 16: colorR = 0.3; colorG = 0.3; colorB = 0.1; break; 
      default: colorR = 0.3; colorG = 0.1; colorB = 0.1;
    }
}
*/
/*
int QuadTree::outputQuad(char *dir)
{
  cout << "writing ";
  ofstream quadout;
  char fname[64];
  strcpy(fname,dir);strcat(fname, "/quad.dat");

  quadout.open(fname,ios::binary|ios::out|ios::app);
  if (!quadout) {
    cerr << "*** error: can't create file " << fname << " ***\n";
    exit(2);
  }
  int ret = _OutputQuad(quadout);
  quadout.close();
  cout << ret << " quads from quadtree." << endl;
  return ret;  
}

int QuadTree::_OutputQuad(ofstream &bBoxout)
{
  int ret = 0;
  if (leaf == 1) {

    double x,y,z;
    //    cout << endl;
    basePlane->projTo3D(center[0] - x_size,
				  center[1] - y_size,
				  x,y,z);
    //    cout << x << " " << y << " " << z << endl;
    bBoxout.write((char*)&x, sizeof(double));
    bBoxout.write((char*)&y, sizeof(double));
    bBoxout.write((char*)&z, sizeof(double));
    basePlane->projTo3D(center[0] - x_size,
				  center[1] + y_size,
				  x,y,z);
    //    cout << x << " " << y << " " << z << endl;
    bBoxout.write((char*)&x, sizeof(double));
    bBoxout.write((char*)&y, sizeof(double));
    bBoxout.write((char*)&z, sizeof(double));
    basePlane->projTo3D(center[0] + x_size,
				  center[1] + y_size,
				  x,y,z);
    //    cout << x << " " << y << " " << z << endl;
    bBoxout.write((char*)&x, sizeof(double));
    bBoxout.write((char*)&y, sizeof(double));
    bBoxout.write((char*)&z, sizeof(double));
    basePlane->projTo3D(center[0] + x_size,
				  center[1] - y_size,
				  x,y,z);
    //    cout << x << " " << y << " " << z << endl;
    bBoxout.write((char*)&x, sizeof(double));
    bBoxout.write((char*)&y, sizeof(double));
    bBoxout.write((char*)&z, sizeof(double));
    
    
//    float col = 0.0;
//    for (int i = 0; i < points.size(); i++) {
//      col = max(col, points[i][2]);
//    }
//
//    col = col / 3.0;
// 
    
    float col = 0; //!!!
    
    double gcolorR, gcolorG, gcolorB;
    getQuadTreeColor(gcolorR, gcolorG, gcolorB);

    float col_r = col + gcolorR;
    float col_g = col + gcolorG;
    float col_b = col + gcolorB;
    bBoxout.write((char*)&col_r, sizeof(float));
    bBoxout.write((char*)&col_g, sizeof(float));
    bBoxout.write((char*)&col_b, sizeof(float));

    return 1;
  }

  for( int i = 0; i < 4; i++){ 
    if (child[i] != 0) {
      ret += child[i]->_OutputQuad(bBoxout);
    }
  }
  return ret;
}
*/

QuadTree::~QuadTree(){
  for(int i = 0; i < 4; i++) {
    if (child[i] !=0) {
      delete child[i];
      child[i] = 0;
    }
  }
}









