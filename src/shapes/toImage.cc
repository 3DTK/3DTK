/*
 * toImage implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file 
 * @author Dorit Borrmann. Institute of Computer Science, University of Osnabrueck, Germany.
*/
#include <cfloat>
#include <fstream>
#ifndef _MSC_VER
#include <getopt.h>
#else
#include "XGetopt.h"
#endif

#include <iostream>
using std::ofstream;
using std::flush;
using std::cout;
using std::string;
using std::cerr;
using std::endl;
#include <errno.h>

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#include <windows.h>
#include <direct.h>
#else
#include <sys/stat.h>
#include <sys/types.h>
#include <strings.h>
#include <dlfcn.h>
#endif

#include "shapes/image.h"
#include "shapes/hough.h"
#include "shapes/ppair.h"
#include "shapes/quadtree.h"
#include <map>
#include "newmat/newmatap.h"
using namespace NEWMAT;
//#include "shapes/convexplane.h"

#define MAX_PARALLEL_ANGLE 10.0
#define MAX_EQUAL_DIST 10.0
#define MIN_PARALLEL_DIST 50.0
#define MIN_RIGHT_ANGLE 80.0

#define NUM_ITERS 1

void usage(char* prog) {
#ifndef _MSC_VER
  const string bold("\033[1m");
  const string normal("\033[m");
#else
  const string bold("");
  const string normal("");
#endif
  cout << endl
	  << bold << "USAGE " << normal << endl
	  << "   " << prog << " [options] directory" << endl << endl;
  cout << bold << "OPTIONS" << normal << endl

	  << bold << "  -f" << normal << " F, " << bold << "--format=" << normal << "F" << endl
	  << "         using shared library F for input" << endl
	  << "         (chose F from {uos, uos_map, uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply})" << endl
	  << endl
	  << bold << "  -m" << normal << " NR, " << bold << "--max=" << normal << "NR" << endl
	  << "         neglegt all data points with a distance larger than NR 'units'" << endl
	  << endl
	  << bold << "  -M" << normal << " NR, " << bold << "--min=" << normal << "NR" << endl
	  << "         neglegt all data points with a distance smaller than NR 'units'" << endl
	  << endl
	  << bold << "  -p" << normal << " P, " << bold << "--plane=" << normal << "P" << endl
	  << "         using algorithm P for plane detection" << endl
	  << "         (chose P from {rht, sht, pht, ppht, apht})" << endl
	  << endl
	  << bold << "  -r" << normal << " NR, " << bold << "--reduce=" << normal << "NR" << endl
	  << "         turns on octree based point reduction (voxel size=<NR>)" << endl
	  << endl
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << endl
	  << "         [ATTENTION: counting naturally starts with 0]" << endl
	  << endl
    	  << endl << endl;
  
  cout << bold << "EXAMPLES " << normal << endl
	  << "   " << prog << " -m 500 -r 5 dat" << endl
	  << "   " << prog << " --max=5000 -r 10.2 dat" << endl
	  << "   " << prog << " -s 2 -e 10 -r dat" << endl << endl;
  exit(1);

}

int parseArgs(int argc, char **argv, string &dir, double &red, int &start, int
  &maxDist, int&minDist, int &octree, IOType &type, bool &quiet) {

  bool reduced = false;
  int  c;
  // from unistd.h:
  extern char *optarg;
  extern int optind;

  /* options descriptor */
  // 0: no arguments, 1: required argument, 2: optional argument
  static struct option longopts[] = {
    { "format",          required_argument,   0,  'f' },  
    { "max",             required_argument,   0,  'm' },
    { "min",             required_argument,   0,  'M' },
    { "start",           required_argument,   0,  's' },
    { "reduce",          required_argument,   0,  'r' },
    { "quit",            no_argument,         0,  'q' },
    { "octree",          optional_argument,   0,  'O' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  cout << endl;
  while ((c = getopt_long(argc, argv, "f:r:s:e:m:M:p:Oq", longopts, NULL)) != -1) 
  switch (c)
	 {
	 case 'r':
	   red = atof(optarg);
     reduced = true;
	   break;
	 case 's':
	   start = atoi(optarg);
	   if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
	   break;
	 case 'f':
     try {
       type = formatname_to_io_type(optarg);
     } catch (...) { // runtime_error
       cerr << "Format " << optarg << " unknown." << endl;
       abort();
     }
     break;
	 case 'q':
     quiet = true;
     break;
   case 'm':
	   maxDist = atoi(optarg);
	   break;
	 case 'O':
     if (optarg) {
       octree = atoi(optarg);
     } else {
       octree = 1;
     }
	   break;
	 case 'M':
	   minDist = atoi(optarg);
	   break;
   case '?':
	   usage(argv[0]);
	   return 1;
      default:
	   abort ();
      }

  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***" << endl;
    usage(argv[0]);
  }
  dir = argv[optind];

#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif

  return 0;
}

void getPPairs(vector<ConvexPlane*> &allPlanes, PPair **ppairs) {
  for (unsigned int i = 0; i < allPlanes.size(); i++) { 
    for (unsigned int j = i+1; j < allPlanes.size(); j++) { 
      PPair pair(allPlanes[i], allPlanes[j]); 
      // parallel or equal planes 
      if (deg(pair.angle) < MAX_PARALLEL_ANGLE) { 
        // planes are equal 
        if (pair.plane_distance < MAX_EQUAL_DIST) { 
          pair.type = PPair::EQUAL; 
        // planes are parallel 
        } else {//if (pair.plane_distance > MIN_PARALLEL_DIST) { 
          pair.type = PPair::PARALLEL; 
        // planes are nothing 
        }  
        // plane is perpendicular 
      } else if (deg(pair.angle) > MIN_RIGHT_ANGLE) { 
        pair.type = PPair::PERPENDICULAR; 
        // planes are nothing in particular 
      } 
      ppairs[i][j] = pair; 
      ppairs[j][i] = pair; 

      } 
    } 
  
}
void getZHK(unsigned int length, bool **graph, vector<set<int> > &zhks ) {
  set<int > unused;

  for (unsigned int i = 0; i < length; i++) unused.insert(i);

  // as long as we have nodes cluster 
  while(!unused.empty()) {
    // start new cluster
    int index = *unused.begin();
    unused.erase(index);
    set<int> zhk;
    zhk.insert(index);

    // now look for all connected nodes 
    vector<int> unvisited;
    unvisited.push_back(index);

    while (!unvisited.empty()) {
      int index = *unvisited.begin();
      unvisited.erase(unvisited.begin());

      for (unsigned int i = 0; i < length; i++) {
        if (i != index && graph[index][i] && zhk.count(i) == 0 ) {
          zhk.insert(i);
          unvisited.push_back(i);
          unused.erase(i);
        }
      }
    }
    zhks.push_back(zhk);
  }
}

void flip( const double *normal , double *n  ) {
  // project to the other side
  double k[3];
  n[0] = normal[0];
  n[1] = normal[1];
  n[2] = 1;
  toKartesian(n, k);
  k[0] *= -1;
  k[1] *= -1;
  k[2] *= -1;
  toPolar(k, n);
}
bool parallel(set<double*> &p1, set<double*> &p2, double min_angle) {
  for (set<double *>::iterator it = p1.begin(); 
      it != p1.end(); it++) {
    for (set<double *>::iterator jt = p2.begin(); 
        jt != p2.end(); jt++) {
      double *a = *it;
      double *b = *jt;
      //if ( polardist(a,b) > ( rad(180) - min_angle ) )
      double flipped[3];
      flip(a, flipped);
      if (polardist(a,b) < min_angle || polardist(flipped, b) < min_angle)
      {
        return true;
      }
    }
  }
  return false;
}
// TODO no idea what this does
/*
ColumnVector calcRotRhos(vector<ConvexPlane*> &planes) {
  unsigned int n = planes.size();
  double xbar[3];
  double *normal;
  double nx;

  ColumnVector A(n+3);
  Matrix B(n+3,n+3);
  B = 0;
  A = 0;
  // fill B and A

  // for all planes
  for (unsigned int i = 0; i < n; i++) {
    vector<Point> points = planes[i]->points;
    normal = planes[i]->n;
    // for all x_j in plane i
    for (unsigned int j = 0; j < points.size(); j++) {
      Point p = points[j];
      xbar[0] = normal[1] * p.z - normal[2]*p.y;
      xbar[1] = normal[2] * p.x - normal[0]*p.z;
      xbar[2] = normal[0] * p.y - normal[1]*p.x;
      nx = normal[0]*p.x + normal[1]*p.y + normal[2]*p.z;
      
      A(1) += nx * xbar[0];
      A(2) += nx * xbar[1];
      A(3) += nx * xbar[2];
      A(i+1 + 3) -= nx;
  
      // upper left 3x3 matrix of B
      B(1, 1) += xbar[0]*xbar[0];
      B(2, 2) += xbar[1]*xbar[1];
      B(3, 3) += xbar[2]*xbar[2];
      
      B(1,2) += xbar[0]*xbar[1];
      B(2,1) += xbar[0]*xbar[1];
      
      B(1,3) += xbar[0]*xbar[2];
      B(3,1) += xbar[0]*xbar[2];
      
      B(2,3) += xbar[1]*xbar[2];
      B(3,2) += xbar[1]*xbar[2];

      B(1, i+1 +3) -= xbar[0];
      B(i+1 +3, 1) -= xbar[0];

      B(2, i+1 +3) -= xbar[1];
      B(i+1 +3, 2) -= xbar[1];
      
      B(3, i+1 +3) -= xbar[2];
      B(i+1 +3, 3) -= xbar[2];
    }

    B(i+1 +3, i+1 +3) = points.size();
  }

  // calc result

//  cout << "A: " << endl << A << endl;
//  cout << "B: " << endl << B << endl;

  ColumnVector X = B.i() * A;

//  cout << "X: " << endl << X << endl;


  return X;
}*/
void join(vector<set<double*> > &cps, double min_angle, vector<set<double*> > &n_cps) {
  bool **par = new bool*[cps.size()];
  for (unsigned int i = 0; i< cps.size(); i++) {
    par[i] = new bool[cps.size()];
  }

  for (unsigned int i = 0; i < cps.size(); i++) {
    par[i][i] = false;
    for (unsigned int j = i+1; j < cps.size(); j++) {
      if (parallel(cps[i], cps[j], min_angle) ) {
        par[i][j] = true;
        par[j][i] = true;
      }
      else {
        par[i][j] = false;
        par[j][i] = false;
      }
    }
  }
  vector<set<int> > zhks; 
  getZHK(cps.size(), par, zhks);
 
  for (unsigned int i = 0; i < zhks.size(); i++) {
    set<double*> zhk;
    n_cps.push_back(zhk);
    for (set<int>::iterator it = zhks[i].begin(); it != zhks[i].end(); it++) {
      n_cps[i].insert( cps[*it].begin(), cps[*it].end() );
    }
  }

  for (unsigned int i = 0; i< cps.size(); i++) {
    delete[] par[i];
  }
  delete[] par;
}

void getModel(vector<ConvexPlane*> &model, vector<ConvexPlane*> &allPlanes) {
  int planes = allPlanes.size();//588;
  double min_angle = rad(MAX_PARALLEL_ANGLE);
  double perp_angle = rad(MIN_RIGHT_ANGLE);
  int min_size = 0;

  // polar coordinates of plane
  vector<double *> norms;
  for (unsigned int i = 0; i < allPlanes.size(); i ++) {;
    double *norm = new double[5]; 
    toPolar(allPlanes[i]->n, norm);     // 0: phi, 1:theta 
    norm[2] = allPlanes[i]->pointsize;  // number of points
    norm[3] = i;                        // index
    norm[4] = allPlanes[i]->rho;        // rho
    norms.push_back(norm);
  }

  // copy of polar coordinate representation
  double **normals = new double*[planes];//[planes][4];
  for (int j = 0; j < planes; j++) {
    normals[j] = norms[j];
  }

  // all normals in Quadtree and cluster
  QuadTree tree(normals, planes , 0.40, min_angle);
	vector<set<double *> > cpsold;
  cpsold = tree.getClusters();
	
  //join similar normals
  vector<set<double *> > cps;
  join(cpsold, min_angle, cps);
  
  // Filtering
  for (unsigned int i = 0; i < cps.size(); i++) {
cout << "Cluster " << i << ": " << cps[i].size();
    set<double *>::iterator it = cps[i].begin(); 
    double *rep = *it;
    // for each normal in this cluster
    while ( it != cps[i].end()) {
      double *normal = (*it);
      double flipped[3];
      flip(normal, flipped);
      // check wether the flipped one is closer
      if (polardist(rep, flipped) < polardist(rep, normal) ) {
        normal[0] = flipped[0];
        normal[1] = flipped[1];
      }
cout << "   " << normal[3] << " normal: " << normal[0] << " " << normal[1] << endl;
      it++;
    }
  }
  
  double **cnorms = new double*[cps.size()];
  bool **cnrels = new bool*[cps.size()];
  int *normpts = new int[cps.size()];
  
cout << "Center normals.." << endl;
  // Calculate the centerd normals
  for (unsigned int i = 0; i < cps.size(); i++) {
cout << "Cluster " << i << ":" << endl;
    cnorms[i] = new double[3];
    cnrels[i] = new bool[cps.size()];
    cnorms[i][0] = 0;
    cnorms[i][1] = 0;
    cnorms[i][2] = 0;
    
    int sumpoints = 0;
    for (set<double *>::iterator it = cps[i].begin(); 
        it != cps[i].end(); it++) {
      double *normal = (*it);
      double k[3], n[3];
      n[0] = normal[0];
      n[1] = normal[1];
      n[2] = 1;
      toKartesian(n, k);
      cnorms[i][0] += k[0] * normal[2];
      cnorms[i][1] += k[1] * normal[2];
      cnorms[i][2] += k[2] * normal[2];
      sumpoints += normal[2];
    }
    normpts[i] = sumpoints;
    cnorms[i][0] /= sumpoints;
    cnorms[i][1] /= sumpoints;
    cnorms[i][2] /= sumpoints;
    Normalize3(cnorms[i]);
    double p[3];
    toPolar(cnorms[i], p);
    cnorms[i][0] = p[0];
    cnorms[i][1] = p[1];

    cout << "   cnormal: " << cnorms[i][0] << " " << cnorms[i][1] << endl;
  }
  
  // Calculate relations between clusters
  for (unsigned int i = 0; i < cps.size(); i++) {
    cnrels[i][i] = false;
    for (unsigned int j = i+1; j < cps.size(); j++) {
      double angle = polardist(cnorms[i], cnorms[j]);
      cnrels[i][j] = angle > perp_angle && angle < (rad(180.0) - perp_angle);
      cnrels[j][i] = cnrels[i][j]; 
    }
  }

  // calculate clusters that are big enough
  vector<int> big_clusters;
  for (unsigned int i = 0; i < cps.size(); i++) {
    if (normpts[i] > min_size) {
      big_clusters.push_back(i);
    }
  }
  cout << "look for biggest cliques..." << endl;
  // Calculate biggest 3-cliques
  int ci, cj, ck, cpoints;
  ci = cj = ck = -1;
  cpoints = 0;
  for (vector<int>::iterator it = big_clusters.begin();
      it != big_clusters.end(); it++) {
    unsigned int i = *it;
    for (vector<int>::iterator jt = it+1;
        jt != big_clusters.end(); jt++) {
      unsigned int j = *jt;
      if (!cnrels[i][j]) continue;
      for (vector<int>::iterator kt = jt+1;
          kt != big_clusters.end(); kt++) {
        unsigned int k = *kt;
        if (cnrels[i][k] && cnrels[j][k]) {
          // candidate clique
          int sum = normpts[i] + normpts[j] + normpts[k];
          if (sum > cpoints) {
            cpoints = sum;
            ci = i;
            cj = j;
            ck = k;
            cout << i << " " << j << " " << k << " " << sum <<  endl;
          }
        }
      }
    }
  }
  if (ci == -1 || cj == -1 || ck == -1) {
    cout << "No clique could be found!" << endl;
    return;
  }

  // Estimate new normals
  double n[3];
  ConvexPlane p1(cnorms[ci]);
  p1.project(cnorms[cj], n);
  ConvexPlane p2(n);
  Cross(p1.n, p2.n, n);
  ConvexPlane p3(n);

  // map from allPlane index to ci, cj or ck
  map<int, int> idxTotype;
  int idx = 0;
cout << "hmm " << endl;
  vector<ConvexPlane*> numerical_planes;
  for (set<double *>::iterator it = cps[ci].begin(); it != cps[ci].end(); it++) {
    double *normal = (*it);
    int index = (int)normal[3];
    ConvexPlane *plane = allPlanes[index];
    /*
    plane->n[0] = p1.n[0];
    plane->n[1] = p1.n[1];
    plane->n[2] = p1.n[2];
    */
    model.push_back(plane);

    cout << "for ci = " << ci << " plane index " << idx << " ap index " << index << endl;
    idxTotype[idx++] = ci;
  }
  for (set<double *>::iterator it = cps[cj].begin(); it != cps[cj].end(); it++) {
    double *normal = (*it);
    int index = (int)normal[3];
    ConvexPlane *plane = allPlanes[index];
    /*
    plane->n[0] = p2.n[0];
    plane->n[1] = p2.n[1];
    plane->n[2] = p2.n[2];
    */
    model.push_back(plane);
    
    cout << "for cj = " << cj << " plane index " << idx << " ap index " << index << endl;
    idxTotype[idx++] = cj;
  }
  for (set<double *>::iterator it = cps[ck].begin(); it != cps[ck].end(); it++) {
    double *normal = (*it);
    int index = (int)normal[3];
    ConvexPlane *plane = allPlanes[index];
    /*
    plane->n[0] = p3.n[0];
    plane->n[1] = p3.n[1];
    plane->n[2] = p3.n[2];
    */
    model.push_back(plane);
    
    cout << "for ck = " << ck << " plane index " << idx << " ap index " << index << endl;
    idxTotype[idx++] = ck;
  }
  
  /*
  int MAX_IT = 100;
//  int MAX_IT = 10000;
  int it_nr = 0;
  ColumnVector X;
  double MIN_ERR = 0.0005;
//  double MIN_ERR = 0.0000000000000000000001;
  double error = 0.0;
  do {
    //cout << it_nr << endl;
    it_nr++;
    // calc rotation and rho's
    X = calcRotRhos(numerical_planes);
    // apply rotation to normals
    double rPos[3] = {0.0, 0.0, 0.0};
    double rPosTheta[3];
    rPosTheta[0] = -X(1);
    rPosTheta[1] = -X(2);
    rPosTheta[2] = -X(3);
    error = fabs (X(1)) + fabs(X(2)) + fabs(X(3)) ;
//    cout << rPosTheta[0] << " " << rPosTheta[1] << " " <<rPosTheta[2] << " " << endl;
    double alignxf[16];
    EulerToMatrix4(rPos, rPosTheta, alignxf);
//    cout << alignxf << endl;
    
    for(unsigned int j = 0; j < numerical_planes.size(); j++) {
      Point normal;
      normal.x = numerical_planes[j]->n[0];
      normal.y = numerical_planes[j]->n[1];
      normal.z = numerical_planes[j]->n[2];
//      cout << "NP " << j << " " << normal.x << " " << normal.y << " " << normal.z << endl;
      normal.transform(alignxf);
//      cout <<"to" << endl;
//      cout << normal.x << " " << normal.y << " " << normal.z << endl;
      numerical_planes[j]->n[0] = normal.x;
      numerical_planes[j]->n[1] = normal.y;
      numerical_planes[j]->n[2] = normal.z;
//      cout << "NP " << j << " " << normal.x << " " << normal.y << " " << normal.z << endl;
    }
    // repeat
  } while (error > MIN_ERR || it_nr > MAX_IT);
//  cout << "ERRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR" << endl;
  // apply rho's to planes
  for(unsigned int j = 0; j < numerical_planes.size(); j++) {
//    cout << j << " s rho is " << numerical_planes[j]->rho << endl; 
    numerical_planes[j]->rho = -X(j+1 +3);
//    cout << j << " s rho is " << numerical_planes[j]->rho << endl; 
  }
  
  bool **graph = new bool*[numerical_planes.size()];
  for (unsigned int i = 0; i < numerical_planes.size(); i++) {
    graph[i] = new bool[numerical_planes.size()];
  }
  

  for (unsigned int i = 0; i < numerical_planes.size(); i++) {
    graph[i][i] = false;
    for (unsigned int j = i+1; j < numerical_planes.size(); j++) {
      if( fabs(numerical_planes[i]->rho - numerical_planes[j]->rho) < MAX_EQUAL_DIST 
          && idxTotype[i] == idxTotype[j] ) { // check wether planes belong to the same type
        graph[i][j] = true;
        graph[j][i] = true;
      } else {
        graph[i][j] = false;
        graph[j][i] = false;
      }
    }
  }
   
  vector<set<int> > zhks;
  getZHK(numerical_planes.size(), graph, zhks);

  for (unsigned int i = 0; i < zhks.size(); i++) {
    vector<ConvexPlane*> pplane;
    for (set<int>::iterator it = zhks[i].begin(); it != zhks[i].end(); it++) {
      pplane.push_back(numerical_planes[*it]);
      cout << "zhk " << i << " p " << (*it) << endl; 
    }
    ConvexPlane *plane = new ConvexPlane(pplane);
    model.push_back(plane);
  }
  */
  //exit(0);
}



/**
 * Main function. The Hough Transform is called for the scan indicated as
 * argument.
 *
 */
int main(int argc, char **argv) 
{

  cout << "(c) Jacobs University Bremen, gGmbH, 2011" << endl << endl;
  
  if (argc <= 1) {
    usage(argv[0]);
  }
  // parsing the command line parameters
  // init, default values if not specified
  string dir;
  double red   = -1.0;
  int    start = 0;
  int    maxDist    = -1;
  int    minDist    = -1;
  int    octree     = 0;
  bool   quiet = false;
  IOType type    = UOS;
  
  cout << "Parse args" << endl;
  parseArgs(argc, argv, dir, red, start, maxDist, minDist, octree, type, quiet);
  Scan::dir = dir;
  int fileNr = start;
  string planedir = dir + "planes"; 

#ifdef _MSC_VER
  int success = mkdir(planedir.c_str());
#else
  int success = mkdir(planedir.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif
  if(success == 0) { 
    if(!quiet) {
      cout << "Writing planes to " << planedir << endl;
    }
  } else if(errno == EEXIST) {
    cout << "Directory " << planedir << " exists already.  CONTINUE" << endl; 
  } else {
    cerr << "Creating directory " << planedir << " failed" << endl;
    exit(1);
  }
  Scan::readScans(type, fileNr, fileNr, dir, maxDist, minDist, 0);
  // reduction filter for current scan!
  //Scan::allScans[0]->calcReducedPoints(red, octree);
  Scan::allScans[0]->toGlobal(red, octree);
  
  double id[16];
  M4identity(id);
  for(int i = 0; i < 10; i++) {
    Scan::allScans[0]->transform(id, Scan::ICP, 0);  // write end pose
  }
  //Hough hough(Scan::allScans[0], quiet);
  //hough.RHT();
  cout << "Hough done" << endl;

  // calculate relations between planes
  //vector<ConvexPlane*> * allPlanes = &hough.planes;
  /*
  PPair **p_graph;
  
  p_graph = new PPair*[allPlanes->size()];
  for (unsigned int i = 0; i < allPlanes->size(); i++) {
    p_graph[i] = new PPair[allPlanes->size()];
  }
  getPPairs(*allPlanes, p_graph);
  */
  // optimize relations
  //vector<ConvexPlane*> model;
  //getModel(model, *allPlanes);
  //cout << "GetModel done" << endl;
  
  //Hough houghdelete(Scan::allScans[0], quiet);
  cout << "start range image calculation" << endl;
  long starttime = GetCurrentTimeInMilliSec(); 
  int size;
  //double * const* points_red = houghdelete.deletePoints(*allPlanes, size); 
  //double * const* points_red = hough.getPoints(size); 
  cout << "Size " << size << endl;
  
  
  double * const* points_red = Scan::allScans[0]->get_points_reduced();
 // Image myImage(0.0, 360.0, 0.0, 180.0, 0.04, points_red, Scan::allScans[0]->get_points_red_size());

  Image myImage(0.0, 360.0, 0.0, 180.0, 0.08, Scan::allScans[0]->get_points());
  
  //Image myImage(90.0, 270.0, 40.0, 90.0, 1.0, points_red, size);
  
  starttime = (GetCurrentTimeInMilliSec() - starttime);
  cout << "Time for Constructor call: " << starttime << endl;

  starttime = GetCurrentTimeInMilliSec(); 
  
  int** regdat;
  int width = myImage.getWidth();
  regdat = new int*[width];
  for(int i = 0; i < width; i++) {
    regdat[i] = new int[myImage.getHeight()];
  }
  
  int** regdat2;
  regdat2 = new int*[width];
  for(int i = 0; i < width; i++) {
    regdat2[i] = new int[myImage.getHeight()];
  }

  //myImage.blobColor(05.0, regdat);
  //cout << "blobColor done" << endl;
  //myImage.printImage("blub.ppm", true);
  myImage.printImage("blub.ppm", false);
  float min = myImage.getMin();
  float max = myImage.getMax();
  float tenPercent = (max - min)*0.45;
  cout << "MinMax " << min << " " << max << " " << tenPercent << " " << (max - min) << " "<< (max - tenPercent) << endl;
  myImage.calcMarker(min + tenPercent, regdat);
  cout << "Marker done" << endl;
  int regions = myImage.cluster(5, regdat, regdat2);
  cout << "Nu kommen die Punkte" << endl;
  myImage.writeCenters(regions, regdat2, Scan::allScans[0]->get_points());
  cout << "dat woarn se" << endl;

  //myImage.blobColor(5, regdat2);
  //cout << "blobColor done" << endl;
  //myImage.printScans(regdat, points_red, Scan::allScans[0]->get_points_red_size());
  cout << "print Scans done" << endl;
  starttime = (GetCurrentTimeInMilliSec() - starttime);

  cout << "Time for whatever " << starttime << endl;
  delete Scan::allScans[0];
  Scan::allScans.clear();
}

