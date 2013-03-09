/*
 * hough implementation
 *
 * Copyright (C) Dorit Borrmann, Jan Elseberg, Kai Lingemann, Andreas Nuechter, Remus Dumitru
 *
 * Released under the GPL version 3.
 *
 */

#include "newmat/newmatap.h"
using namespace NEWMAT;
#include "shapes/hough.h"
#include "slam6d/globals.icc"
#include <math.h>
#include <time.h>
#include <limits>
#include "shapes/quadtree.h"
#include <errno.h>
#include <iterator>

#ifdef _MSC_VER
#include <windows.h>
#include <direct.h>
#else
#include <dlfcn.h>
#endif 
#include <sys/stat.h>

/**
  * Hough Constructor.
  * Loads the Configfile, located in "bin/hough.cfg" and initializes the
  * accumulator. For details about the implemented Hough methods please read:
  *
  * Dorit Borrmann, Jan Elseberg, Kai Lingemann, and Andreas Nüchter. The 3D
  * Hough Transform for Plane Detection in Point Clouds - A Review and A new
  * Accumulator Design, Journal 3D Research, Springer, Volume 2, Number 2, 
  * March 2011.
  */

Hough::Hough(bool q, std::string configFile)
{
  quiet = q;

  // If the user has specified a configFile, load it
  if(configFile.size() > 0) {
    myConfigFileHough.LoadCfg(configFile.c_str());
    if (!quiet) {
	 std::cout << "Loaded Configfile" << std::endl;
      myConfigFileHough.ShowConfiguration();
    }
  }
}

Hough::Hough(Scan * GlobalScan, bool q, std::string configFile)
{
 
  quiet = q;
  if(configFile.size() > 0) {
    myConfigFileHough.LoadCfg(configFile.c_str());
    if(!quiet) {
	 std::cout << "Loaded Configfile" << std::endl;
      myConfigFileHough.ShowConfiguration();
    }
  }

  SetScan(GlobalScan);
}

void Hough::SetScan(Scan* scan)
{

  nrEntries = 0;
  maximum = false;

  planeCounter = 0;

  allPoints = new vector<Point>();

  DataXYZ points_red = scan->get("xyz reduced");
  for(unsigned int i = 0; i < points_red.size(); i++)
    {
    Point p(points_red[i]);
    allPoints->push_back(p);
    }

  switch(myConfigFileHough.Get_AccumulatorType()) {
    case 0:
      acc = new AccumulatorSimple(myConfigFileHough);
      break;
    case 1:
      acc = new AccumulatorBall(myConfigFileHough);
      break;
    case 2:
      acc = new AccumulatorCube(myConfigFileHough);
      break;
    case 3:
      acc = new AccumulatorBallI(myConfigFileHough);
      break;
  }
  srand(time(0)); // make the results actually random
  //srand(0); // make the results repeatable
}

/**
  * Desctructor.
  */

Hough::~Hough() {
  if(out.is_open()) {
    out.flush();
    out.close();
  }

  for(vector<ConvexPlane*>::iterator it = planes.begin(); 
      it != planes.end(); it++) {
      ConvexPlane* tmp = (*it);
      delete tmp;
  }
  delete acc;
  delete allPoints;
  
}

/**
  * Randomized Hough Transform
  */
int Hough::RHT() {

  if (!quiet) cout << "RHT" << endl;
  Point p1, p2, p3;
  double theta, phi, rho;
  int planeSize = 2000;

  unsigned int stop = (unsigned int)(allPoints->size()/100.0)*myConfigFileHough.Get_MinSizeAllPoints();
  int plane = 1;
  long start, end;
  start = GetCurrentTimeInMilliSec(); 
  int counter = 0;
  while( allPoints->size() > stop && 
          planes.size() < (unsigned int)myConfigFileHough.Get_MaxPlanes() &&
          counter < (int)myConfigFileHough.Get_TrashMax()) { 
    unsigned int pint = (int) (((*allPoints).size())*(rand()/(RAND_MAX+1.0)));

    p1 = (*allPoints)[pint];
    pint = (int) (((*allPoints).size())*(rand()/(RAND_MAX+1.0)));
    p2 = (*allPoints)[pint];
    pint = (int) (((*allPoints).size())*(rand()/(RAND_MAX+1.0)));
    p3 = (*allPoints)[pint];

    // check distance
    if(!distanceOK(p1, p2, p3)) continue;
    //cout << "Distance OK" << endl;
    // calculate Plane
    if(calculatePlane(p1, p2, p3, theta, phi, rho)) {
      // increment accumulator cell
      if(acc->accumulate(theta, phi, rho)) {
        end = GetCurrentTimeInMilliSec() - start;
        start = GetCurrentTimeInMilliSec();
        if (!quiet) cout << "Time for RHT " << plane << ": " << end << endl; 
        double * n = acc->getMax(rho, theta, phi);
	   if (!quiet) cout << rho << " " << theta << " " << phi << endl;
        planeSize = deletePoints(n, rho);
        delete[] n;
	   if (!quiet) cout << "Delete Points done " << plane << endl;
        if(planeSize < (int)myConfigFileHough.Get_MinPlaneSize()) counter++;
        end = GetCurrentTimeInMilliSec() - start;
        start = GetCurrentTimeInMilliSec();
        if(!quiet) cout << "Time for Polygonization " << plane << ": " << end << endl; 
        acc->resetAccumulator();
        plane++;
        if(!quiet) cout << "Planes " << planes.size() << endl;
      }

    }
  }
  /*
  vector<Point>::iterator itr = allPoints->begin();
 
  Point p;
  start = GetCurrentTimeInMilliSec(); 
  while(itr != allPoints->end()) {
    p = *(itr);
    cout << p.x << " " << p.y << " " << p.z << endl;
    itr++;
  } 
  */
  return (int)planes.size();
}

/**
 * Standard Hough Transform
 */
void Hough::SHT() {
  vector<Point>::iterator itr = allPoints->begin();
 
  Point p;
  long start, end;
  start = GetCurrentTimeInMilliSec(); 
  while(itr != allPoints->end()) {
    p = *(itr);
    acc->accumulate(p);
    itr++;
  } 
  end = GetCurrentTimeInMilliSec() - start;
  start = GetCurrentTimeInMilliSec();
  if (!quiet) cout << "Time for SHT: " << end << endl; 
  
  if(myConfigFileHough.Get_PeakWindow()) {
    acc->peakWindow(myConfigFileHough.Get_WindowSize());
  }

  multiset<int*, maxcompare>* maxlist = acc->getMax();
  multiset<int*, maxcompare>::iterator it = maxlist->begin();
  int threshold = ((*it)[0] * myConfigFileHough.Get_PlaneRatio());
  unsigned int stop = (int)(allPoints->size()/100.0)*myConfigFileHough.Get_MinSizeAllPoints();
  
  while(it != maxlist->end() && 
        stop < allPoints->size() && 
        planes.size() < (unsigned int)myConfigFileHough.Get_MaxPlanes() && 
        (*it)[0] > threshold) {
    int* tmp = (*it);
    double * polar = acc->getMax(tmp);
    deletePoints(polar, polar[3]); 
    delete[] polar;
    it++;
  } 
  if (!quiet) cout << "Time for Polygonization: " << end << endl; 
  
  it = maxlist->begin();
    for(;it != maxlist->end(); it++) {
      int* tmp = (*it);
      delete[] tmp;
  } 
  
  delete maxlist; 
}

/**
  * Function that returns all the remaining points in the allPoints vector.
  * @return points that do not lie on an already detected plane
  */
double * const* Hough::getPoints(int &size) {
  size = allPoints->size();
  double ** returnPoints = new double*[allPoints->size()];
  for(unsigned int i = 0; i < allPoints->size(); i++) {
    Point p = (*allPoints)[i];
    returnPoints[i] = new double[3];
    returnPoints[i][0] = p.x;
    returnPoints[i][1] = p.y;
    returnPoints[i][2] = p.z;
  }
  return returnPoints;
}

/**
  * Deletes all points that lie on a list of planes. 
  * @param model the list of planes
  * @param size the number of remaining points after delete operation
  * @return the remaining points
  */
double * const* Hough::deletePoints(vector<ConvexPlane*> &model, int &size) {
  for(unsigned int i = 0; i < model.size(); i++) {
    deletePoints(model[i]->n, model[i]->rho); 
  }
  double ** returnPoints = new double*[allPoints->size()];
  for(unsigned int i = 0; i < allPoints->size(); i++) {
    Point p = (*allPoints)[i];
    returnPoints[i] = new double[3];
    returnPoints[i][0] = p.x;
    returnPoints[i][1] = p.y;
    returnPoints[i][2] = p.z;
  }
  size = allPoints->size();
  return returnPoints;
}

/**
 * Probabilistic Hough Transform
 */

void Hough::PHT() {
  unsigned int stop =
  (int)(allPoints->size()/100.0)*myConfigFileHough.Get_MinSizeAllPoints();
  bool *voted = new bool[allPoints->size()];
  for(unsigned int i = 0; i < allPoints->size(); i++) {
    voted[i] = false;
  }

  unsigned int i = 0;
  while(i < stop && planes.size() < (unsigned int)myConfigFileHough.Get_MaxPlanes()) {
    unsigned int pint = (int) (((*allPoints).size())*(rand()/(RAND_MAX+1.0)));

    if(!voted[i]) {
      Point p = (*allPoints)[pint];
      acc->accumulate(p);
      i++;
    }
    
  }
  // List of Maxima
  if(myConfigFileHough.Get_PeakWindow()) {
    acc->peakWindow(myConfigFileHough.Get_WindowSize());
  }
  multiset<int*, maxcompare>* maxlist = acc->getMax();
//  cout << "Mean "  << acc->mean() << endl;
//  cout << "Variance "  << acc->variance() << endl;
  
  multiset<int*, maxcompare>::iterator it = maxlist->begin();
  int threshold = ((*it)[0] * myConfigFileHough.Get_PlaneRatio());
  while(it != maxlist->end() && 
        stop < allPoints->size() && 
        planes.size() < (unsigned int)myConfigFileHough.Get_MaxPlanes() && 
        (*it)[0] > threshold) {
     
    int* tmp = (*it);
    double * tmp2 = acc->getMax(tmp);
    deletePoints(tmp2, tmp2[3]);
    delete [] tmp2;
    it++;
  } 
  it = maxlist->begin();
    for(;it != maxlist->end(); it++) {
      int* tmp = (*it);
      delete[] tmp;
  } 
  
  delete maxlist; 
  delete[] voted;
}

/**
 * Progressive Probabilistic Hough Transform
 */

void Hough::PPHT() {
  unsigned int stop =
  (int)(allPoints->size()/100.0)*myConfigFileHough.Get_MinSizeAllPoints();
  while(stop < allPoints->size() && 
        planes.size() < (unsigned int)myConfigFileHough.Get_MaxPlanes()) {
    bool *voted = new bool[allPoints->size()];
    for(unsigned int i = 0; i < allPoints->size(); i++) {
      voted[i] = false;
    }
   
    unsigned int pint;
    do { 
      pint = (int) (((*allPoints).size())*(rand()/(RAND_MAX+1.0)));

      Point p = (*allPoints)[pint];
      if(!voted[pint]) {
        double * angles = acc->accumulateRet(p);
        if(angles[0] > -0.0001) {
          double * n = polar2normal(angles[1], angles[2]);
          deletePoints(n, angles[0]);
          acc->resetAccumulator();
          delete [] n;
        } else {
          voted[pint] = true;
        }
        delete [] angles;
      }

    } while(!voted[pint]);
   
	delete[] voted;
  }
}

/**
 * Adaptive Probabilistic Hough Transform
 */

void Hough::APHT() {

  int max = 0;
  int maxpos = 0;
  vector<int*> mergelist = vector<int*>(); 
  int stability[20] = {0}; 
  do {
  
    //randomly select points and perform HT for them
    vector<int*> altlist = mergelist; 
    mergelist = vector<int*>(); 
    multiset<int*,valuecompare> maxlist = multiset<int*,valuecompare>();
    for(int i = 0; i < 10; i++) {
      unsigned int pint = (int) (((*allPoints).size())*(rand()/(RAND_MAX+1.0)));
      Point p = (*allPoints)[pint];
      int * max = acc->accumulateAPHT(p);
      // store the maximum cell touched by the HT
      maxlist.insert(max);
    }
 
    // merge active with previous list
    multiset<int*,valuecompare>::iterator mi = maxlist.begin(); 
    vector<int*>::iterator ai = altlist.begin(); 

    int i = 0;
    while(i < 20) {
      bool mi_oder_ai = false;
      if(ai == altlist.end()) {
        if(mi == maxlist.end()) {
          break;
        } else {
          mi_oder_ai = true;
        }
      } else if(mi == maxlist.end()) {
        mi_oder_ai = false;
      } else if((*mi)[0] <= (*ai[0])) {
        mi_oder_ai = false;
      } else {
        mi_oder_ai = true;
      }
    
      int *tmp;
      if(mi_oder_ai) {
        tmp = (*mi);
        mi++;
      } else {
        tmp = (*ai);
        ai++;
      }

      bool insert = true;
      for(vector<int*>::iterator it = mergelist.begin(); it != mergelist.end();
    it++) {
        if(myConfigFileHough.Get_AccumulatorType() != 2) {
          if( sqrt((float)
            ((*it)[3] - (tmp[3])) * ((*it)[3] - (tmp[3])) +
            ((*it)[1] - (tmp[1])) * ((*it)[1] - (tmp[1])) +
            ((*it)[2] - (tmp[2])) * ((*it)[2] - (tmp[2])) ) < 3.0f) {
            insert = false;
            break;
          }
        } else { 
          if( sqrt((float)
            ((*it)[4] - (tmp[4])) * ((*it)[4] - (tmp[4])) +
            ((*it)[1] - (tmp[1])) * ((*it)[1] - (tmp[1])) +
            ((*it)[2] - (tmp[2])) * ((*it)[2] - (tmp[2])) +
            ((*it)[3] - (tmp[3])) * ((*it)[3] - (tmp[3])) ) < 3.0f) {
            insert = false;
            break;
          }
        }
      }
      if(insert) {
        mergelist.push_back(tmp);
        i++;
      }
    }
    // compare lists, calculate stability
    i = 0;
    for(unsigned int i = 1; i < altlist.size(); i++) {
      for(unsigned int j = 0; j < i; j++) {
        int * tmp1 = mergelist[j];
        bool treffer = false;
        for(unsigned int k = 0; k < i; k++) {
          int * tmp2 = altlist[k];
          if(myConfigFileHough.Get_AccumulatorType() != 2) {
            if( sqrt((float)
              ((tmp2)[3] - (tmp1[3])) * ((tmp2)[3] - (tmp1[3])) +
              ((tmp2)[1] - (tmp1[1])) * ((tmp2)[1] - (tmp1[1])) +
              ((tmp2)[2] - (tmp1[2])) * ((tmp2)[2] - (tmp1[2])) ) < 3.0f) {
              treffer = true;
              break;
            }
          } else { 
            if( sqrt((float)
              ((tmp2)[4] - (tmp1[4])) * ((tmp2)[4] - (tmp1[4])) +
              ((tmp2)[1] - (tmp1[1])) * ((tmp2)[1] - (tmp1[1])) +
              ((tmp2)[2] - (tmp1[2])) * ((tmp2)[2] - (tmp1[2])) +
              ((tmp2)[3] - (tmp1[3])) * ((tmp2)[3] - (tmp1[3])) ) < 3.0f) {
              treffer = true;
              break;
            }
          }
        }
        if(!treffer) {
          stability[i-1] = -1;
          break;
        }
      }
      stability[i-1]++;
    }
    for(int i = mergelist.size(); i < 20; i++) {
      stability[i] = 0;
    }
    // determine stability count, 
    // a set of n entries is considered to be stable, if all planes in the set
    // are the maximal entries of the mergelist in this iteration and have also
    // been the maximal entries in the previous iteration (the order in the set
    // does not count). The stability count for a number n is the number of
    // iterations for which the set of size n has been stable.
    // The maximum stability count is the stability count for the size n, that
    // is the maximum of all stability counts.
    max = 0;
    maxpos = 0;
    for(int i = 0; i < 20; i++) {
      if(stability[i] >= max) {
        max = stability[i];
        maxpos = i;
      }
    }
    // repeat until maximum stability count exceeds a threshold
  } while(max < (int)myConfigFileHough.Get_AccumulatorMax()
  && stability[myConfigFileHough.Get_MaxPlanes()] < 
  myConfigFileHough.Get_AccumulatorMax() * myConfigFileHough.Get_PlaneRatio()
  );
  if(stability[myConfigFileHough.Get_MaxPlanes()] >= myConfigFileHough.Get_AccumulatorMax() * myConfigFileHough.Get_PlaneRatio()) {
    maxpos = myConfigFileHough.Get_MaxPlanes();
  }
  
  for(int i = 0; i <= maxpos; i++) {
    double * n = acc->getMax(mergelist[i]); 
    deletePoints(n, n[3]);
   
    delete[] n;
  }
      
}

/**
  * Verifies the distance criterion needed for the RHT. The distance between the
  * three selected points may not exceed a maximal or fall below a minimal
  * distance. 
  */
bool Hough::distanceOK(Point p1, Point p2, Point p3) {
  // p1 - p2
  double distance = (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z); 
  if(sqrt(distance) < myConfigFileHough.Get_MinDist()) return false;
  if(sqrt(distance) > myConfigFileHough.Get_MaxDist()) return false;

  // p2 - p3
  distance = (p3.x - p2.x) * (p3.x - p2.x) + (p3.y - p2.y) * (p3.y - p2.y) + (p3.z - p2.z) * (p3.z - p2.z); 
  if(sqrt(distance) < myConfigFileHough.Get_MinDist()) return false;
  if(sqrt(distance) > myConfigFileHough.Get_MaxDist()) return false;

  // p3 - p1
  distance = (p1.x - p3.x) * (p1.x - p3.x) + (p1.y - p3.y) * (p1.y - p3.y) + (p1.z - p3.z) * (p1.z - p3.z); 
  if(sqrt(distance) < myConfigFileHough.Get_MinDist()) return false;
  if(sqrt(distance) > myConfigFileHough.Get_MaxDist()) return false;

  return true;
  
}

/**
  * Calculates the polar coordinates (rho, theta, phi) of the plane spanned by
  * three points p1, p2, and p3.
  */
bool Hough::calculatePlane(Point p1, Point p2, Point p3, double &theta, double &phi, double &rho) {

  double v1[3];
  double v2[3];
  double n[3];

  v1[0] = p3.x - p2.x;
  v1[1] = p3.y - p2.y;
  v1[2] = p3.z - p2.z;
  v2[0] = p1.x - p2.x;
  v2[1] = p1.y - p2.y;
  v2[2] = p1.z - p2.z;

  Cross(v1,v2,n);

  if(sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]) < 0.000000001) return false;

  Normalize3(n);

  rho = p1.x * n[0] + p1.y * n[1] + p1.z * n[2];
  
  if(rho < 0) {
    rho = -rho;
    for(int i = 0; i < 3; i++) {
      n[i] = -n[i];
    }
  }
  double polar[3];
  toPolar(n, polar);

  phi = polar[0];
  theta = polar[1];

  if((fabs(theta) < 0.001) && (fabs(phi) < 0.001) ) return false;
  return true;
}

/**
  * Improved function for point deletion.
  */
int Hough::deletePointsQuad(double * n, double rho) {

  if (!quiet) cout << allPoints->size() ;
  Normalize3(n);
  vector<Point> *nallPoints = new vector<Point>();

  vector<Point> planePoints;
  out << n[0] << " " << n[1] << " " << n[2] << " " << rho << " ";
  
  Point p;
  vector<Point>::iterator itr = allPoints->begin();
  
  while(itr != allPoints->end()) {
    p = *(itr);
    // if point close to plane, delete it
    double distance = p.x * n[0] + p.y * n[1] + p.z*n[2] - rho;
    if( fabs(distance) < myConfigFileHough.Get_MaxPointPlaneDist()) {
   
      planePoints.push_back(p);

    } 
   itr++;
  }
  double n2[4];

  // calculating the best fit plane
  calcPlane(planePoints, n2);
  out << n2[0] << " " << n2[1] << " " << n2[2] << " " << n2[3] << " " ;
  
  itr = allPoints->begin();
  planePoints.clear();  
 
  while(itr != allPoints->end()) {
    p = *(itr);
    // if point close to plane, delete it
    double distance = p.x * n2[0] + p.y * n2[1] + p.z*n2[2] - n2[3];
    if( fabs(distance) < myConfigFileHough.Get_MaxPointPlaneDist()) {
   
      planePoints.push_back(p);

    } else {
      nallPoints->push_back(p);
    }
   itr++;
  }
  delete allPoints;
  allPoints = nallPoints;
  
  if (!quiet) cout << "Planepoints " << planePoints.size() << endl;
  int nr_points = planePoints.size();
  // TODO Clustering
  double min_angle = rad(1.0);
  double _phi, _theta;
	double **pppoints;
	pppoints = new double*[nr_points];

  for (unsigned int i = 0; i < planePoints.size(); i++) {
    pppoints[i] = new double[3];

    Point p(planePoints[i]);
    double m[3];
    m[0] = p.x;
    m[1] = p.y;
    m[2] = p.z;

    Normalize3(m);
    _phi = acos(m[2]);
    double costheta = acos(m[0]/sin(_phi));
    double sintheta = m[1]/sin(_phi);
    double EPS = 0.0001;

    if(fabs(sin(costheta) - sintheta) < EPS) {
      _theta = costheta;
    } else if(fabs( sin( 2*M_PI - costheta ) - sintheta ) < EPS) {
      _theta = 2*M_PI - costheta;
    } else {
      _theta = 0;
      cout << "Error JP" << endl;
    }

    pppoints[i][0] = _phi;
    pppoints[i][1] = _theta;
    pppoints[i][2] = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
  }
  planePoints.clear();

  QuadTree tree(pppoints, nr_points , 0.3, min_angle);
	vector<set<double *> > cps;
  cps = tree.getClusters();
  
  unsigned int max_points = 0;
  int index = -1;
  for (unsigned int i = 0; i < cps.size(); i++) {
    if (max_points < cps[i].size()) {
      max_points = cps[i].size();
      index = i;
    }
  }
  for (unsigned int i = 0; i < cps.size(); i++) {
    for (set<double *>::iterator it = cps[i].begin(); 
        it != cps[i].end(); it++) {
      Point p;
      p.x = (*it)[2] * cos( (*it)[1] ) * sin( (*it)[0] );
      p.y = (*it)[2] * sin( (*it)[1] ) * sin( (*it)[0] );
      p.z = (*it)[2] * cos( (*it)[0] );
      if ((int)i != index) {
        allPoints->push_back(p);
      } else {
        planePoints.push_back(p);
      }
    }
  }

  double n4[4];
  calcPlane(planePoints, n4);

  ConvexPlane * plane1 = new ConvexPlane(n4, planePoints);
  planes.push_back(plane1);
	

  for (int i = 0; i < nr_points; i++) {
    delete[] pppoints[i];
  }
  delete[] pppoints;
  return max_points;
  
  // ENDE
}

/**
  * Deletes points from allPoints that lie on the given plane. All points from
  * the biggest cluster on that plane are deleted. If the cluster is
  * sufficiently planar, the convex hull is added to the result list.
  *
  * @param n normal vector of the plane
  * @param rho distance between plane and origin
  * @return the number of points in the deleted cluster plane
  */
int Hough::deletePoints(double * n, double rho) {
  char direction = ' ';
  Normalize3(n);
  vector<Point> *nallPoints = new vector<Point>();

  vector<Point> planePoints;
  
  Point p;
  vector<Point>::iterator itr = allPoints->begin();
 
  while(itr != allPoints->end()) {
    p = *(itr);
    // if point close to plane, delete it
    double distance = p.x * n[0] + p.y * n[1] + p.z*n[2] - rho;
    if( fabs(distance) < myConfigFileHough.Get_MaxPointPlaneDist()) {
   
      planePoints.push_back(p);

    } 
   itr++;
  }
  double n2[4];
  // calculating the best fit plane
  double D = calcPlane(planePoints, n2);
  
  bool nocluster = false;
  
  if(D > myConfigFileHough.Get_MinPlanarity()) {
    nocluster = true;
  } 
  
  if(fabs(n2[0]) < fabs(n2[1])) {
    if(fabs(n2[1]) < fabs(n2[2])) {
      direction = 'z';
    } else {
      direction = 'y';
    } 
  } else if (fabs(n2[2]) < fabs(n2[0])){
    direction = 'x';
  } else {
    direction = 'z';
  }

  itr = allPoints->begin();
  planePoints.clear();  
 
  vPtPair planePairs;
  double minx, maxx, miny, maxy;
  minx = 1000000;
  miny = 1000000;
  maxx = -1000000;
  maxy= -1000000;
  while(itr != allPoints->end()) {
    p = *(itr);
    // if point close to plane, delete it
    double distance = p.x * n2[0] + p.y * n2[1] + p.z*n2[2] - n2[3];
    if( fabs(distance) < myConfigFileHough.Get_MaxPointPlaneDist()) {
   
      Point tmp, p2;
      double distance = p.x * n2[0] + p.y * n2[1] + p.z*n2[2] - n2[3];
      tmp.x = p.x - distance * n2[0];
      tmp.y = p.y - distance * n2[1];
      tmp.z = p.z - distance * n2[2];
      switch(direction) {
        case 'x': p2.x = tmp.y;
                  p2.y = tmp.z;
                  break; 
        case 'y': p2.x = tmp.x;
                  p2.y = tmp.z;
                  break;
        case 'z': p2.x = tmp.x;
                  p2.y = tmp.y; 
                  break;
        default: cout << "OHOH" << endl;
      }
      p2.z = -1;
      if(p2.x < minx) minx = p2.x; 
      if(p2.y < miny) miny = p2.y; 
      if(p2.x > maxx) maxx = p2.x; 
      if(p2.y > maxy) maxy = p2.y; 
      PtPair myPair(p,p2);
      
      planePairs.push_back(myPair);

    } else {
      nallPoints->push_back(p);
    }
    itr++;
  }
  delete allPoints;
  allPoints = nallPoints;

  int region = -1;
  if(planePairs.size() > 2) {
    region = cluster(planePairs, minx, maxx, miny, maxy);
  }
  // delete points from this list
  list< double*> point_list;
  
  vPtPair::iterator vitr;  
  vector<Point> tmp_points;
  for(vitr = planePairs.begin(); vitr != planePairs.end(); vitr++) {
    
  // Case distinction x-z or x-y or y-z
    p = (*vitr).p1;
    Point p2 = (*vitr).p2;
    if(fabs(p2.z - region) < 0.1) {
      double * point = new double[2];
      point[0] = p2.x;
      point[1] = p2.y;
      point_list.push_back(point);
      tmp_points.push_back(p);
    } else {
      allPoints->push_back(p);
    }
  }
  D = calcPlane(tmp_points, n2);
  
  nocluster = false;
  
  if(D > myConfigFileHough.Get_MinPlanarity()) {
    nocluster = true;
  }
  unsigned int maxPlane = point_list.size();
  
  // color points
  unsigned char rgb[3];
  for(int x = 0; x < 3; x++) {
    rgb[x] = (unsigned char)((255)*(rand()/(RAND_MAX+1.0)));
  }
  for(itr = tmp_points.begin(); itr != tmp_points.end(); itr++) {
      p = (*itr);
      if(nocluster || maxPlane >= myConfigFileHough.Get_MinPlaneSize()) {
        p.rgb[0] = 0;
        p.rgb[1] = 0;
        p.rgb[2] = 0;
      } else { 
        p.rgb[0] = rgb[0];
        p.rgb[1] = rgb[1];
        p.rgb[2] = rgb[2];
      }
      coloredPoints.push_back(p);
  }
  tmp_points.clear();

  if(nocluster || maxPlane < myConfigFileHough.Get_MinPlaneSize()) {
    for(list<double* >::iterator it = point_list.begin();
      it != point_list.end(); ) {
      double* tmp = (*it);
      it = point_list.erase(it);
      delete[] tmp;
    }
    point_list.clear();
    return maxPlane;
  }

  // If plane ok, calculate convex hull
  vector<double *> convex_hull;
  ConvexPlane::JarvisMarchConvexHull(point_list,convex_hull);
  
  for(list<double* >::iterator it = point_list.begin();
      it != point_list.end(); ) {
      double* tmp = (*it);
      it = point_list.erase(it);
      delete[] tmp;
  }
  point_list.clear();
  
  if(nocluster) {
    for(vector<double* >::iterator it = convex_hull.begin();
      it != convex_hull.end(); it++) {
      double* tmp = (*it);
      delete[] tmp;
    } 
    return maxPlane; 
  }

  ConvexPlane * plane1 = new ConvexPlane(n2, n2[3], direction, convex_hull);
  plane1->pointsize = maxPlane;
  planes.push_back(plane1);

  if(!quiet) cout << "Points left " << allPoints->size() << "\n";
  return maxPlane;
  // ENDE
}

/**
  * Clustering using Two-Pass algorithm
  */
int Hough::cluster(vPtPair &pairs, double minx, double maxx, double miny, double maxy) {
  if(pairs.size() < 3) return -1;
  vPtPair::iterator vitr;
  vector<set<int> > linked;
  double factor = myConfigFileHough.Get_PointDist();
  int xlength = 1 + (maxx - minx) / factor;
  int ylength = 1 + (maxy - miny) / factor; 

  vector< vector<int> > colors;

  for(int i = 0; i < ylength; i++) {
    vector<int> *tmp = new vector<int>;
    colors.push_back(*tmp);
    for(int j = 0; j < xlength; j++) {
      colors[i].push_back(0);
    }
    delete tmp;
  }
 
  //int colors[ylength][xlength];
  vector< vector<bool> > points;
  for(int i = 0; i < ylength; i++) {
    vector<bool> *tmp = new vector<bool>;
    points.push_back(*tmp);
    for(int j = 0; j < xlength; j++) {
      points[i].push_back(false);
    }
    delete tmp;
  }

  int region = 0;
  
  for(vitr = pairs.begin(); vitr != pairs.end(); vitr++) {
    int x = (int)(((*vitr).p2.x - minx)  / (maxx - minx) * xlength - 0.5);   
    int y = (int)(((*vitr).p2.y - miny)  / (maxy - miny) * ylength - 0.5);   
    points[y][x] = true; 
  }
  int up, left;
  for(int y = 0; y < ylength; y++) {
    for(int x = 0; x < xlength; x++) {
      if(points[y][x]) {
        if(x==0) {
          left = 0;
        } else {
          left = colors[y][x-1];
        }
        if(y==0) {
          up = 0;
        } else {
          up = colors[y-1][x];
        }
        if (left == 0) {
          if (up == 0) {
            colors[y][x] = ++region;            // new region
            set<int> joined;
            joined.insert(region);
            linked.push_back(joined);
          } else {
            colors[y][x] = up;    // use upper region
          }
        } else if (up == 0) {
            colors[y][x] = left;  // use left region
        } else {
            colors[y][x] = up;    // use upper region (left is the same)
            if(left == up) {
              continue;
            }
            else {
              set<int> current;
              set<int> joined;
              joined.insert(up);
              joined.insert(left);
              for(vector<set<int> >::iterator itr = linked.begin(); itr != linked.end(); ) {
                current = (*itr);
                if(current.find(up) != current.end() || current.find(left) != current.end()) {
                  set<int> newjoined;
                  insert_iterator<set<int> > res_ins(newjoined, newjoined.begin());

                  set_union(joined.begin(), joined.end(), current.begin(), current.end(), res_ins); 
                  joined.clear();
                 // delete joined;
                  joined = newjoined;
                  itr=linked.erase(itr);
                } else {
                  itr++;
                }
              }
              linked.push_back(joined);
            }
          
        }
        
      }
    }
  }
  int *counter = new int[region+1];
  for(int i = 0; i <= region; i++) {
    counter[i] = 0;
  }
  for(int y = 0; y < ylength; y++) {
    for(int x = 0; x < xlength; x++) {
      counter[colors[y][x]]++;
    }
  }

  int *linkedindex = new int[linked.size()];
  for(unsigned int i = 0; i < linked.size(); i++) {
    linkedindex[i] = 0;
  }  
  set<int> current;
  for(unsigned int i = 0; i < linked.size(); i++) {
    current = linked[i];
    for(set<int>::iterator it = current.begin(); it != current.end(); it++) {
      linkedindex[i] += counter[(*it)];
    }
  }
  
  delete[] counter;

  int maxindex = 0;
  for(unsigned int i = 1; i < linked.size(); i++) {
    if(linkedindex[i] > linkedindex[maxindex]) {
      maxindex = i;
    }
  }
 
  delete[] linkedindex;
  set<int> max = linked[maxindex];
  int zaehler = 0;
  for(vitr = pairs.begin(); vitr != pairs.end(); vitr++) {
    int x = (int)(((*vitr).p2.x - minx) / (maxx - minx) * xlength - 0.5);   
    int y = (int)(((*vitr).p2.y - miny) / (maxy - miny) * ylength - 0.5);   
    if(max.find(colors[y][x]) != max.end()) {
      (*vitr).p2.z = maxindex;
      zaehler++;
    } else {
      (*vitr).p2.z = -1;
    } 
  }
  
   
  /*            
  for(vector<set<int> >::iterator itr = linked.begin(); itr != linked.end(); ) {
    set<>current = (*itr);
                if(current.find(up) != current.end() || current.find(left) != current.end()) {
  */  
  return maxindex;
}


/**
  * Writes <the convex hull of each detected plane to the directory given in the
  * config file. Also writes a list of detected planes to the directory.
  */
int Hough::writePlanes(int startCount) {
  int counter = startCount;
  string blub = "";
  blub = blub + myConfigFileHough.Get_PlaneDir();
#ifdef _MSC_VER
  int success = mkdir(blub.c_str());
#else
  int success = mkdir(blub.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif

  if(success != 0 && errno != EEXIST) { 
    cerr << "Creating directory " << blub << " failed" << endl;
    return -1;
  }
  
  if(!quiet) cout << "Writing " << planes.size() << " Planes to " << blub << endl;
  
  ofstream out;
  string blub3 = blub + "/planes.list";
  out.open(blub3.c_str());
  for(vector<ConvexPlane*>::iterator it = planes.begin(); it != planes.end(); it++) {
    string blub2 = blub + "/plane"+ to_string(counter,3) + ".3d";
    out << "Plane " << blub2 << endl;
    (*it)->writePlane(blub2, counter);
    counter++;
  }

  out.close();
  out.flush();

  return (int)planes.size();
}

/**
  * Writes the convex hull of each detected plane to a file.
  */
void Hough::writePlanes(std::string filePrefix) 
{
  int counter = 0;
  
  if (!quiet) std::cout << "There are " << planes.size() << " planes." << std::endl;
    
  for(vector<ConvexPlane*>::iterator it = planes.begin(); it != planes.end(); it++) 
  {
    std::stringstream ss;
    ss << filePrefix << "_" << counter << ".txt";
    (*it)->writeNormal(ss.str(), counter);
    //(*it)->writePlane(ss.str(), counter);
    counter++;
  }

}
/**
  * Writes the remaining points from allPoints to the plane directory given in
  * the config file.
  */
void Hough::writePlanePoints(string filename) {

  ofstream out;
  out.open(filename.c_str());

  Point p;
  vector<Point>::iterator itr = coloredPoints.begin();
 
  while(itr != coloredPoints.end()) {
    p = *(itr);
    out << p.x << " " << p.y << " " << p.z << " " << (int)p.rgb[0] << " " << (int)(p.rgb[1]) << " " << (int)(p.rgb[2]) << endl;
    itr++;
  }
  itr = allPoints->begin();
  
  while(itr != allPoints->end()) {
    p = *(itr);
    out << p.x << " " << p.y << " " << p.z << " " << 255 << " " << 255 << " " << 255 << endl;
    itr++;
  }
  out.close();
}

/**
  * Writes the remaining points from allPoints to the plane directory given in
  * the config file.
  */
void Hough::writeAllPoints(int index, vector<Point> points) {

  string blub = "";
  blub = blub + myConfigFileHough.Get_PlaneDir() + "/scans";
#ifdef _MSC_VER
  int success = mkdir(blub.c_str());
#else
  int success = mkdir(blub.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif

  if(success != 0 && errno != EEXIST) { 
    cerr << "Creating directory " << blub << " failed" << endl;
    return;
  }
  
  blub = blub + "/scan" + to_string(index,3) + ".3d";

  ofstream out;
  out.open(blub.c_str());

  Point p;
  vector<Point>::iterator itr = points.begin();
  
  while(itr != points.end()) {
    p = *(itr);
    out << p.x << " " << p.y << " " << p.z << " " << (int)p.rgb[0] << " " << (int)(p.rgb[1]) << " " << (int)(p.rgb[2]) << endl;
    itr++;
  }
  out.close();
}


/**
  * Given a set of points this function will calculate the best fit plane
  * @param the set of points
  * @param the plane description as normal vector and distance to origin
  */
double calcPlane(vector<Point> &ppoints, double plane[4]) {
  SymmetricMatrix A(3);
	A = 0;
  int n;
  n = ppoints.size();
  if(n < 3) return 0;
  double cx, cy, cz;
  cx = 0.0;
  cy = 0.0;
  cz = 0.0;

  for (int i = 0; i < n; i++) {
    Point p = ppoints[i];
    cx += p.x;
    cy += p.y;
    cz += p.z;
  }
  cx /= n;
  cy /= n;
  cz /= n;
       
  for (int i = 0; i < n; i++) {
    Point p = ppoints[i];
    A(1, 1) += (p.x - cx)*(p.x - cx);
    A(2, 2) += (p.y - cy)*(p.y - cy);
    A(3, 3) += (p.z - cz)*(p.z - cz);
    A(1, 2) += (p.x - cx)*(p.y - cy);
    A(1, 3) += (p.x - cx)*(p.z - cz);
    A(2, 3) += (p.y - cy)*(p.z - cz);
  }

  DiagonalMatrix D;
  Matrix V;
  try {
    Jacobi(A,D,V);
  } catch (ConvergenceException) {
    cout << "couldn't find plane..." << endl;
    return 0;
  }
  /*
     cout << "A: "<< endl << A << endl;
     cout << "D: "<< endl << D << endl;
     cout << "V: "<< endl << V << endl;
     */
  int index;
  D.MinimumAbsoluteValue1(index);
     
  plane[0] = V(1,index);
  plane[1] = V(2,index);
  plane[2] = V(3,index);
  plane[3] = plane[0]*cx + plane[1]*cy + plane[2]*cz;
  
  double sum = 0.0;
  for(int i = 1; i < 4; i++) {
    sum += D(i);
  }
  sum = D(index)/sum;
  return sum;
}

