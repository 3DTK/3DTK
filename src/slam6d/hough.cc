#include "newmat/newmatap.h"
using namespace NEWMAT;
#include "slam6d/hough.h"
#include "slam6d/globals.icc"
#include <math.h>
#include <time.h>
#include <limits>
#include "slam6d/quadtree.h"
#include <errno.h>
#include <dlfcn.h>
#include <sys/stat.h>

Hough::Hough(Scan * GlobalScan, bool q) {
 
  quiet = q;
  char CfgFileName[] = "bin/hough.cfg";
  myConfigFileHough.LoadCfg(CfgFileName); 
  cout << "Loaded Configfile" << endl;
  if(!quiet) {
    myConfigFileHough.ShowConfiguration();
  }

  nrEntries = 0;
  maximum = false;

  planeCounter = 0;
  
  allPoints = new vector<Point>();

  const double** points_red = GlobalScan->get_points_reduced();
  for(int i = 0; i < GlobalScan->get_points_red_size(); i++) {
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
  srand(time(0));
  
}

Hough::~Hough() {
  if(out.is_open()) out.close();
  delete acc;
  delete allPoints;
  
}

/**
  * Randomized Hough Transform
  */


void Hough::RHT() {

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
          counter < myConfigFileHough.Get_TrashMax()) { 
    unsigned int pint = (int) (((*allPoints).size())*(rand()/(RAND_MAX+1.0)));

    p1 = (*allPoints)[pint];
    pint = (int) (((*allPoints).size())*(rand()/(RAND_MAX+1.0)));
    p2 = (*allPoints)[pint];
    pint = (int) (((*allPoints).size())*(rand()/(RAND_MAX+1.0)));
    p3 = (*allPoints)[pint];

    // check distance
    if(!distanceOK(p1, p2, p3)) continue;
    // calculate Plane
    if(calculatePlane(p1, p2, p3, theta, phi, rho)) {
      // increment accumulator cell
      if(acc->accumulate(theta, phi, rho)) {
        end = GetCurrentTimeInMilliSec() - start;
        start = GetCurrentTimeInMilliSec();
        if(!quiet) cout << "Time for RHT " << plane << ": " << end << endl; 
        double * n = acc->getMax(rho, theta, phi);
        planeSize = deletePoints(n, rho);
        if(planeSize < myConfigFileHough.Get_MinPlaneSize()) counter++;
        end = GetCurrentTimeInMilliSec() - start;
        start = GetCurrentTimeInMilliSec();
        if(!quiet) cout << "Time for Polygonization " << plane << ": " << end << endl; 
        acc->resetAccumulator();
        plane++;
      }

    }
  
  }
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
  cout << "Time for SHT: " << end << endl; 
  
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
  cout << "Time for Polygonization: " << end << endl; 
  
  it = maxlist->begin();
    for(;it != maxlist->end(); it++) {
      int* tmp = (*it);
      delete[] tmp;
  } 
  
  delete maxlist; 
   
}

/**
 * Probabilistic Hough Transform
 */

void Hough::PHT() {
  unsigned int stop =
  (int)(allPoints->size()/100.0)*myConfigFileHough.Get_MinSizeAllPoints();
  bool voted[allPoints->size()];
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
}

/**
 * Progressive Probabilistic Hough Transform
 */

void Hough::PPHT() {
  unsigned int stop =
  (int)(allPoints->size()/100.0)*myConfigFileHough.Get_MinSizeAllPoints();
  while(stop < allPoints->size() && 
        planes.size() < (unsigned int)myConfigFileHough.Get_MaxPlanes()) {
    bool voted[allPoints->size()];
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
          double * n = acc->getMax(angles[0], angles[1], angles[2]);
          deletePoints(n, angles[0]);
          acc->resetAccumulator();
          delete [] n;
        } else {
          voted[pint] = true;
        }
        delete [] angles;
      }

    } while(!voted[pint]);
    
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
          if( sqrt( 
            ((*it)[3] - (tmp[3])) * ((*it)[3] - (tmp[3])) +
            ((*it)[1] - (tmp[1])) * ((*it)[1] - (tmp[1])) +
            ((*it)[2] - (tmp[2])) * ((*it)[2] - (tmp[2])) ) < 3.0) {
            insert = false;
            break;
          }
        } else { 
          if( sqrt( 
            ((*it)[4] - (tmp[4])) * ((*it)[4] - (tmp[4])) +
            ((*it)[1] - (tmp[1])) * ((*it)[1] - (tmp[1])) +
            ((*it)[2] - (tmp[2])) * ((*it)[2] - (tmp[2])) +
            ((*it)[3] - (tmp[3])) * ((*it)[3] - (tmp[3])) ) < 3.0) {
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
            if( sqrt( 
              ((tmp2)[3] - (tmp1[3])) * ((tmp2)[3] - (tmp1[3])) +
              ((tmp2)[1] - (tmp1[1])) * ((tmp2)[1] - (tmp1[1])) +
              ((tmp2)[2] - (tmp1[2])) * ((tmp2)[2] - (tmp1[2])) ) < 3.0) {
              treffer = true;
              break;
            }
          } else { 
            if( sqrt( 
              ((tmp2)[4] - (tmp1[4])) * ((tmp2)[4] - (tmp1[4])) +
              ((tmp2)[1] - (tmp1[1])) * ((tmp2)[1] - (tmp1[1])) +
              ((tmp2)[2] - (tmp1[2])) * ((tmp2)[2] - (tmp1[2])) +
              ((tmp2)[3] - (tmp1[3])) * ((tmp2)[3] - (tmp1[3])) ) < 3.0) {
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
  } while(max < myConfigFileHough.Get_AccumulatorMax());

  for(int i = 0; i <= maxpos; i++) {
    double * n = acc->getMax(mergelist[i]); 
    deletePoints(n, n[3]);
   
    delete[] n;
  }
      
}

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
  * Verbesserte Funktion zum loeschen der Punkte
  */
int Hough::deletePointsQuad(double * n, double rho) {

  cout << allPoints->size() ;
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
  cout << "A" << endl;
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
  cout << "B" << endl;
  delete allPoints;
  allPoints = nallPoints;
  
  cout << "Planepoints " << planePoints.size() << endl;
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
      cout << "Fehler JP" << endl;
    }

    pppoints[i][0] = _phi;
    pppoints[i][1] = _theta;
    pppoints[i][2] = sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
  }
  planePoints.clear();
  cout << "C" << endl;

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

  ConvexPlane plane1(n4, planePoints);
  planes.push_back(plane1);
	

  for (int i = 0; i < nr_points; i++) {
    delete[] pppoints[i];
  }
  cout << "E" << endl;
  delete[] pppoints;
  return max_points;
  
  // ENDE
}

/**
  * Clustern using Two-Pass algorithm
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

  
  int region = cluster(planePairs, minx, maxx, miny, maxy);
  vector< wykobi::point2d<double> > point_list;
  
  vPtPair::iterator vitr;
  for(vitr = planePairs.begin(); vitr != planePairs.end(); vitr++) {
    
  // Case distinction x-z or x-y or y-z
    p = (*vitr).p1;
    Point p2 = (*vitr).p2;
    if(fabs(p2.z - region) < 0.1) {
      wykobi::point2d<double> point; 
      point = wykobi::make_point(p2.x, p2.y);
      point_list.push_back(point);
    } else {
      allPoints->push_back(p);
    }
  }
  int maxPlane = point_list.size();
  if(maxPlane < myConfigFileHough.Get_MinPlaneSize()) return maxPlane;
  wykobi::polygon<double,2> convex_hull;
  wykobi::polygon<double,2> convex_hull2;
  wykobi::algorithm::convex_hull_jarvis_march< wykobi::point2d<double> >(point_list.begin(),point_list.end(),std::back_inserter(convex_hull));

  
  if(nocluster) return maxPlane; 

  ConvexPlane plane1(n, n2[3], direction, convex_hull);
  planes.push_back(plane1);

  cout << " " << allPoints->size() << "\n";

  return maxPlane;
  // ENDE
}

int Hough::cluster(vPtPair &pairs, double minx, double maxx, double miny, double maxy) {
  vPtPair::iterator vitr;
  vector<set<int> > linked;
  double factor = myConfigFileHough.Get_MaxPointPlaneDist()*3;
  
  int xlength = 1 + (maxx - minx) / factor;
  int ylength = 1 + (maxy - miny) / factor; 

  int colors[ylength][xlength];
  bool points[ylength][xlength];
  int region = 0;
  for(int x = 0; x < xlength; x++) {
    for(int y = 0; y < ylength; y++) {
      points[y][x] = false;
      colors[y][x] = region;
    }
  }
  
  for(vitr = pairs.begin(); vitr != pairs.end(); vitr++) {
    int x = (int)(((*vitr).p2.x - minx - 0.5)  / (maxx - minx) * xlength);   
    int y = (int)(((*vitr).p2.y - miny - 0.5)  / (maxy - miny) * ylength);   
    
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
            set<int> *joined = new set<int>;
            joined->insert(region);
            linked.push_back(*joined);
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
              set<int> *joined = new set<int>;
              joined->insert(up);
              joined->insert(left);
              for(vector<set<int> >::iterator itr = linked.begin(); itr != linked.end(); ) {
                current = (*itr);
                if(current.find(up) != current.end() || current.find(left) != current.end()) {
                  set<int> *newjoined = new set<int>();
                  insert_iterator<set<int> > res_ins(*newjoined, newjoined->begin());

                  set_union(joined->begin(), joined->end(), current.begin(), current.end(), res_ins); 
                  joined->clear();
                  delete joined;
                  joined = newjoined;
                  itr=linked.erase(itr);
                } else {
                  itr++;
                }
              }
              linked.push_back(*joined);
            }
          
        }
        
      }
    }
  }
  
  int counter[region+1];
  for(int i = 0; i <= region; i++) {
    counter[i] = 0;
  }
  for(int y = 0; y < ylength; y++) {
    for(int x = 0; x < xlength; x++) {
      counter[colors[y][x]]++;
    }
  }

  int linkedindex[linked.size()];
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
  
  int maxindex = 0;
  for(unsigned int i = 1; i < linked.size(); i++) {
    if(linkedindex[i] > linkedindex[maxindex]) {
      maxindex = i;
    }
  }
 
  set<int> max = linked[maxindex];
  int zaehler = 0;
  for(vitr = pairs.begin(); vitr != pairs.end(); vitr++) {
    int x = (int)(((*vitr).p2.x - minx - 0.5) / (maxx - minx) * xlength);   
    int y = (int)(((*vitr).p2.y - miny - 0.5) / (maxy - miny) * ylength);   
    if(max.find(colors[y][x]) != max.end()) {
      (*vitr).p2.z = maxindex;
      zaehler++;
    } else {
      (*vitr).p2.z = -1;
    } 
  }
  return maxindex;
}

void Hough::writePlanes() {
  int counter = 0;
  string blub = "";
  blub = blub + myConfigFileHough.Get_PlaneDir();
#ifdef _MSC_VER
  int success = mkdir(blub.c_str());
#else
  int success = mkdir(blub.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
#endif

  if(success != 0 && errno != EEXIST) { 
    cerr << "Creating directory " << blub << " failed" << endl;
    return;
  }
  
  if(!quiet) cout << "Writing " << planes.size() << " Planes to " << blub << endl;
  
  for(vector<ConvexPlane>::iterator it = planes.begin(); it != planes.end(); it++) {
    string blub2 = blub + "/plane"+ to_string(counter,3) + ".3d";
    (*it).writePlane(blub2, counter);
    counter++;
  }
}

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
    out << p.x << " " << p.y << " " << p.z << endl;
    itr++;
  }
  out.close();
}


// given a set of points this will calculate the best fit plane
double calcPlane(vector<Point> &ppoints, double plane[4]) {
  SymmetricMatrix A(3);
	A = 0;
  int n;
  n = ppoints.size();
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

  return D(index)/n;
}

