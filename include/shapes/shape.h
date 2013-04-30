#ifndef __SHAPE_H__
#define __SHAPE_H__

#include <vector>
using std::vector;
#include "slam6d/globals.icc"
#include "newmat/newmatio.h"
#include "newmat/newmatap.h"
#include "geom_math.h"
using namespace NEWMAT;



/**
 * The Shape class is for efficient collision detection in the Octree.
 */
template <class T=double>
class CollisionShape {
  public:


  /**
   * This is the main function for speeding up the search for points on the shape.
   *
   * @param cx, cy, cz, size  The center and size of the octrees bucket. Buckets are axis aligned bounding cubes
   *
   * @return returns wether this shape is within the cube (true even if only partially)
   *         if unsure err on the side of caution, i.e. return true
   */
  virtual bool isInCube(T cx, T cy, T cz, T size) = 0;
  
  virtual void refine(vector<T *> *points) = 0;

  virtual bool containsPoint(T* p) = 0;

  virtual bool hypothesize(vector<T *> &points) = 0;

  virtual unsigned char getNrPoints() = 0;

  virtual CollisionShape<T> *copy() = 0;

  virtual CollisionShape<T>& operator=(const CollisionShape<T> &other) {return *this;};

//  virtual bool valid() = 0;
};


template <class T=double>
class CollisionPlane : public CollisionShape<T> {
  public:

//  CollisionPlane (T *_plane, T _maxDist) {
  CollisionPlane (T _maxDist) {
    maxDist = _maxDist;
    // TODO make nicer
  /*  nx = _plane[0];
    ny = _plane[1];
    nz = _plane[2];
    d = _plane[3];*/
  }

  CollisionPlane(T _maxDist, T x, T y, T z, T _d) {
    maxDist = _maxDist;
    nx = x;
    ny = y;
    nz = z;
    d = _d;
  }

  virtual bool isInCube(T x, T y, T z, T size) {
    T xm, xp, ym, yp, zm, zp;
    T Fxm, Fxp, Fym, Fyp, Fzm, Fzp;
    xm = x - size;
    xp = x + size;
    ym = y - size;
    yp = y + size;
    zm = z - size;
    zp = z + size;

    Fxm = nx * xm;
    Fym = ny * ym;
    Fzm = nz * zm;

    bool positive = (Fxm + Fym + Fzm + d > 0);


    Fxp = nx * xp;
    if( (Fxp + Fym + Fzm + d < 0) == positive )
      return true;

    Fyp = ny * yp;
    if( (Fxm + Fyp + Fzm + d < 0) == positive )
      return true;

    if( (Fxp + Fyp + Fzm + d < 0) == positive )
      return true;

    Fzp = nz * zp;
    if( (Fxm + Fym + Fzp + d < 0) == positive )
      return true;

    if( (Fxp + Fym + Fzp + d < 0) == positive )
      return true;
    if( (Fxm + Fyp + Fzp + d < 0) == positive )
      return true;
    if( (Fxp + Fyp + Fzp + d < 0) == positive )
      return true;

    return false;
  }


  virtual bool containsPoint(T* p) {
    return fabs(planeDist(p, nx, ny, nz, d)) < maxDist;
  }

  virtual void refine(vector<T *> *points) {
    cout << nx << " " << ny << " " << nz << " " << d << endl; 
    T plane[4] = {0,0,0,0};
    T centroid[3];
    fitPlane((*points), plane, centroid);
    nx = plane[0];
    ny = plane[1];
    nz = plane[2];
    d = plane[3];
    cout << nx << " " << ny << " " << nz << " " << d << endl; 
  }
  
  virtual bool hypothesize(vector<T *> &points) {
    if (points.size() < getNrPoints()) return false;
    T a[3], b[3], f[3], plane[4];

    for (int j = 0; j < 3;j++) {                     // compute plane
      a[j] = points[0][j] - points[1][j];
      b[j] = points[0][j] - points[2][j];
      f[j] = points[0][j] + points[1][j] + points[2][j];
      f[j] /= 3.0;
    }
    Cross(a,b, plane);
    if (fabs(Len2(plane)) < 0.0001 ) {
      // points are collinear
      return false;
    }
    Normalize3(plane);
    plane[3] = -1.0 * planeDist(f, plane[0], plane[1], plane[2], 0.0);    // compute distance from origin
    if (plane[3] < 0.0) {                            // flip normal if necessary
      for (int j = 0; j < 4;j++) {
        plane[j] = -plane[j];
      }
    }

    nx = plane[0];
    ny = plane[1];
    nz = plane[2];
    d = plane[3];
    return true;
  }

  virtual unsigned char getNrPoints() {
    return 3;
  }
  
  virtual CollisionShape<T>* copy() {
    return new CollisionPlane<T>(maxDist, nx, ny, nz, d); 
  }
  
  
  virtual CollisionPlane<T>& operator=(const CollisionShape<T> &_other) {
    CollisionPlane<T> &other = (CollisionPlane<T> &)_other;
    if (this != &other) {
       this->maxDist =  other.maxDist;
       this->nx      =  other.nx     ;
       this->ny      =  other.ny     ;
       this->nz      =  other.nz     ;
       this->d       =  other.d      ;
    }

    return *this;
  }

  void getPlane(double &x, double &y, double &z, double &_d) {
    x = nx;
    y = ny;
    z = nz;
    _d = d;
  }


  protected:
    T maxDist;
    T nx, ny, nz, d;  // plane equation TODO make nicer 

};

template <class T=double>
class LightBulbPlane : public CollisionPlane<T> {
  public:

  LightBulbPlane (T _maxDist, T _maxSize) : CollisionPlane<T>(_maxDist) {
    maxSize = _maxSize;
    c[0] = 0;
    c[1] = 0;
    c[2] = 0;
  }
  
  LightBulbPlane (T _maxDist, T _maxSize, T x, T y, T z, T _d, T* center) : CollisionPlane<T>(_maxDist) {
    maxSize = _maxSize;
    CollisionPlane<T>::nx = x;
    CollisionPlane<T>::ny = y;
    CollisionPlane<T>::nz = z;
    CollisionPlane<T>::d = _d;
    for(int i = 0; i < 3; i++) {
      c[i] = center[i];
    }

  }
  
  void refine(vector<T *> *points) {
    cout << "LightBulbPlane" << endl; 
    cout << this->nx << " " << this->ny << " " << this->nz << " " << this->d << endl; 
    T plane[4];
    fitPlane((*points), plane, c);
    this->nx = plane[0];
    this->ny = plane[1];
    this->nz = plane[2];
    this->d = plane[3];
    cout << this->nx << " " << this->ny << " " << this->nz << " " << this->d << endl; 
  }
 
  bool isInCube(T cx, T cy, T cz, T size) {
    double radius = sqrt(3*size*size);
    T c_dist = (cx - c[0])*(cx - c[0]) + (cy - c[1])*(cy - c[1]) + (cz - c[2])*(cz - c[2]);
    return c_dist <= ((radius + maxSize)*(radius + maxSize)); 
  }

  virtual bool containsPoint(T* p) {
    if(fabs(p[0]*CollisionPlane<T>::nx + p[1]*CollisionPlane<T>::ny + p[2]*CollisionPlane<T>::nz + CollisionPlane<T>::d) < CollisionPlane<T>::maxDist) {
      return (Dist2(p, c) < (maxSize*maxSize));  
    }
    return false;
  }
  
  virtual bool hypothesize(vector<T *> &points) {
    if(!CollisionPlane<T>::hypothesize(points)) return false;
    double maxSize2 = maxSize*maxSize;
    for(int i = 0; i < 3; i++) {
      for(int j = 0; j < 3; j++) {
        if(Dist2(points[i], points[j]) > maxSize2) return false;
      }
    }
    for (int j = 0; j < 3;j++) {                     // compute plane
      c[j] = points[0][j] + points[1][j] + points[2][j];
      
      c[j] /= 3.0;
    }
    return true;
  }
  
  virtual CollisionShape<T> * copy() {
    return new LightBulbPlane<T>(CollisionPlane<T>::maxDist, maxSize, CollisionPlane<T>::nx, CollisionPlane<T>::ny, CollisionPlane<T>::nz, CollisionPlane<T>::d, c);
  }
  
  virtual LightBulbPlane<T>& operator=(const CollisionShape<T> &_other) {
    LightBulbPlane<T> &other = (LightBulbPlane<T> &)_other;
    if (this != &other) {
      this->maxDist =  other.maxDist;
      this->nx      =  other.nx     ;
      this->ny      =  other.ny     ;
      this->nz      =  other.nz     ;
      this->d       =  other.d      ;
      this->maxSize =  other.maxSize;
      for(int i = 0; i < 3; i++) {
        this->c[i] = other.c[i];
      }
    }

    return *this;
  }

  void getCenter(T &x, T &y, T &z) {
    x = this->c[0];
    y = this->c[1];
    z = this->c[2];
  }
  /*
  bool validate(vector<T *> pts) {
    // create array which will not be used
    bool plane[125][125];
    for(int i = 0; i < 125; i++) {
      for(int j = 0; j < 125; j++) {
        plane[j][i] = false;
      }
    }
    double t[3];
    double alignxf[16]; 
    double aa[4]; 
    aa[0] = -1.0 * acos(this.ny); 
    aa[1] = this.nz / sqrt( this.nz*this.nz + this.nx*this.nx ); 
    aa[2] = 0; 
    aa[3] = -this.nx / sqrt( this.nx*this.nz + this.nx*this.nx );

    AAToMatrix(aa, t, alignxf);

    // compute 2d projection of the points, and scale reflectivity
    for (unsigned int i = 0; i < points.size(); i++) {
      double *p = points[i];                                                                                  
      
      npoints[i] = new double[4];
      wykobi::point2d<double> point; 
      
      double x, y;

      x = -(p[0] * alignxf[0] + p[1] * alignxf[4] + p[2] * alignxf[8]);
      y = p[0] * alignxf[2] + p[1] * alignxf[6] + p[2] * alignxf[10];

      if (x > maxx) maxx = x;
      if (x < minx) minx = y;
      if (y > maxz) maxz = y;
      
      point = wykobi::make_point(x, y);
      point_list.push_back(point);
    }
   
    vector< wykobi::point2d<double> > point_list;
    
    wykobi::polygon<double,2> convex_hull;
    wykobi::algorithm::convex_hull_jarvis_march< wykobi::point2d<double> >(point_list.begin(),point_list.end(),std::back_inserter(convex_hull));

    

    return true;
  }
  */
  protected:
    T maxSize;
    T c[3];
};

#endif
