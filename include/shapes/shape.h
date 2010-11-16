#ifndef __SHAPE_H__
#define __SHAPE_H__

#include <vector>
using std::vector;

/**
 * The Shape class is for efficient collision detection in the Octree.
 */
template <class T=double>
class CollisionShape {


  /**
   * This is the main function for speeding up the search for points on the shape.
   *
   * @param cx, cy, cz, size  The center and size of the octrees bucket. Buckets are axis aligned bounding cubes
   *
   * @return returns wether this shape is within the cube (true even if only partially)
   *         if unsure err on the side of caution, i.e. return true
   */
  virtual bool isInCube(T cx, T cy, T cz, T size) = 0;


  virtual bool containsPoint(T* p) = 0;

  virtual void hypothesize(vector<T *> &points) = 0;

  virtual unsigned char getNrPoints() = 0;

  virtual bool valid() = 0;
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
    return fabs(p[0]*nx + p[1]*ny + p[2]*nz + d) < maxDist;
  }

  virtual bool hypothesize(vector<T *> &points) {
    if (points.size() < getNrPoints()) return false;
    T a[3], b[3], c[3], plane[4];
    
    for (int j = 0; j < 3;j++) {                     // compute plane
      a[j] = points[0][j] - points[1][j];
      b[j] = points[0][j] - points[2][j];
      c[j] = points[0][j] + points[1][j] + points[2][j];
      c[j] /= 3.0;
    }
    Cross(a,b, plane);
    if (fabs(Len2(plane)) < 0.0001 ) {
      // points are collinear
      return false;
    }
    Normalize3(plane);
    plane[3] = -1.0 * planeDist(c, plane[0], plane[1], plane[2], 0);    // compute distance from origin
    if (plane[3] < 0.0) {                            // flip normal if necessary
      for (int j = 0; j < 4;j++) {
        plane[j] = -plane[j];
      }
    }
    return true;
  }

  virtual unsigned char getNrPoints() {
    return 3;
  }

  protected:
    T maxDist;
    T nx, ny, nz, d;  // plane equation TODO make nicer 

};


#endif
