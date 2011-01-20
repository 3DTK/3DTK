#ifndef __COLORDISPLAY_H__
#define __COLORDISPLAY_H__
#include "show/colormanager.h"
#include "limits.h"
#include <set>
using std::set;

class colordisplay {
  public:
  inline void setColorManager(ColorManager *_cm) { cm = _cm; }
  
  virtual void displayOctTreeCulled(long targetpts) = 0;
  virtual void displayOctTreeAllCulled() = 0; 
  virtual unsigned long maxTargetPoints() = 0;
  virtual void selectRay(float * &point) {};
  virtual void selectRay(double * &point) {};
  virtual void selectRay(set<float*> &points, int depth=INT_MAX) {};
  virtual void selectRay(set<double*> &points, int depth=INT_MAX) {};
  virtual void selectRayBrushSize(set<float*> &points, int brushsize) {};
  virtual void selectRayBrushSize(set<double*> &points, int brushsize) {};

  virtual void displayOctTree(double minsize) {} ;
  virtual void displayOctTree(float minsize) {} ;

  protected:
  ColorManager *cm;
};



#endif
