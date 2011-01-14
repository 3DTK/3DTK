#ifndef __COLORDISPLAY_H__
#define __COLORDISPLAY_H__
#include "show/colormanager.h"

class colordisplay {
  public:
  inline void setColorManager(ColorManager *_cm) { cm = _cm; }
  
  virtual void displayOctTreeCulled(long targetpts) = 0;
  virtual void displayOctTreeAllCulled() = 0; 
  virtual unsigned long maxTargetPoints() = 0;
  virtual void selectRay(float * &point) {};
  virtual void selectRay(double * &point) {};
  virtual void selectRay(vector<float*> &points) {};
  virtual void selectRay(vector<double*> &points) {};

  virtual void displayOctTree(double minsize) {} ;
  virtual void displayOctTree(float minsize) {} ;

  protected:
  ColorManager *cm;
};



#endif
