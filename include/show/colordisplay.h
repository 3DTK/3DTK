#ifndef __COLORDISPLAY_H__
#define __COLORDISPLAY_H__

#include "show/colormanager.h"
#include "limits.h"
#include <set>
using std::set;

class colordisplay {
  public:
  virtual ~colordisplay() {}
  inline void setColorManager(ColorManager *_cm) { cm = _cm; }
  
  void displayLOD(float lod) { 
    if (cm) cm->load();
    drawLOD(lod); 
    if (cm) cm->unload();
  };

  void display() {
    if (cm) cm->load();
    draw();
    if (cm) cm->unload();
  }

  virtual void selectRay(float * &point) {};
  virtual void selectRay(double * &point) {};
  virtual void selectRay(set<float*> &points, int depth=INT_MAX) {};
  virtual void selectRay(set<double*> &points, int depth=INT_MAX) {};
  virtual void selectRayBrushSize(set<float*> &points, int brushsize) {};
  virtual void selectRayBrushSize(set<double*> &points, int brushsize) {};

  virtual void cycleLOD() {};

  protected:
  
  virtual void drawLOD(float lod) = 0; 
  virtual void draw() = 0; 
  
  ColorManager *cm;
};



#endif
