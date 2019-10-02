#ifndef __COLORDISPLAY_H__
#define __COLORDISPLAY_H__

#include "show/colormanager.h"
#include "limits.h"
#include <set>

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

  // these are needed on windows so that show can be used as a library
  // we need to call the ExtractFrustum in a same process as the draw function on windows
  void extractFrustumAndDisplayLOD(float lod, short detail) {
	  if (cm) cm->load();
	  extractFrustumAndDrawLOD(lod, detail);
	  if (cm) cm->unload();
  };
  void extractFrustumAndDisplay(short detail) {
	  if (cm) cm->load();
	  extractFrustumAndDraw(detail);
	  if (cm) cm->unload();
  }

  virtual void selectRay(float * &point) {};
  virtual void selectRay(double * &point) {};
  virtual void selectRay(std::set<float*> &points, int depth=INT_MAX) {};
  virtual void selectRay(std::set<double*> &points, int depth=INT_MAX) {};
  virtual void selectRayBrushSize(std::set<float*> &points, int brushsize) {};
  virtual void selectRayBrushSize(std::set<double*> &points, int brushsize) {};

  virtual void cycleLOD() {};

  protected:

  virtual void drawLOD(float lod) = 0;
  virtual void draw() = 0;

  // these are needed on windows so that show can be used as a library
  // we need to call the ExtractFrustum in a same process as the draw function on windows
  virtual void extractFrustumAndDrawLOD(float lod, short detail) = 0;
  virtual void extractFrustumAndDraw(short detail) = 0;

  ColorManager *cm;
};



#endif
