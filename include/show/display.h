#include <string.h>
#include <stdlib.h>
#include "show/scancolormanager.h"
#include "show/viewcull.h"
//#include "show/colormanager.h"
/*
#include "limits.h"
#include <vector>
using std::vector;
#include <string>
#include <string.h>
using std::string;

#include <stdlib.h>
#include <stdio.h>
#include <glut.h>
#include <GL/glut.h>
#include <glu.h>
*/
#ifndef __DISPLAY_H__
#define __DISPLAY_H__

class SDisplay {
  public:
  //inline void setColorManager(ColorManager *_cm) { cm = _cm; }


  //virtual SDisplay* readFromFile(string &filename) = 0;
  virtual const char *getName() { return "Unnamed display"; }
  virtual void display(double detail) { displayAll(); }
  virtual void displayAll();

  virtual void displayObject() = 0;

  static void readDisplays(std::string &filename, std::vector<SDisplay*> &displays);


  protected:
  static double mirror[16];
  //ColorManager *cm;
};

class PointDisplay : public SDisplay {

  public:
  static SDisplay* readFromFile(std::string &filename);
  virtual void displayObject();

  private:

  PointDisplay(std::vector<float*> &p, std::vector<std::string> &l);

  std::vector<float *> points;
  std::vector<std::string> labels;
};

class CoordDisplay : public SDisplay {

  public:
  static SDisplay* readFromFile(std::string &filename);
  virtual void displayObject();

  private:

  CoordDisplay(std::vector<float*> &c);

  std::vector<float *> coords;
};

class LineDisplay : public SDisplay {

  public:
  static SDisplay* readFromFile(std::string &filename);
  virtual void displayObject();

  LineDisplay(std::vector<float*> &l);
  private:

  std::vector<float *> lines;
};

class PlaneDisplay : public SDisplay {
  public:
  static SDisplay* readFromFile(std::string &filename, float* color);
  virtual void displayObject();


  PlaneDisplay(std::vector<float*> &p, float* c);
  private:
  std::vector<float *> points;
  float * color;

};


class GroupPlaneDisplay : public SDisplay {
  public:
  static SDisplay* readFromFile(std::string &filename);
  virtual void displayObject();
  std::vector<PlaneDisplay*> planes;


  GroupPlaneDisplay(std::vector<PlaneDisplay*> &p);
};

class BoxDisplay : public GroupPlaneDisplay {
  public:
  static SDisplay* readFromFile(std::string &filename);
  virtual void displayObject();
  private:
  BoxDisplay(std::vector<PlaneDisplay*> &p);
};

class BoundingBoxDisplay : public SDisplay {
  public:
  static SDisplay* readFromFile(std::string &filename);
  virtual void displayObject();

  private:
  BoundingBoxDisplay(std::vector<float*> &l);
  std::vector<float*> lines;
};

#endif
