#include <string.h>
#include <stdlib.h>
#include "show/viewcull.h"
#include "show/scancolormanager.h"
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

using namespace std;

class Display {
  public:
  //inline void setColorManager(ColorManager *_cm) { cm = _cm; }
  
 
  //virtual Display* readFromFile(string &filename) = 0;
  virtual const char *getName() { return "Unnamed display"; }
  virtual void display(double detail) { displayAll(); } 
  virtual void displayAll();
  
  virtual void displayObject() = 0;

  static void readDisplays(string &filename, vector<Display*> &displays); 
  

  protected:
  static double mirror[16];
  //ColorManager *cm;
};

class PointDisplay : public Display { 
 
  public:
  static Display* readFromFile(string &filename);
  virtual void displayObject();

  private:

  PointDisplay(vector<float*> &p, vector<string> &l);

  vector<float *> points;
  vector<string> labels;
};

class LineDisplay : public Display { 
 
  public:
  static Display* readFromFile(string &filename);
  virtual void displayObject();

  private:

  LineDisplay(vector<float*> &l);

  vector<float *> lines;
};

class PlaneDisplay : public Display {
  public: 
  static Display* readFromFile(string &filename, float* color);
  virtual void displayObject();

  private:

  PlaneDisplay(vector<float*> &p, float* c);
  vector<float *> points;
  float * color;

};

class GroupPlaneDisplay : public Display {
  public:
  static Display* readFromFile(string &filename);
  virtual void displayObject();

  private:

  GroupPlaneDisplay(vector<PlaneDisplay*> &p);
  vector<PlaneDisplay*> planes;
};

#endif
