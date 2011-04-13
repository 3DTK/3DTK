#ifndef __DISPLAY_H__
#define __DISPLAY_H__
//#include "show/colormanager.h"
#include "limits.h"
#include <vector>
using std::vector;
#include <string>
using std::string;

#include <glu.h>
#include <glut.h>

class Display {
  public:
  //inline void setColorManager(ColorManager *_cm) { cm = _cm; }
  
 
  //virtual Display* readFromFile(string &filename) = 0;
  virtual const char *getName() { return "Unnamed display"; }
  virtual void display(double detail) { displayAll(); } 
  virtual void displayAll();
  
  virtual void displayObject() = 0;

  static void readDisplays(string &filename, vector<Display*> &displays) {
  }

  protected:
  static double mirror[16];
  //ColorManager *cm;
};

class LineDisplay : public Display { 
 
  public:
  static Display* readFromFile(string &filename);
  virtual void displayObject();

  private:

  LineDisplay(vector<float*> &l);

  vector<float *> lines;
};

#endif
