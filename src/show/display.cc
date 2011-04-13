#include "show/display.h"

#include <fstream>
using std::ifstream;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <glu.h>
#include <glut.h>

double Display::mirror[16] = {1,0,0,0,
  0,1,0,0,
  0,0,-1,0,
  0,0,0,1};
  
void Display::displayAll() {
  // TODO ensure all settings are set to default
  //
  glPushMatrix();
  glMultMatrixd(mirror);
  displayObject();
  glPopMatrix();
}

LineDisplay::LineDisplay(vector<float*> &l) {
  lines = l;
}

Display * LineDisplay::readFromFile(string &filename) {
//  string Filename = "/home/jelseber/software/ros/mystacks/irma3d/bag2scans/lines.pts"; 
  string Filename = "/home/jelseber/software/ros/mystacks/irma3d/bag2scans/corrs.3d"; 
  //string Filename = "/home/jelseber/software/ros/mystacks/irma3d/bag2scans/corr_000_002.pts"; 
//  string Filename = "/home/jelseber/software/ros/mystacks/irma3d/bag2scans/scan000.3d"; 
  ifstream input;
  input.open(Filename.c_str());

  vector<float*> lines;
  while (input.good()) {
    try {
      float *p = new float[6];
      input >> p[0] >> p[1] >> p[2];
      input >> p[3] >> p[4] >> p[5];
      lines.push_back(p);
    } catch (...) {
      break;
    }
  }
  
  input.close();
  input.clear();

  return new LineDisplay(lines);
    
}

void LineDisplay::displayObject() {

  glColor3f(1.0, 0.3, 0.3);
  glLineWidth(1.0);
  glBegin(GL_LINES);

  for (unsigned int i = 0; i < lines.size(); i++) {
    glColor3f(1,0.3,0.3);
    glVertex3f(lines[i][0], lines[i][1], lines[i][2]);
    glColor3f(0.3,1,0.3);
    glVertex3f(lines[i][3], lines[i][4], lines[i][5]);
  }

  glEnd();

  glBegin(GL_POINTS);
  for (unsigned int i = 0; i < lines.size(); i++) {
    glColor3f(1,0.3,0.3);
    glVertex3f(lines[i][0], lines[i][1], lines[i][2]);
    glColor3f(0.3,1,0.3);
    glVertex3f(lines[i][3], lines[i][4], lines[i][5]);
  }
  glEnd();


}
