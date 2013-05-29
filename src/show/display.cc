/*
 * display implementation
 *
 * Copyright (C) Dorit Borrmann, Jan Elseberg
 *
 * Released under the GPL version 3.
 *
 */

#include "show/display.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#ifdef __APPLE__
#include <GLUT/glut.h>
#elif WITH_FREEGLUT
#include <GL/freeglut.h>
#else
#include <GL/glut.h>
#endif

#include <glui.h>


double SDisplay::mirror[16] = {1,0,0,0,
  0,1,0,0,
  0,0,-1,0,
  0,0,0,1};
  
void SDisplay::displayAll() {
  // TODO ensure all settings are set to default
  //
  glPushMatrix();
  glMultMatrixd(mirror);
  displayObject();
  glPopMatrix();
}

void SDisplay::readDisplays(string &filename, vector<SDisplay*> &displays) {
  ifstream input;
  input.open(filename.c_str());

  string type;
  string objectfile;
  while(input.good()) {
    input >> type;
    try {
      input >> objectfile;
      if(strcmp(type.c_str(), "Plane") == 0) {
        float * color = new float[3];
        for(int i = 0; i < 3; i++) {
          color[i] = rand()/((double)RAND_MAX);
        }
        displays.push_back(PlaneDisplay::readFromFile(objectfile, color));
      } else if(strcmp(type.c_str(), "Point") == 0) {
        displays.push_back(PointDisplay::readFromFile(objectfile));
      } else if(strcmp(type.c_str(), "Line") == 0) {
        displays.push_back(LineDisplay::readFromFile(objectfile));
      } else if(strcmp(type.c_str(), "GroupPlane") == 0) {
        displays.push_back(GroupPlaneDisplay::readFromFile(objectfile));
      } else {
        cerr << "Unknown SDisplay Object" << endl;
      }
    } catch(...) {
      cerr << "Wrong display type" << endl;
    }
  }
  input.close();
  input.clear();
}

PointDisplay::PointDisplay(vector<float*> &p, vector<string> &l) {
  points = p;
  labels = l;
}

SDisplay * PointDisplay::readFromFile(string &filename) {
  ifstream input;
  input.open(filename.c_str());

  vector<float*> points;
  vector<string> labels;
  while (input.good()) {
    try {
      float *p = new float[3];
      input >> p[0] >> p[1] >> p[2];
      points.push_back(p);
      string l;
      input >> l;
      labels.push_back(l);
    } catch (...) {
      break;
    }
  }
 
  points.pop_back();
  labels.pop_back();
  input.close();
  input.clear();

  return new PointDisplay(points, labels);
    
}

void PointDisplay::displayObject() {

  glColor3f(1.0, 0.3, 0.3);
  
  glPointSize(5);
  glBegin(GL_POINTS);
  for (unsigned int i = 0; i < points.size(); i++) {
    glVertex3f(points[i][0], points[i][1], points[i][2]);
  }
  glEnd();
  
  glPushMatrix();
  for(unsigned int i = 0; i < points.size(); i++) {
    
    glRasterPos3f(points[i][0], points[i][1], points[i][2]);
    _glutBitmapString(GLUT_BITMAP_HELVETICA_10, labels[i].c_str());
    //_glutBitmapString(GLUT_BITMAP_9_BY_15, "http://threedtk.de");
   
  }
  glPopMatrix();
  
}

LineDisplay::LineDisplay(vector<float*> &l) {
  lines = l;
}

SDisplay * LineDisplay::readFromFile(string &filename) {
  ifstream input;
  input.open(filename.c_str());

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
 
  lines.pop_back();
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

GroupPlaneDisplay::GroupPlaneDisplay(vector<PlaneDisplay*> &p) {
  planes = p;
}

void GroupPlaneDisplay::displayObject() {
  for(unsigned int i = 0; i < planes.size(); i++) {
    planes[i]->displayObject();
  }
}

SDisplay* GroupPlaneDisplay::readFromFile(string &filename) {
  ifstream input;
  input.open(filename.c_str());
  vector<PlaneDisplay*> planes;
  if(input.good()) {
    int end = filename.find_last_of('/');
    string dir = filename.substr(0, end);
    string tmp;
    input >> tmp;
    if(dir.length() != 0) dir = dir + "/"; 
    try {
    if(strcmp(tmp.c_str(), "randomcolor") != 0) {
      while(input.good()) {
        int planeNr;
        input >> planeNr;
        string planename = dir + "plane" + to_string(planeNr, 3) + ".3d";
        float * color = new float[3];
        for(int i = 0; i < 3; i++) {
          input >> color[i];
        }
        planes.push_back((PlaneDisplay*)PlaneDisplay::readFromFile(planename, color));
      }
    } else {
      while(input.good()) {
        int planeNr;
        input >> planeNr;
        string planename = dir + "/plane" + to_string(planeNr, 3) + ".3d";
        float * color = new float[3];
        for(int i = 0; i < 3; i++) {
          color[i] = rand()/((double)RAND_MAX);
        }
        planes.push_back((PlaneDisplay*)PlaneDisplay::readFromFile(planename, color));
      }
    }
    } catch(...) {}
  }
  input.close();
  input.clear();
  return new GroupPlaneDisplay(planes);
  
}

PlaneDisplay::PlaneDisplay(vector<float*> &p, float* c) {
  points = p;
  color = c;
}

SDisplay * PlaneDisplay::readFromFile(string &filename, float* color) {
  ifstream input;
  input.open(filename.c_str());

  vector<float*> points;
  while (input.good()) {
    try {
      float *p = new float[3];
      input >> p[0] >> p[1] >> p[2];
      points.push_back(p);
    } catch (...) {
      break;
    }
  }
  points.pop_back();
  
  input.close();
  input.clear();

  return new PlaneDisplay(points, color);
}

void PlaneDisplay::displayObject() {
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // TODO
  
  glColor4d(color[0], color[1], color[2], 0.5);
  glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
  glBegin(GL_POLYGON);
  for(vector<float*>::iterator point = (points).begin();
    point != (points).end(); point++) {
    glVertex3f((*point)[0], (*point)[1], (*point)[2]);
  }
  glEnd();
  glBlendFunc(GL_ONE, GL_ZERO); // TODO
}
