#include "show/display.h"
#include "slam6d/globals.icc"
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

void Display::readDisplays(string &filename, vector<Display*> &displays) {
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
      } else if(strcmp(type.c_str(), "Line") == 0) {
        displays.push_back(LineDisplay::readFromFile(objectfile));
      } else if(strcmp(type.c_str(), "GroupPlane") == 0) {
        displays.push_back(GroupPlaneDisplay::readFromFile(objectfile));
      } else {
        cerr << "Unknown Display Object" << endl;
      }
    } catch(...) {
      cerr << "Wrong display type" << endl;
    }
  }
  input.close();
  input.clear();
}

LineDisplay::LineDisplay(vector<float*> &l) {
  lines = l;
}

Display * LineDisplay::readFromFile(string &filename) {
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
  for(int i = 0; i < planes.size(); i++) {
    planes[i]->displayObject();
  }
}

Display* GroupPlaneDisplay::readFromFile(string &filename) {
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

Display * PlaneDisplay::readFromFile(string &filename, float* color) {
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
