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
#include "show/show_common.h"
#include <fstream>
#include <iostream>

#ifdef WITH_OPENGL
#include <GL/glui.h>
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#ifdef WIN32
#define FREEGLUT_STATIC
#define _LIB
#define FREEGLUT_LIB_PRAGMAS 0
#endif
#include <GL/freeglut.h>
#endif
#else
#include "show/dummygl.h"
#endif

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

void SDisplay::readDisplays(std::string &filename, std::vector<SDisplay*> &displays) {
  std::ifstream input;
  input.open(filename.c_str());

  std::string type;
  std::string objectfile;
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
      } else if(strcmp(type.c_str(), "Box") == 0) {
        displays.push_back(BoxDisplay::readFromFile(objectfile));
      } else if(strcmp(type.c_str(), "Coord") == 0) {
        displays.push_back(CoordDisplay::readFromFile(objectfile));
      } else if(strcmp(type.c_str(), "BoundingBox") == 0) {
        displays.push_back(BoundingBoxDisplay::readFromFile(objectfile));
      } else {
        std::cerr << "Unknown SDisplay Object" << std::endl;
      }
    } catch(...) {
      std::cerr << "Wrong display type" << std::endl;
    }
  }
  input.close();
  input.clear();
}

PointDisplay::PointDisplay(std::vector<float*> &p, std::vector<std::string> &l) {
  points = p;
  labels = l;
}

SDisplay * PointDisplay::readFromFile(std::string &filename) {
  std::ifstream input;
  input.open(filename.c_str());

  std::vector<float*> points;
  std::vector<std::string> labels;
  while (input.good()) {
    try {
      float *p = new float[3];
      input >> p[0] >> p[1] >> p[2];
      points.push_back(p);
      std::string l;
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

CoordDisplay::CoordDisplay(std::vector<float*> &c) {
  coords = c;
}

SDisplay * CoordDisplay::readFromFile(std::string &filename) {
  std::ifstream input;
  input.open(filename.c_str());
  float dummy;
  std::vector<float*> coords;
  while (input.good()) {
    try {
      float *p = new float[16];
      double tmp[4];
      for(unsigned int i = 0; i < 4; i++) {
        for(unsigned int j = 0; j < 4; j++) {
          input >> tmp[j];
        }
        if(i < 3) Normalize3(tmp);
        for(unsigned int j = 0; j < 4; j++) {
          p[i*4+j] = tmp[j];
        }
      }
      input >> dummy;
      coords.push_back(p);
    } catch (...) {
      break;
    }
  }

  coords.pop_back();
  input.close();
  input.clear();

  return new CoordDisplay(coords);

}

void CoordDisplay::displayObject() {

  glColor3f(1.0, 0.3, 0.3);
  glLineWidth(5.0);
  glBegin(GL_LINES);

  for (unsigned int i = 0; i < coords.size(); i++) {

    float s = 10.0*coords[i][15];
    glColor3f(1,0.0,0.0);
    glVertex3f(coords[i][12], coords[i][13], coords[i][14]);
    glVertex3f(coords[i][12] + s*coords[i][0], coords[i][13] + s*coords[i][1], coords[i][14] + s*coords[i][2]);
    glColor3f(0.0,1.0,0.0);
    glVertex3f(coords[i][12], coords[i][13], coords[i][14]);
    glVertex3f(coords[i][12] + s*coords[i][4], coords[i][13] + s*coords[i][5], coords[i][14] + s*coords[i][6]);
    glColor3f(0.0,0.0,1.0);
    glVertex3f(coords[i][12], coords[i][13], coords[i][14]);
    glVertex3f(coords[i][12] + s*coords[i][8], coords[i][13] + s*coords[i][9], coords[i][14] + s*coords[i][10]);
  }

  glEnd();

}


LineDisplay::LineDisplay(std::vector<float*> &l) {
  lines = l;
}

SDisplay * LineDisplay::readFromFile(std::string &filename) {
  std::ifstream input;
  input.open(filename.c_str());

  std::vector<float*> lines;
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


BoxDisplay::BoxDisplay(std::vector<PlaneDisplay*> &p) : GroupPlaneDisplay(p)  {
  //planes = p;
}

SDisplay* BoxDisplay::readFromFile(std::string &filename) {
  std::ifstream input;
  input.open(filename.c_str());
  std::vector<PlaneDisplay*> planes;
  if(input.good()) {
    double minx, maxx, miny, maxy, minz, maxz;
    int end = filename.find_last_of('/');
    std::string dir = filename.substr(0, end);
    std::string tmp;
    input >> minx;
    input >> maxx;
    input >> miny;
    input >> maxy;
    input >> minz;
    input >> maxz;

    try {
      for(int i = 0; i < 6; i++) {
        float * color = new float[3];
        for(int i = 0; i < 3; i++) {
          color[i] = rand()/((double)RAND_MAX);
        }
        std::vector<float*> points;
        float *p1 = new float[3];
        float *p2 = new float[3];
        float *p3 = new float[3];
        float *p4 = new float[3];
        switch(i) {
          case 0:
            p1[0] = minx;
            p1[1] = miny;
            p1[2] = minz;
            points.push_back(p1);
            p2[0] = minx;
            p2[1] = maxy;
            p2[2] = minz;
            points.push_back(p2);
            p3[0] = maxx;
            p3[1] = maxy;
            p3[2] = minz;
            points.push_back(p3);
            p4[0] = maxx;
            p4[1] = miny;
            p4[2] = minz;
            points.push_back(p4);
            break;
          case 1:
            p1[0] = minx;
            p1[1] = miny;
            p1[2] = maxz;
            points.push_back(p1);
            p2[0] = minx;
            p2[1] = maxy;
            p2[2] = maxz;
            points.push_back(p2);
            p3[0] = maxx;
            p3[1] = maxy;
            p3[2] = maxz;
            points.push_back(p3);
            p4[0] = maxx;
            p4[1] = miny;
            p4[2] = maxz;
            points.push_back(p4);
            break;
          case 2:
            p1[2] = minz;
            p1[1] = miny;
            p1[0] = minx;
            points.push_back(p1);
            p2[2] = minz;
            p2[1] = maxy;
            p2[0] = minx;
            points.push_back(p2);
            p3[2] = maxz;
            p3[1] = maxy;
            p3[0] = minx;
            points.push_back(p3);
            p4[2] = maxz;
            p4[1] = miny;
            p4[0] = minx;
            points.push_back(p4);
            break;
          case 3:
            p1[2] = minz;
            p1[1] = miny;
            p1[0] = maxx;
            points.push_back(p1);
            p2[2] = minz;
            p2[1] = maxy;
            p2[0] = maxx;
            points.push_back(p2);
            p3[2] = maxz;
            p3[1] = maxy;
            p3[0] = maxx;
            points.push_back(p3);
            p4[2] = maxz;
            p4[1] = miny;
            p4[0] = maxx;
            points.push_back(p4);
            break;
          case 4:
            p1[0] = minx;
            p1[2] = minz;
            p1[1] = miny;
            points.push_back(p1);
            p2[0] = minx;
            p2[2] = maxz;
            p2[1] = miny;
            points.push_back(p2);
            p3[0] = maxx;
            p3[2] = maxz;
            p3[1] = miny;
            points.push_back(p3);
            p4[0] = maxx;
            p4[2] = minz;
            p4[1] = miny;
            points.push_back(p4);
            break;
          case 5:
            p1[0] = minx;
            p1[2] = minz;
            p1[1] = maxy;
            points.push_back(p1);
            p2[0] = minx;
            p2[2] = maxz;
            p2[1] = maxy;
            points.push_back(p2);
            p3[0] = maxx;
            p3[2] = maxz;
            p3[1] = maxy;
            points.push_back(p3);
            p4[0] = maxx;
            p4[2] = minz;
            p4[1] = maxy;
            points.push_back(p4);
            break;
        }
        planes.push_back(new PlaneDisplay(points, color));
      }
    } catch(...) {}
  }
  input.close();
  input.clear();
  return new BoxDisplay(planes);

}

void BoxDisplay::displayObject() {
  for(unsigned int i = 0; i < planes.size(); i++) {
    planes[i]->displayObject();
  }
}


GroupPlaneDisplay::GroupPlaneDisplay(std::vector<PlaneDisplay*> &p) {
  planes = p;
}

void GroupPlaneDisplay::displayObject() {
  for(unsigned int i = 0; i < planes.size(); i++) {
    planes[i]->displayObject();
  }
}

SDisplay* GroupPlaneDisplay::readFromFile(std::string &filename) {
  std::ifstream input;
  input.open(filename.c_str());
  std::vector<PlaneDisplay*> planes;
  if(input.good()) {
    int end = filename.find_last_of('/');
    std::string dir = filename.substr(0, end);
    std::string tmp;
    input >> tmp;
    if(dir.length() != 0) dir = dir + "/";
    try {
    if(strcmp(tmp.c_str(), "randomcolor") != 0) {
      while(input.good()) {
        int planeNr;
        input >> planeNr;
        std::string planename = dir + "plane" + to_string(planeNr, 3) + ".3d";
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
        std::string planename = dir + "/plane" + to_string(planeNr, 3) + ".3d";
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

PlaneDisplay::PlaneDisplay(std::vector<float*> &p, float* c) {
  points = p;
  color = c;
}

SDisplay * PlaneDisplay::readFromFile(std::string &filename, float* color) {
  std::ifstream input;
  input.open(filename.c_str());

  std::vector<float*> points;
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
  for(std::vector<float*>::iterator point = (points).begin();
    point != (points).end(); point++) {
    glVertex3f((*point)[0], (*point)[1], (*point)[2]);
  }
  glEnd();
  glBlendFunc(GL_ONE, GL_ZERO); // TODO
}

BoundingBoxDisplay::BoundingBoxDisplay(std::vector<float*> &l) {
  lines = l;
}

SDisplay * BoundingBoxDisplay::readFromFile(std::string &filename) {
  std::ifstream input;
  input.open(filename.c_str());

  std::vector<float*> lines;
  if (input.good()) {
    std::string currentLine;
    while (std::getline(input, currentLine)) {
      std::istringstream iss(currentLine);
      double minx, maxx, miny, maxy, minz, maxz;
      if (!(iss >> minx >> maxx >> miny >> maxy >> minz >> maxz)) break;

      //along x-axis
      float* p5 = new float[6];
      p5[0] = minx;
      p5[1] = miny;
      p5[2] = maxz;
      p5[3] = maxx;
      p5[4] = miny;
      p5[5] = maxz;
      lines.push_back(p5);
      float* p3 = new float[6];
      p3[0] = minx;
      p3[1] = miny;
      p3[2] = minz;
      p3[3] = maxx;
      p3[4] = miny;
      p3[5] = minz;
      lines.push_back(p3);
      float* p10 = new float[6];
      p10[0] = minx;
      p10[1] = maxy;
      p10[2] = maxz;
      p10[3] = maxx;
      p10[4] = maxy;
      p10[5] = maxz;
      lines.push_back(p10);
      float* p12 = new float[6];
      p12[0] = maxx;
      p12[1] = maxy;
      p12[2] = minz;
      p12[3] = minx;
      p12[4] = maxy;
      p12[5] = minz;
      lines.push_back(p12);

      //along y-axis
      float* p2 = new float[6];
      p2[0] = minx;
      p2[1] = miny;
      p2[2] = minz;
      p2[3] = minx;
      p2[4] = maxy;
      p2[5] = minz;
      lines.push_back(p2);
      float* p4 = new float[6];
      p4[0] = minx;
      p4[1] = miny;
      p4[2] = maxz;
      p4[3] = minx;
      p4[4] = maxy;
      p4[5] = maxz;
      lines.push_back(p4);
      float* p7 = new float[6];
      p7[0] = maxx;
      p7[1] = miny;
      p7[2] = maxz;
      p7[3] = maxx;
      p7[4] = maxy;
      p7[5] = maxz;
      lines.push_back(p7);
      float* p8 = new float[6];
      p8[0] = maxx;
      p8[1] = miny;
      p8[2] = minz;
      p8[3] = maxx;
      p8[4] = maxy;
      p8[5] = minz;
      lines.push_back(p8);

      //along z-axis
      float* p1 = new float[6];
      p1[0] = minx;
      p1[1] = miny;
      p1[2] = minz;
      p1[3] = minx;
      p1[4] = miny;
      p1[5] = maxz;
      lines.push_back(p1);
      float* p6 = new float[6];
      p6[0] = maxx;
      p6[1] = miny;
      p6[2] = maxz;
      p6[3] = maxx;
      p6[4] = miny;
      p6[5] = minz;
      lines.push_back(p6);
      float* p9 = new float[6];
      p9[0] = minx;
      p9[1] = maxy;
      p9[2] = minz;
      p9[3] = minx;
      p9[4] = maxy;
      p9[5] = maxz;
      lines.push_back(p9);
      float* p11 = new float[6];
      p11[0] = maxx;
      p11[1] = maxy;
      p11[2] = maxz;
      p11[3] = maxx;
      p11[4] = maxy;
      p11[5] = minz;
      lines.push_back(p11);
    }
 }
  input.close();
  input.clear();

  return new BoundingBoxDisplay(lines);

}

void BoundingBoxDisplay::displayObject() {

  glLineWidth(4*pointsize);
  glBegin(GL_LINES);

  for (unsigned int i = 0; i < lines.size(); i++) {
    int boxReset = i%12;
    if (boxReset < 4) {
      glColor3f(1.0, 0.0, 0.0);
      glVertex3f(lines[i][0], lines[i][1], lines[i][2]);
      glColor3f(1.0, 0.0, 0.0);
      glVertex3f(lines[i][3], lines[i][4], lines[i][5]);
    } if (boxReset >= 4 && boxReset < 8) {
      glColor3f(0.0, 1.0, 0.0);
      glVertex3f(lines[i][0], lines[i][1], lines[i][2]);
      glColor3f(0.0, 1.0, 0.0);
      glVertex3f(lines[i][3], lines[i][4], lines[i][5]);
    } if (boxReset >= 8 && boxReset < 12) {
      glColor3f(0.0, 0.0, 1.0);
      glVertex3f(lines[i][0], lines[i][1], lines[i][2]);
      glColor3f(0.0, 0.0, 1.0);
      glVertex3f(lines[i][3], lines[i][4], lines[i][5]);
    }

  }

  glEnd();

  for (unsigned int i = 0; i < lines.size(); i++) {
    glColor3f(1.0, 0.0, 1.0);
    glVertex3f(lines[i][0], lines[i][1], lines[i][2]);
    glColor3f(1.0, 0.0, 1.0);
    glVertex3f(lines[i][3], lines[i][4], lines[i][5]);
  }
  glEnd();


}
/*
void BoundingBoxDisplay::displayObject() {

  for(unsigned int i = 0; i < lines.size(); i++) {
    lines[i]->displayObject();
  }

}
*/
