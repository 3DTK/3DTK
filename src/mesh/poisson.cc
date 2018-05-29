#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include "slam6d/scan.h"
#include "slam6d/normals.h"

#include "mesh/poisson.h"

using namespace std;

int main(int argc, char **argv) {
  vector<Point> points; 
  vector<Point> normals;

  string path = argv[1];
  IOType scanType = IOType::XYZ;
  int start = 0, end = 1;

  Scan::openDirectory(false, path, scanType, start, end);

  for(ScanVector::iterator it = Scan::allScans.begin();
      it != Scan::allScans.end();
      ++it) {
    Scan* scan = *it;
    const double* rPos = scan->get_rPos();
    DataXYZ xyz = scan->get("xyz");

    points.reserve(xyz.size());
    normals.reserve(xyz.size());

    for(unsigned int j = 0; j < xyz.size(); j++) {
      points.push_back(Point(xyz[j][0], xyz[j][1], xyz[j][2]));
    }
    calculateNormalsKNN(normals, points, 10, rPos);
  }
  fstream fs("./dat/test/calc_norm_test.xyz", fstream::out);
  for (unsigned int i = 0; i < points.size(); ++i) {
    fs << points[i].x << " " << points[i].y << " " << points[i].z << " " 
      << normals[i].x << " " << normals[i].y << " " << normals[i].z << endl;
  }
  fs.close();
  Scan::closeDirectory();
  return 0;
}

Poisson::Poisson() {
  initialize();
}

Poisson::~Poisson() {
  initialize();
}

void Poisson::initialize() {
  return;
}

int writeAsXYZ() {
  return 0;
}
