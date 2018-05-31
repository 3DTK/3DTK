#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include "poisson/Geometry.h"
#include "poisson/PoissonParam.h"
#include "wrap/callback.h"

#include "mesh/poisson.h"

using namespace std;
using namespace vcg;

int Execute2(PoissonParam &Par, vector<Point3D<float> > Pts, vector<Point3D<float> > Nor, 	CoredVectorMeshData &mesh, Point3D<float> &newCenter, float &newScale, vcg::CallBackPos *cb );

int main(int argc, char **argv) {
  // fixed command supported now
  // more parameters to be supported later
  if (argc != 5) {
    cout << "Parameters should be like: poisson -in path/of/scan -out path/of/output.obj" << endl;
    return 0;
  }
  else if (strcmp(argv[1], "-in") != 0 || strcmp(argv[3], "-out") != 0) {
    cout << "Parameters should be like: poisson -in path/of/scan -out path/of/output.obj" << endl;
    return 0;
  }
  
  vector<Point> points; 
  vector<Point> normals;

  string inputPath = argv[2];
  string outputPath = argv[4];
  IOType scanType = IOType::XYZ;
  int start = 0, end = 1;

  Scan::openDirectory(false, inputPath, scanType, start, end);

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

  Scan::closeDirectory();

  //// Write calculated normal to file for test
  // fstream fs("./dat/test/calc_norm_test.xyz", fstream::out);
  // for (unsigned int i = 0; i < points.size(); ++i) {
  //   fs << points[i].x << " " << points[i].y << " " << points[i].z << " " 
  //     << normals[i].x << " " << normals[i].y << " " << normals[i].z << endl;
  // }
  // fs.close();

  Poisson poisson;
  poisson.setVertices(points);
  poisson.setNormals(normals);
  poisson.apply();
  // CoredVectorMeshData m;
  // poisson.getMesh(&m);
  poisson.exportMesh(outputPath.c_str());

  return 0;
}

Poisson::Poisson() {
  initialize();
}

Poisson::~Poisson() {
  initialize();
}

void Poisson::initialize() {
  reconstructed = 0;
  vertices.clear();
  normals.clear();
  return;
}

int Poisson::setVertices(vector<Point> v) {
  vertices.clear();
  reconstructed = 0;
  vertices.resize(v.size());
  for (int i = 0; i < vertices.size(); ++i) {
    vertices[i].coords[0] = v[i].x; 
    vertices[i].coords[1] = v[i].y; 
    vertices[i].coords[2] = v[i].z;
  }
  return 1;
}

int Poisson::setNormals(vector<Point> n) {
  normals.clear();
  reconstructed = 0;
  normals.resize(n.size());
  for (int i = 0; i < normals.size(); ++i) {
    normals[i].coords[0] = n[i].x; 
    normals[i].coords[1] = n[i].y; 
    normals[i].coords[2] = n[i].z;
  }
  return 1;
}

int Poisson::getMesh(CoredVectorMeshData *m) {
  if (!ready()) {
    return 0;
  }
  *m = mesh;
  return 1;
}

int Poisson::exportMesh(const char* modelPath) {
  fstream fs(modelPath, fstream::out);
  // // write unscaled out of core vertices
  // for (int i = 0; i < mesh.oocPoints.size(); ++i) {
  //   fs << "v " << mesh.oocPoints[i].coords[0] << " " << mesh.oocPoints[i].coords[1] << " " << mesh.oocPoints[i].coords[2] << endl;
  // }

  Point3D<float> vertex;
  TriangleIndex tIndex;
  int inCoreFlag;
  int numVerticesInCore = mesh.inCorePoints.size(),
    numVerticesOutCore = mesh.oocPoints.size(),
    numFaces=mesh.triangleCount();

  // get and write correct scaled vertices coordinates
  for (int i = 0; i < numVerticesInCore; ++i) {
    vertex = mesh.inCorePoints[i];
    fs << "v " << vertex.coords[0] * scale + center.coords[0] << " " 
      << vertex.coords[1] * scale + center.coords[1] << " " 
      << vertex.coords[2] * scale + center.coords[2] << " " << endl;
  }
  for (int i = 0; i < numVerticesOutCore; ++i) {
    vertex = mesh.oocPoints[i];
    fs << "v " << vertex.coords[0] * scale + center.coords[0] << " " 
      << vertex.coords[1] * scale + center.coords[1] << " " 
      << vertex.coords[2] * scale + center.coords[2] << " " << endl;
  }

  // get and write correct ordered faces indexes
  for (int i = 0; i < numFaces; ++i) {
    mesh.nextTriangle(tIndex, inCoreFlag);
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[0])) {
      tIndex.idx[0]+=int(mesh.inCorePoints.size());
    }
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[1])) {
      tIndex.idx[1]+=int(mesh.inCorePoints.size());
    }
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[2])) {
      tIndex.idx[2]+=int(mesh.inCorePoints.size());
    }
    fs << "f " << tIndex.idx[0] + 1 << " " 
      << tIndex.idx[1] + 1 << " " 
      << tIndex.idx[2] + 1 << " " << endl;
  }

  fs.close();
  return 0;
}

int Poisson::ready() {
  return vertices.size() > 0 && normals.size() > 0 && vertices.size() == normals.size() && reconstructed;
}

int Poisson::apply() {
  reconstructed = Execute2(params, vertices, normals, mesh, center, scale, cb);;
}
