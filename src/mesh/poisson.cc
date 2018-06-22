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

// test mesh filtering with vcglib ---
#include <vcg/complex/complex.h>
#include <vcg/complex/algorithms/update/topology.h>
#include <vcg/complex/algorithms/update/normal.h>
#include <vcg/complex/algorithms/clean.h>
#include <vcg/complex/algorithms/smooth.h>
#include <wrap/io_trimesh/import.h>
#include <wrap/io_trimesh/export_ply.h>
class MyFace;
class MyVertex;
struct MyUsedTypes : public UsedTypes<	Use<MyVertex>::AsVertexType, Use<MyFace>::AsFaceType>{};
class MyVertex : public Vertex< MyUsedTypes, vertex::VFAdj, vertex::Coord3f, vertex::Normal3f, vertex::BitFlags  >{};
class MyFace : public Face  < MyUsedTypes, face::VFAdj, face::Normal3f, face::VertexRef, face::BitFlags > {};
class MyMesh : public vcg::tri::TriMesh<vector<MyVertex>, vector<MyFace> > {};
// test mesh filtering with vcglib ---

int Execute2(PoissonParam &Par, vector<Point3D<float>> Pts, vector<Point3D<float>> Nor, CoredVectorMeshData &mesh, Point3D<float> &newCenter, float &newScale, vcg::CallBackPos *cb );

Poisson::Poisson() {
  mesh = NULL;
  initialize();
}

Poisson::~Poisson() {
  initialize();
}

void Poisson::initialize() {
  reconstructed = 0;
  if (mesh == NULL) {
    mesh = new CoredVectorMeshData;
  }
  else {
    delete mesh;
    mesh = NULL;
  }
  return;
}

int Poisson::setVertices(vector<Point> v) {
  vertices.clear();
  initialize();
  // reconstructed = 0;
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
  initialize();
  // reconstructed = 0;
  normals.resize(n.size());
  for (int i = 0; i < normals.size(); ++i) {
    normals[i].coords[0] = n[i].x; 
    normals[i].coords[1] = n[i].y; 
    normals[i].coords[2] = n[i].z;
  }
  return 1;
}

int Poisson::setParams(PoissonParam &p) {
  params = p;
  return 1;
}

int Poisson::getMesh(CoredVectorMeshData *m) {
  if (!ready()) {
    return 0;
  }
  m = mesh;
  return 1;
}

int Poisson::exportMesh(const char* modelPath) {
  fstream fs(modelPath, fstream::out);
  // // write unscaled out of core vertices
  // for (int i = 0; i < mesh->oocPoints.size(); ++i) {
  //   fs << "v " << mesh->oocPoints[i].coords[0] << " " << mesh->oocPoints[i].coords[1] << " " << mesh->oocPoints[i].coords[2] << endl;
  // }

  mesh->resetIterator(); // reset iterator
  Point3D<float> vertex;
  TriangleIndex tIndex;
  int inCoreFlag;
  int numVerticesInCore = mesh->inCorePoints.size(),
    numVerticesOutCore = mesh->oocPoints.size(),
    numFaces=mesh->triangleCount();

  // get and write correct scaled vertices coordinates
  for (int i = 0; i < numVerticesInCore; ++i) {
    vertex = mesh->inCorePoints[i];
    fs << "v " << vertex.coords[0] * scale + center.coords[0] << " " 
      << vertex.coords[1] * scale + center.coords[1] << " " 
      << vertex.coords[2] * scale + center.coords[2] << " " << endl;
  }
  for (int i = 0; i < numVerticesOutCore; ++i) {
    vertex = mesh->oocPoints[i];
    fs << "v " << vertex.coords[0] * scale + center.coords[0] << " " 
      << vertex.coords[1] * scale + center.coords[1] << " " 
      << vertex.coords[2] * scale + center.coords[2] << " " << endl;
  }

  // get and write correct ordered faces indexes
  for (int i = 0; i < numFaces; ++i) {
    mesh->nextTriangle(tIndex, inCoreFlag);
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[0])) {
      tIndex.idx[0]+=int(mesh->inCorePoints.size());
    }
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[1])) {
      tIndex.idx[1]+=int(mesh->inCorePoints.size());
    }
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[2])) {
      tIndex.idx[2]+=int(mesh->inCorePoints.size());
    }
    fs << "f " << tIndex.idx[0] + 1 << " " 
      << tIndex.idx[1] + 1 << " " 
      << tIndex.idx[2] + 1 << " " << endl;
  }

  fs.close();
  return 0;
}

int Poisson::testVcgFilter() {
  MyMesh m;
  
  mesh->resetIterator(); // reset iterator
  Point3D<float> vertex;
  TriangleIndex tIndex;
  int inCoreFlag;
  int numVerticesInCore = mesh->inCorePoints.size(),
    numVerticesOutCore = mesh->oocPoints.size(),
    numFaces=mesh->triangleCount();

  // get and write correct scaled vertices coordinates
  for (int i = 0; i < numVerticesInCore; ++i) {
    vertex = mesh->inCorePoints[i];
    vcg::tri::Allocator<MyMesh>::AddVertex(m,MyMesh::CoordType(
        vertex.coords[0] * scale + center.coords[0], 
        vertex.coords[1] * scale + center.coords[1], 
        vertex.coords[2] * scale + center.coords[2]));
  }
  for (int i = 0; i < numVerticesOutCore; ++i) {
    vertex = mesh->oocPoints[i];
    vcg::tri::Allocator<MyMesh>::AddVertex(m,MyMesh::CoordType(
        vertex.coords[0] * scale + center.coords[0], 
        vertex.coords[1] * scale + center.coords[1], 
        vertex.coords[2] * scale + center.coords[2]));
  }

  // get and write correct ordered faces indexes
  for (int i = 0; i < numFaces; ++i) {
    mesh->nextTriangle(tIndex, inCoreFlag);
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[0])) {
      tIndex.idx[0] += int(mesh->inCorePoints.size());
    }
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[1])) {
      tIndex.idx[1] += int(mesh->inCorePoints.size());
    }
    if (!(inCoreFlag & CoredMeshData::IN_CORE_FLAG[2])) {
      tIndex.idx[2] += int(mesh->inCorePoints.size());
    }
    vcg::tri::Allocator<MyMesh>::AddFace(m, tIndex.idx[0], tIndex.idx[1], tIndex.idx[2]); // vertice index start from 0 here
  }
  
  tri::UpdateTopology<MyMesh>::VertexFace(m);
  // test: laplacian smoothing for three iterations
  for(int i=0; i < 3; ++i)
  {
    tri::UpdateNormal<MyMesh>::PerFaceNormalized(m);
    tri::Smooth<MyMesh>::VertexCoordPasoDoble(m, 1, 0, 50, false);
  }
  tri::io::ExporterPLY<MyMesh>::Save(m, "out_s.ply");

  return 1;
}

int Poisson::ready() {
  return vertices.size() > 0 && normals.size() > 0 && vertices.size() == normals.size() && reconstructed;
}

int Poisson::apply() {
  reconstructed = Execute2(params, vertices, normals, *mesh, center, scale, cb);
  return reconstructed;
}
