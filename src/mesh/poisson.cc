#include <iostream>
#include <string>
#include <vector>
#include <fstream>

#include "mesh/poisson.h"
#include "poisson/Ply.h"
#include "poisson/PoissonRecon.h"
#include "poisson/SurfaceTrimmer.h"

using namespace std;

// test mesh filtering with vcglib ---
// #include <vcg/complex/complex.h>
// #include <vcg/complex/algorithms/update/topology.h>
// #include <vcg/complex/algorithms/update/normal.h>
// #include <vcg/complex/algorithms/pointcloud_normal.h>
// #include <vcg/complex/algorithms/clean.h>
// #include <vcg/complex/algorithms/smooth.h>
// #include <wrap/io_trimesh/import.h>
// #include <wrap/io_trimesh/export_ply.h>
// #include <wrap/io_trimesh/export_obj.h>

// class MyFace;
// class MyVertex;
// struct MyUsedTypes : public UsedTypes<	Use<MyVertex>::AsVertexType, Use<MyFace>::AsFaceType>{};
// class MyVertex : public Vertex< MyUsedTypes, vertex::VFAdj, vertex::Coord3f, vertex::Normal3f, vertex::BitFlags  >{};
// class MyFace : public Face  < MyUsedTypes, face::VFAdj, face::Normal3f, face::VertexRef, face::BitFlags > {};
// class MyMesh : public vcg::tri::TriMesh<vector<MyVertex>, vector<MyFace> > {};
// test mesh filtering with vcglib ---

// int Execute2(PoissonParam &Par, vector<Point3D<float>> Pts, vector<Point3D<float>> Nor, CoredVectorMeshData &mesh, Point3D<float> &newCenter, float &newScale, vcg::CallBackPos *cb );

Poisson::Poisson() {
  // mesh = NULL;
  initialize();
}

Poisson::~Poisson() {
  initialize();
  for (int i = 0; i < points.size(); ++i) {
    delete [] points[i];
    delete [] normals[i];
  }
  for (int i = 0; i < vertices.size(); ++i) {
    delete [] vertices[i];
  }
  for (int i = 0; i < faces.size(); ++i) {
    delete [] faces[i];
  }
  for (int i = 0; i < tVertices.size(); ++i) {
    delete [] tVertices[i];
  }
  for (int i = 0; i < tFaces.size(); ++i) {
    delete [] tFaces[i];
  }
}

void Poisson::initialize() {
  reconstructed = 0;
  // if (mesh == NULL) {
  //   mesh = new CoredVectorMeshData;
  // }
  // else {
  //   delete mesh;
  //   mesh = NULL;
  // }
  return;
}

int Poisson::setPoints(vector<Point> v) {
  points.clear();
  initialize();
  // reconstructed = 0;
  points.resize(v.size());
  for (int i = 0; i < points.size(); ++i) {
    points[i] = new float[3];
    points[i][0] = v[i].x; 
    points[i][1] = v[i].y; 
    points[i][2] = v[i].z;
  }
  return 1;
}

int Poisson::setNormals(vector<Point> n) {
  normals.clear();
  initialize();
  // reconstructed = 0;
  normals.resize(n.size());
  for (int i = 0; i < normals.size(); ++i) {
    normals[i] = new float[3];
    normals[i][0] = n[i].x; 
    normals[i][1] = n[i].y; 
    normals[i][2] = n[i].z;
  }
  return 1;
}

int Poisson::setParams(PoissonParam &p) {
  Depth.set = true; Depth.value = p.Depth;
  return 1;
}

// int Poisson::getMesh(CoredFileMeshData<PlyValueVertex<Real>> *m) {
//   if (!ready()) {
//     return 0;
//   }
//   m = &mesh;
//   return 1;
// }

int Poisson::exportMesh(const char* modelPath) {
  fstream fs(modelPath, fstream::out);
  // // write unscaled out of core points
  // for (int i = 0; i < mesh->oocPoints.size(); ++i) {
  //   fs << "v " << mesh->oocPoints[i].coords[0] << " " << mesh->oocPoints[i].coords[1] << " " << mesh->oocPoints[i].coords[2] << endl;
  // }

  // get and write correct scaled points coordinates
  for (int i = 0; i < vertices.size(); ++i) {
    fs << "v " << vertices[i][0] << " " 
      << vertices[i][1] << " " 
      << vertices[i][2] << " " << endl;
  }

  // get and write correct ordered faces indexes
  for (int i = 0; i < faces.size(); ++i) {
    // if (removedFaces[i]) {
    //   continue;
    // }
    fs << "f " << faces[i][0] + 1 << " " 
      << faces[i][1] + 1 << " " 
      << faces[i][2] + 1 << " " << endl;
  }

  fs.close();
  return 0;
}

int Poisson::exportTrimmedMesh(const char* modelPath) {
  fstream fs(modelPath, fstream::out);
  // // write unscaled out of core points
  // for (int i = 0; i < mesh->oocPoints.size(); ++i) {
  //   fs << "v " << mesh->oocPoints[i].coords[0] << " " << mesh->oocPoints[i].coords[1] << " " << mesh->oocPoints[i].coords[2] << endl;
  // }

  // get and write correct scaled points coordinates
  for (int i = 0; i < tVertices.size(); ++i) {
    fs << "v " << tVertices[i][0] << " " 
      << tVertices[i][1] << " " 
      << tVertices[i][2] << " " << endl;
  }

  // get and write correct ordered faces indexes
  for (int i = 0; i < tFaces.size(); ++i) {
    // if (removedFaces[i]) {
    //   continue;
    // }
    fs << "f " << tFaces[i][0] + 1 << " " 
      << tFaces[i][1] + 1 << " " 
      << tFaces[i][2] + 1 << " " << endl;
  }

  fs.close();
  return 0;
}

// temp removed
int Poisson::testVcgFilter() {
  // MyMesh m;
  
  // mesh->resetIterator(); // reset iterator
  // Point3D<float> vertex;
  // TriangleIndex tIndex;
  // int inCoreFlag;
  // int numPointsInCore = mesh->inCorePoints.size(),
  //   numPointsOutCore = mesh->oocPoints.size(),
  //   numFaces=mesh->triangleCount();

  // // set vcg model vertices with correct scaled points coordinates
  // for (int i = 0; i < vertices.size(); ++i) {
  //   vertex = mesh->inCorePoints[i];
  //   vcg::tri::Allocator<MyMesh>::AddVertex(m,MyMesh::CoordType(
  //       get<0>(vertices[i]), 
  //       get<1>(vertices[i]), 
  //       get<2>(vertices[i])
  //   ));
  // }

  // // return 1; // testing purpose
  // // set vcg model faces with correct faces indexes
  // for (int i = 0; i < faces.size(); ++i) {
  //   mesh->nextTriangle(tIndex, inCoreFlag);
  //   vcg::tri::Allocator<MyMesh>::AddFace(m, get<0>(faces[i]), get<1>(faces[i]), get<2>(faces[i]));
  // }
  
  // tri::UpdateTopology<MyMesh>::VertexFace(m);
  // // test: laplacian smoothing for three iterations
  // for(int i=0; i < 3; ++i)
  // {
  //   tri::UpdateNormal<MyMesh>::PerFaceNormalized(m);
  //   tri::Smooth<MyMesh>::VertexCoordPasoDoble(m, 1, 0, 50, false);
  // }
  // tri::io::ExporterPLY<MyMesh>::Save(m, "out_s.ply");

  // return 1;
}

// temp moved to apply
int Poisson::updateModel() {
  // mesh.resetIterator();
  // // get vertices
  // for (int i = 0; i < mesh.inCorePoints.size(); ++i) {
  //   // v[3] = mesh.inCorePoints[i].value; // density value
  //   vertices.push_back(make_tuple(
  //     mesh.inCorePoints[i].point.coords[0], 
  //     mesh.inCorePoints[i].point.coords[1], 
  //     mesh.inCorePoints[i].point.coords[2]
  //   ));
  // }
  // for (int i = 0; i < mesh.outOfCorePointCount(); ++i) {
  //   PlyValueVertex< Real > vt;
  //   mesh.nextOutOfCorePoint(vt);
  //   // v[3] = vt.value; // density value
  //   vertices.push_back(make_tuple(
  //     vt.point.coords[0], 
  //     vt.point.coords[1], 
  //     vt.point.coords[2]
  //   ));
  // }
  // // get faces
  // for (int i = 0; i < mesh.polygonCount(); ++i) {
  //   int f[3];
  //   vector<CoredVertexIndex> face;
  //   mesh.nextPolygon(face);
  //   for (int j = 0; j < face.size(); ++j) {
  //     if (face[j].inCore) {
  //       f[j] = face[j].idx;
  //     }
  //     else {
  //       f[j] = face[j].idx + (int)(mesh.inCorePoints.size());
  //     }
  //   }
  //   faces.push_back(make_tuple(
  //     f[0], f[1], f[2]
  //   ));
  // }

  // return 1;
}

// for each face, find the nearest points to its center, 
// remove this face if the distance is larger than a certain `maxDist`
int Poisson::distFilter(float maxDist) {
  // // int record = -1;
  // removedFaces.resize(faces.size());
  // for (int i = 0; i < faces.size(); ++i) {
  //   // cx, cy, cz: center of face
  //   float cx = (get<0>(vertices[get<0>(faces[i])]) + get<0>(vertices[get<1>(faces[i])]) + get<0>(vertices[get<2>(faces[i])])) / 3, 
  //     cy = (get<1>(vertices[get<0>(faces[i])]) + get<1>(vertices[get<1>(faces[i])]) + get<1>(vertices[get<2>(faces[i])])) / 3, 
  //     cz = (get<2>(vertices[get<0>(faces[i])]) + get<2>(vertices[get<1>(faces[i])]) + get<2>(vertices[get<2>(faces[i])])) / 3;
  //   float dist = numeric_limits<float>::max();
  //   for (int j = 0; j < points.size(); ++j) {
  //     float px = points[j].coords[0],
  //       py = points[j].coords[1],
  //       pz = points[j].coords[2];
  //     float d = sqrt(pow(px - cx, 2) + pow(py - cy, 2) + pow(pz - cz, 2));
  //     dist = d < dist ? d : dist;
  //   }
  //   // record = dist > record ? dist : record;
  //   removedFaces[i] = dist > maxDist ? 1 : 0;
  // }
  // return 1;
}

// filter mesh with density value
// implementation in SurfaceTrimmer.h
int Poisson::surfaceTrimmer(float dstVal) {
  CallSurfaceTrimmer(dstVal, vertices, faces, tVertices, tFaces);
  return 1;
}

// not used now, temp removed
int Poisson::calcNormalVcg() {
  // MyMesh m;
  // MyMesh::VertexIterator vi = vcg::tri::Allocator<MyMesh>::AddVertices(m,3);

  // int mask = vcg::tri::io::Mask::IOM_VERTNORMAL;
  // tri::io::ImporterOBJ<MyMesh>::Open(m, "dat/test/pointsn.obj", mask);

  // // tri::UpdateNormal<MyMesh>::PerVertex(m);
  // tri::PointCloudNormal<MyMesh>::Param p;
  // p.smoothingIterNum = 2;
  // p.useViewPoint = true;
  // tri::PointCloudNormal<MyMesh>::Compute(m, p);
  // for (vi = m.vert.begin(); vi != m.vert.end(); ++vi) {
  //   auto v = *vi;
  //   auto n = v.N();
  //   n[0]; n[1]; n[2];
  // }

  // tri::io::ExporterOBJ<MyMesh>::Save(m, "dat/test/points_out.obj", mask);

  // return 1;
}

int Poisson::ready() {
  return points.size() > 0 && normals.size() > 0 && points.size() == normals.size() && reconstructed;
}

int Poisson::apply() {
  CoredFileMeshData<PlyValueVertex<float>> mesh;
  reconstructed = Execute< 2 , PlyValueVertex< Real > , true  >( points, normals, mesh );
  if (reconstructed) {
    // updateModel();
    mesh.resetIterator();
    // get vertices
    for (int i = 0; i < mesh.inCorePoints.size(); ++i) {
      float *v = new float[4];
      v[0] = mesh.inCorePoints[i].point.coords[0];
      v[1] = mesh.inCorePoints[i].point.coords[1];
      v[2] = mesh.inCorePoints[i].point.coords[2];
      v[3] = mesh.inCorePoints[i].value;
      vertices.push_back(v);
      // vertices.push_back(make_tuple(
      //   mesh.inCorePoints[i].point.coords[0], 
      //   mesh.inCorePoints[i].point.coords[1], 
      //   mesh.inCorePoints[i].point.coords[2]
      // ));
    }
    for (int i = 0; i < mesh.outOfCorePointCount(); ++i) {
      PlyValueVertex< Real > vt;
      mesh.nextOutOfCorePoint(vt);
      float *v = new float[4];
      v[0] = vt.point.coords[0];
      v[1] = vt.point.coords[1];
      v[2] = vt.point.coords[2];
      v[3] = vt.value;
      vertices.push_back(v);
      // vertices.push_back(make_tuple(
      //   vt.point.coords[0], 
      //   vt.point.coords[1], 
      //   vt.point.coords[2]
      // ));
    }
    // get faces
    for (int i = 0; i < mesh.polygonCount(); ++i) {
      int *f = new int[3];
      vector<CoredVertexIndex> face;
      mesh.nextPolygon(face);
      for (int j = 0; j < face.size(); ++j) {
        if (face[j].inCore) {
          f[j] = face[j].idx;
        }
        else {
          f[j] = face[j].idx + (int)(mesh.inCorePoints.size());
        }
      }
      faces.push_back(f);
    }
  }
  return reconstructed;
}
