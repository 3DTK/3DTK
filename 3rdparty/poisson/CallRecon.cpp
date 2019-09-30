/*
Author: Xia Sun @ Zhejiang University
This code demonstrates how to use vectors of points normal as input data
to call Poisson Surface Reconstruction, and retrieve reconstructed mesh data
into vector of vertices, faces, colors(optional), and density value (optional)
*/

#include <iostream>
#include <vector>
#include <string>

#include <stdio.h>
#include <string.h>

#include "Ply.h"
#include "FEMTree.h"
#include "PoissonRecon.h"
// #include "SurfaceTrimmer.h"

struct Argument {
  std::string inPath;
  std::string outPath;
  int depth = 10;
  float trimVal = 0;
  bool density = false;
};

using namespace std;

int simpleCmdParse( int argc, char* argv[], Argument &arg );
int argValidate( Argument &arg );
int readPoints( const char *inDir, vector<float*> &pts, vector<float*> &norms, vector<float*> &colors);
int writeModel( const char *outDir, vector<float*> &vts, vector<int*> &faces);

int main( int argc , char* argv[] ) {
  Argument arg;
  const static unsigned int Dim = 3;
  // input points and normals
  vector<float*> pts; 
  vector<float*> norms;
  vector<float*> colors;
  // output vertices and faces
  vector<float*> vts; // x, y, z, and density value
  vector<float*> tVts; // x, y, z, and density value
  vector<int*> faces; 
  vector<int*> tFaces; 
  // CoredFileMeshData< PlyValueVertex< Real > > mesh;
  float scale;

  // parse and validate cmd arguments, for demo purpose
  if (!simpleCmdParse(argc, argv, arg)) {
    return 1;
  }
  if (!argValidate(arg)) {
    return 1;
  }

  // read data
  readPoints(arg.inPath.c_str(), pts, norms, colors);

  // set parameters
  {
    In.set = true; In.value = new char[arg.inPath.size() + 1]; strcpy(In.value, arg.inPath.c_str()); // must use this method for cmdString
    Out.set = true; Out.value = new char[arg.outPath.size() + 1]; strcpy(Out.value, arg.outPath.c_str());
    Depth.set = true; Depth.value = arg.depth;
    Density.set = arg.density;
    Colors.set = true;
    Normals.set = true;
  }

  // call poisson surface reconstruction
  if( arg.density ) {
    Execute< float, PointStreamColor< float > >(pts, norms, colors, IsotropicUIntPack< Dim , FEMDegreeAndBType< 1 , BOUNDARY_NEUMANN >::Signature >());
    // Execute< 3, float, PointStreamColor< float > >(argc, argv);
    // Execute< 2 , PlyValueVertex< Real > , true  >( pts, norms, mesh );
  }
	else {
    // Execute< 2 ,      PlyVertex< Real > , false >( pts, norms, mesh );
  }
  
  // retrieve vertices, density value (optional) and faces from mesh
  {
    // mesh.resetIterator();
    // // get vertices
    // for (int i = 0; i < mesh.inCorePoints.size(); ++i) {
    //   float *v = new float[4];
    //   v[0] = mesh.inCorePoints[i].point.coords[0]; // x
    //   v[1] = mesh.inCorePoints[i].point.coords[1]; // y
    //   v[2] = mesh.inCorePoints[i].point.coords[2]; // z
    //   v[3] = mesh.inCorePoints[i].value; // density value
    //   vts.push_back(v);
    // }
    // for (int i = 0; i < mesh.outOfCorePointCount(); ++i) {
    //   float *v = new float[4];
    //   PlyValueVertex< Real > vt;
    //   mesh.nextOutOfCorePoint(vt);
    //   v[0] = vt.point.coords[0];
    //   v[1] = vt.point.coords[1];
    //   v[2] = vt.point.coords[2];
    //   v[3] = vt.value;
    //   vts.push_back(v);
    // }
    // // get faces
    // for (int i = 0; i < mesh.polygonCount(); ++i) {
    //   int *f = new int[3];
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
    //   faces.push_back(f);
    // }
  }

  // call SurfaceTrimmer
  // CallSurfaceTrimmer(arg.trimVal, vts, faces, tVts, tFaces);

  // write to file
  // writeModel(arg.outPath.c_str(), tVts, tFaces);
  
  // clear memory
  for (int i = 0; i < pts.size(); ++i) {
    delete [] pts[i];
    delete [] norms[i];
  }
  for (int i = 0; i < vts.size(); ++i) {
    delete [] vts[i];
  }
  for (int i = 0; i < faces.size(); ++i) {
    delete [] faces[i];
  }
  for (int i = 0; i < tVts.size(); ++i) {
    delete [] tVts[i];
  }
  for (int i = 0; i < tFaces.size(); ++i) {
    delete [] tFaces[i];
  }

  return 0;
}

int simpleCmdParse( int argc, char* argv[], Argument &arg ) {
  for (int i = 1; i < argc; ++i) {
    if (!strcmp(argv[i], "--in") && i + 1 < argc) {
      arg.inPath = argv[i + 1];
      ++i;
      continue;
    }
    if (!strcmp(argv[i], "--out") && i + 1 < argc) {
      arg.outPath = argv[i + 1];
      ++i;
      continue;
    }
    if (!strcmp(argv[i], "--depth") && i + 1 < argc) {
      arg.depth = atoi(argv[i + 1]);
      ++i;
      continue;
    }
    if (!strcmp(argv[i], "--trim") && i + 1 < argc) {
      arg.trimVal = atof(argv[i + 1]);
      ++i;
      continue;
    }
    if (!strcmp(argv[i], "--density")) {
      arg.density = true;
      continue;
    }
    cout << "Wrong input arguments!" << endl
      << "Should contain at least: " << endl 
      << "------" << endl
      << "--in (input file path)" << endl 
      << "--out (output file path)" << endl
      << "--depth (octree depth, default 10)" << endl
      << "--trim (value of trimmer threshold, default 0)" << endl
      << "--density (whether to have density, default true)" << endl
      << "------" << endl
      << "For example: " << endl
      << "./CallRecon --in Data/sphere_points_cut.xyz --out Data/sphere_reconmesh_cut.ply --depth 12 --density" << endl;
    return 0;
  }
  return 1;
}

int argValidate( Argument &arg ) {
  if (arg.inPath.size() == 0) {
    cout << "--in (input file path) required" << endl;
    return 0;
  }
  if (arg.outPath.size() == 0) {
    cout << "--out (output file path) required" << endl;
    return 0;
  }
  return 1;
}

int readPoints( const char *inDir, vector<float*> &pts, vector<float*> &norms, vector<float*> &colors ) {
  // using c style io
  FILE *fp;
  fp = fopen(inDir, "r");
  if (fp == NULL) {
    cout << "Fail to open input file! Please have a check." << endl;
    return 0;
  }
  float x, y, z, nx, ny, nz, r, g, b, a;
  int e;
  while ((e = fscanf(fp, "%f %f %f %f %f %f %f %f %f %f", &x, &y, &z, &nx, &ny, &nz, &r, &g, &b, &a)) != EOF) {
    float *p = new float[3]; p[0] = x, p[1] = y, p[2] = z;
    float *n = new float[3]; n[0] = nx, n[1] = ny, n[2] = nz;
    float *c = new float[3]; c[0] = r, c[1] = g, c[2] = b; // discard alpha value
    pts.push_back(p);
    norms.push_back(n);
    colors.push_back(c);
  }
  if (fclose(fp) != 0) {
    cout << "Fail to close file! Please have a check." << endl;
  }
  return 1;
}

int writeModel( const char *outDir, vector<float*> &vts, vector<int*> &faces) {
  // using c style io
  FILE *fp;
  fp = fopen(outDir, "w");
  if (fp == NULL) {
    cout << "Fail to create output file! Please have a check." << endl;
    return 0;
  }
  for (int i = 0; i < vts.size(); ++i) {
    fprintf(fp, "v %f %f %f\n", vts[i][0], vts[i][1], vts[i][2]);
  }
  for (int i = 0; i < faces.size(); ++i) {
    fprintf(fp, "f %d %d %d\n", faces[i][0] + 1, faces[i][1] + 1, faces[i][2] + 1);
  }
  if (fclose(fp) != 0) {
    cout << "Fail to close created file! Please have a check." << endl;
  }
  return 1;
}
