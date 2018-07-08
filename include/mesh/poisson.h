/*------------------- 
** Poisson object (prototype)
** Author: Xia Sun
--------------------*/
#ifndef __MESH_H__
#define __MESH_H__

#ifndef WITH_OPENCV
#define WITH_TEST

#include <slam6d/scan.h>
#include <slam6d/normals.h>

#ifdef WITH_OPENCV
#include <normals/normals_panorama.h>
#endif

// #include "../../src/mesh/poisson/PoissonRecon.h"

struct PoissonParam {
  int Depth;
};

class Poisson {
public:
  // public attributes
  
  // public methods
  Poisson();
  ~Poisson();
  int setPoints(std::vector<Point> v);
  int setNormals(std::vector<Point> n);
  int setParams(PoissonParam &p);
  // int getMesh(CoredFileMeshData<PlyValueVertex<float>> *m);
  int apply();
  int distFilter(float maxDist); // filter reconstructed model
  int testVcgFilter(); // test mesh processing with vcglib
  int calcNormalVcg();
  int exportMesh(const char *modelPath);

private:
  // private attributes
  int reconstructed;
  PoissonParam params;
  std::vector<std::tuple<float, float, float>> vertices; // model vertices
  std::vector<std::tuple<int, int, int>> faces; // model faces
  std::vector<int> removedFaces;
  std::vector<float *> points; // pointset
  std::vector<float *> normals; // pointset normals
  // CoredFileMeshData<PlyValueVertex<float>> mesh;
  float* center;
  float scale;

  // private methods
  void initialize();
  int ready();
  int updateModel(); // update poisson vertices and faces data structure to stl based ones
};

#endif
#endif
