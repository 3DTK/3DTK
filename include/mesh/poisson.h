/*------------------- 
** Poisson object (prototype)
** Author: Xia Sun
--------------------*/
#ifndef __MESH_H__
#define __MESH_H__

#ifndef WITH_OPENCV
#define WITH_OPENCV

#include <slam6d/scan.h>
#include <slam6d/normals.h>

#ifdef WITH_OPENCV
#include <normals/normals_panorama.h>
#endif

class Poisson {
public:
  // public attributes
  
  
  // public methods
  Poisson();
  ~Poisson();
  int setPoints(std::vector<Point> v);
  int setNormals(std::vector<Point> n);
  int setParams(PoissonParam &p);
  int getMesh(CoredVectorMeshData *m);
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
  std::vector<Point3D<float>> points; // pointset
  std::vector<Point3D<float>> normals; // pointset normals
  CoredVectorMeshData *mesh;
  Point3D<float> center;
  float scale;
  vcg::CallBackPos *cb;

  // private methods
  void initialize();
  int ready();
  int updateModel(); // update poisson vertices and faces data structure to stl based ones
};

#endif
#endif
