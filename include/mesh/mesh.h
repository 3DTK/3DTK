/* Mesh model object
* Author: Xia Sun
*/
#ifndef __MESH_H__
#define __MESH_H__

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <limits>
#include <algorithm>

#include <GL/glut.h>

class Mesh {
public:
  // public attributes
  bool empty;
  int numVertices;
  int numFaces;
  int numNormals;

  // public methods
  Mesh();
  ~Mesh();
  void load(std::string modelPath);
  void save(std::string modelPath);
  std::vector<float*> getOrderedVertices();
  float* getBBox();

private:
  // private attributes
  std::vector<float*> oVertices; // ordered vertices, each float* means 9 verices of a triangle face
  std::vector<float*> vertices;
  std::vector<int*> faces;
  std::vector<float*> normals;
  float *bbox; // x y z min/max

  // private methods
  void initialize();
  void sortVertices();
  void updateBBox(float vX, float vY, float vZ);
};

#endif
