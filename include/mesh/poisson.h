/*-------------------
** Poisson object (prototype)
** Author: Xia Sun
--------------------*/
#ifndef __POISSON_H__
#define __POISSON_H__

struct PoissonParam {
  int Depth = 8;
  float Trim = 0.0;
  float samplesPerNode = 1.0;
  bool UseColor = false;
  bool ExportNormal = true;
};

class Poisson {
public:
  // public attributes

  // public methods
  Poisson();
  ~Poisson();
  int setPoints(std::vector<std::vector<float>> &v); // for IO, use v of v instead of v of float* to reach convinience of other developers
  int setNormals(std::vector<std::vector<float>> &n);
  int setColors(std::vector<std::vector<float>> &c);
  int getVertices(std::vector<std::vector<float>> &v);
  int getFaces(std::vector<std::vector<int>> &f);
  int setParams(PoissonParam &p);
  int apply();
  int surfaceTrimmer(float dstVal);
  int exportMesh(const char *modelPath);
  int exportTrimmedMesh(const char *modelPath);

private:
  // private attributes
  int reconstructed;
  int updated;
  int trimmed;
  PoissonParam params;
  std::vector<float*> vertices;     // model vertices (length 10, x y z density nx ny nz r g b)
  std::vector<int*> faces;          // model faces
  std::vector<float*> tVertices;    // trimmed model vertices
  std::vector<int*> tFaces;         // trimmed model faces
  std::vector<float*> points;       // input pointset
  std::vector<float*> normals;      // input pointset normals
  std::vector<float*> colors;       // input pointset colors
  float* center;
  float scale;

  // private methods
  void reset();
};

#endif // __POISSON_H__
