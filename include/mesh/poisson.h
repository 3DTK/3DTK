/*------------------- 
** Poisson object (prototype)
** Author: Xia Sun
--------------------*/
#ifndef __POISSON_H__
#define __POISSON_H__

struct PoissonParam {
  int Depth;
};

class Poisson {
public:
  // public attributes
  
  // public methods
  Poisson();
  ~Poisson();
  int setPoints(std::vector<std::vector<float>> &v); // for IO, use v of v instead of v of float* to reach convinience of other developers
  int setNormals(std::vector<std::vector<float>> &n);
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
  std::vector<float*> vertices;     // model vertices (length4, with density value)
  std::vector<int*> faces;          // model faces
  std::vector<float*> tVertices;    // trimmed model vertices
  std::vector<int*> tFaces;         // trimmed model faces
  std::vector<float*> points;       // input pointset
  std::vector<float*> normals;      // input pointset normals
  float* center;
  float scale;

  // private methods
  void reset();
};

#endif // __POISSON_H__
