#include "mesh/mesh.h"

using namespace std;

Mesh::Mesh() {
  initialize();
}

Mesh::~Mesh() {
  initialize();
  delete [] bbox;
}

// init all attributes with default value
void Mesh::initialize() {
  empty = true;
  for (float *v : vertices) {
    delete [] v;
  }
  for (int *f : faces) {
    delete [] f;
  }
  for (float *n : normals) {
    delete [] n;
  }
  for (float *ov : oVertices) {
    delete [] ov;
  }
  vertices.clear();
  faces.clear();
  normals.clear();
  oVertices.clear();
  numVertices = 0;
  numFaces = 0;
  numNormals = 0;
  delete [] bbox;
  bbox = new float[6] {0};
}

// load mesh model from file
void Mesh::load(string modelPath) {
  if (numVertices) {
    initialize();
  }
  // read .obj
  ifstream ifs(modelPath.c_str());
  if (!ifs.good()) {
    cout << "model file does not exist" << endl;
    return;
  }
  cout << "model file found" << endl;
  string line;
  while (getline(ifs, line)) {
    istringstream iss(line);
    string lineHead;
    float lineVal1f, lineVal2f, lineVal3f;
    int lineVal1i, lineVal2i, lineVal3i;
    iss >> lineHead;
    if (lineHead == "v") {
      iss >> lineVal1f >> lineVal2f >> lineVal3f;
      vertices.push_back(new float[3] {lineVal1f, lineVal2f, lineVal3f});
      updateBBox(lineVal1f, lineVal2f, lineVal3f); // update bbox with new vertex
    }
    else if (lineHead == "f") {
      iss >> lineVal1i >> lineVal2i >> lineVal3i;
      faces.push_back(new int[3] {lineVal1i, lineVal2i, lineVal3i});
    }
    else if (lineHead == "n") {
      iss >> lineVal1f >> lineVal2f >> lineVal3f;
      normals.push_back(new float[3] {lineVal1f, lineVal2f, lineVal3f});
    }
    else {
      continue;
    }
  }
  // after loading mesh model info
  numVertices = vertices.size();
  numFaces = faces.size();
  numNormals = normals.size();
  sortVertices();
}

// write mesh model to file
void Mesh::save(std::string modelPath) {
  if (!numVertices) {
    return;
  }
  // write .obj
  ofstream ofs(modelPath.c_str());
  string line;
  for (float *v : vertices) {
    line = "v " + to_string(v[0]) + " " + to_string(v[1]) + " " + to_string(v[2]);
    ofs << line << endl;
  }
  for (int *f : faces) {
    line = "f " + to_string(f[0]) + " " + to_string(f[1]) + " " + to_string(f[2]);
    ofs << line << endl;
  }
  for (float *n : normals) {
    line = "n " + to_string(n[0]) + " " + to_string(n[1]) + " " + to_string(n[2]);
    ofs << line << endl;
  }
  ofs.close();
}

// sort vertices into triangles for easy rendering
void Mesh::sortVertices() {
  oVertices.clear();
  oVertices.resize(numFaces);
  for (int i = 0; i < numFaces; ++i) {
    int vIdx1 = faces[i][0] - 1, vIdx2 = faces[i][1] - 1, vIdx3 = faces[i][2] - 1;
    oVertices[i] = new float[9] {
      vertices[vIdx1][0], vertices[vIdx1][1], vertices[vIdx1][2],
      vertices[vIdx2][0], vertices[vIdx2][1], vertices[vIdx2][2],
      vertices[vIdx3][0], vertices[vIdx3][1], vertices[vIdx3][2]
    };
  }
  return;
}

// get sorted vertices
vector<float*> Mesh::getOrderedVertices() {
  if (!oVertices.size()) {
    // err
    throw;
  }
  return oVertices;
}

// update bounding box of model with a new vertex
void Mesh::updateBBox(float vX, float vY, float vZ) {
  // assign initial bbox with first vertex
  // then update accordingly
  if (empty) {
    delete [] bbox;
    bbox = new float[6] {vX, vX, vY, vY, vZ, vZ};
    empty = false;
  }
  else {
    bbox[0] = min(bbox[0], vX); bbox[1] = max(bbox[1], vX);
    bbox[2] = min(bbox[2], vY); bbox[3] = max(bbox[3], vY);
    bbox[4] = min(bbox[4], vZ); bbox[5] = max(bbox[5], vZ);
  }
}

float* Mesh::getBBox() {
  return bbox;
}
