#include "mesh/recon_commons.h"

using namespace std;

/**
  * Convert from point to float[] and from left-handed to right-handed coordinate system
  */
void convert(vector<Point> &src, vector<vector<float>> &dst) {
  dst.resize(src.size());
  for (int i = 0; i < src.size(); ++i) {
    dst[i].resize(3);
    dst[i][0] = src[i].z;
    dst[i][1] = -src[i].x;
    dst[i][2] = src[i].y;
  }
}
