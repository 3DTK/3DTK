#include "mesh/calc_normals.h"

using namespace std;

void calcNormals(vector<Point> &points, vector<Point> &normals, normal_method ntype, int k1, int k2, int width, int height, const double* rPos, const double* rPosTheta, Scan *scan) {
  if (ntype == KNN)
    calculateNormalsKNN(normals, points, k1, rPos);
  else if (ntype == ADAPTIVE_KNN)
    calculateNormalsAdaptiveKNN(normals, points, k1, k2, rPos);
  else if (ntype == AKNN)
    calculateNormalsApxKNN(normals, points, k1, rPos);
  else if (ntype == ADAPTIVE_AKNN)
    calculateNormalsAdaptiveApxKNN(normals, points, k1, k2, rPos);
  else
  {
#ifdef WITH_OPENCV
    // create panorama
    fbr::panorama fPanorama(width, height, fbr::EQUIRECTANGULAR,
                            1, 0, fbr::EXTENDED);
    fPanorama.createPanorama(scan2mat(scan));

    if(ntype == PANORAMA)
      calculateNormalsPANORAMA(normals,
                              points,
                              fPanorama.getExtendedMap(),
                              rPos);
    else if(ntype == PANORAMA_FAST)
      calculateNormalsFAST(normals,
                          points,
                          fPanorama.getRangeImage(),
                          fPanorama.getMaxRange(),
                          rPos,
                          fPanorama.getExtendedMap());
#endif
  }
}