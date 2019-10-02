#ifndef __CALC_NORMALS_H__
#define __CALC_NORMALS_H__

#include <iostream>
#include <vector>
#include <slam6d/normals.h>
#ifdef WITH_OPENCV
#include <normals/normals_panorama.h>
#endif

enum normal_method {KNN, ADAPTIVE_KNN,
				AKNN, ADAPTIVE_AKNN,
#ifdef WITH_OPENCV
				PANORAMA, PANORAMA_FAST
#endif
};

// calculate normals
void calcNormals(
  std::vector<Point> &points, std::vector<Point> &normals, normal_method ntype,
  int k1, int k2, int width, int height, const double* rPos, const double* rPosTheta, Scan *scan);

#endif