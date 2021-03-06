#ifndef __NNPARAMS_H__
#define __NNPARAMS_H__

struct NNParams {
/**
   * pointer to the closest point.  size = 4 bytes of 32 bit machines
   */
  void *closest;

  /**
   * distance to the closest point. size = 8 bytes
   */
  double closest_d2;

  // distance to the closest point in voxels
  int closest_v;

  // location of the query point in voxel coordinates
  int x;
  int y;
  int z;

  //direction for search along dir
  float dir[3];
  int dir_v[3];
  int maxDistDir_v;
  double maxDistDir;

  //Percent of noise for search along dir
  double minDistR;
  int minDistR_v;

  /**
   * pointer to the point, size = 4 bytes of 32 bit machines
   */
  double *p;

  /**
   * expand to 128 bytes to avoid false-sharing, 16 bytes from above + 28*4 bytes = 128 bytes
   */
//  int padding[24];

  int count;
  int max_count;

};
struct NNPoints{
  std::vector<float*> knnRangePoints;
};
#endif
