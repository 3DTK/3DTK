#ifndef IMAGE_H
#define IMAGE_H

#include <vector>
#include "slam6d/point.h"

using std::vector;

class Image {
 
public:
  Image(float _minw, float _maxw, float _minh, float _maxh, float _resolution,
  const vector<Point> *points = 0);
  Image(float _minw, float _maxw, float _minh, float _maxh, float _resolution, const vector<double*> *points = 0);
  Image(float _minw, float _maxw, float _minh, float _maxh, float _resolution, double * const* points, int size);
  int getWidth();
  int getHeight();
  float getValueAt(int width, int height);
  float getMin();
  float getMax();

  int calcMarker(float threshold, int ** regdat);
  void mergeregion(int x, int y, int delreg, int targetreg, int** img, int it = 0);
  int blobColor(float dist, int ** regdat);
  int cluster(float dist, int** dat, int ** regdat);
  void writeCenters(int regions, int** clusters, const vector<Point> *points);
  void printScans(int ** regdat, double * const* points, int size); 
  void printImage(const char* filename, bool color, int** img, int height, int width, int min, int max);
  void printImage(const char *filename, bool color);
  ~Image();

private:
  int width;
  int height;
  float minw;
  float maxw;
  float minh;
  float maxh;
  float threshold;
  float ** data;
};

#endif
