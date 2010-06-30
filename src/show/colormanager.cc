#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#include <windows.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "colormanager.h"
#include "stdio.h"
#include "../globals.icc"
#include "float.h"

//#include "math.h"

ColorManager::ColorManager(unsigned int _buckets, unsigned int types, unsigned int pointdim, float *_mins, float *_maxs)
  : buckets(_buckets) {
  
  colormap = new float*[buckets + 1];  // allow a color more for values equal to max
  for (unsigned int i = 0; i < buckets; i++) {
    colormap[i] = new float[3];
  }
  colormap[buckets] = new float[3];


 
  mins = new float[pointdim];
  maxs = new float[pointdim];
  for (unsigned int i = 0; i < pointdim; i++) { 
    mins[i] = _mins[i];
    maxs[i] = _maxs[i];
  }

  setCurrentDim(0);

}

ColorManager::~ColorManager() {
  for (unsigned int i = 0; i < buckets; i++) {
    delete[] colormap[i];
  }
  delete[] colormap[buckets];
  delete[] colormap;
  
  delete[] mins;
  delete[] maxs;
}

void ColorManager::setColorMap(ColorMap &cm) {
  for (unsigned int i = 0; i < buckets; i++) {
    cm.calcColor(colormap[i], i, buckets);
  }
  cm.calcColor(colormap[buckets], buckets-1, buckets);
}

void ColorManager::setCurrentDim(unsigned int cdim) {
  currentdim = cdim;
  makeValid();
}

void ColorManager::setColor(double *val) {
  int index = toIndex(val);
  glColor3f( colormap[index][0], colormap[index][1], colormap[index][2] );
}
    
unsigned int ColorManager::toIndex(double *val) {
  double value = val[currentdim];
  if (value < min) return 0;
  if (value > max) return buckets;
  return (unsigned int)((value-min)/extentbuckets);
  //return (int)((val[currentdim]-min)/extentbuckets);
}
    
void ColorManager::setMinMax(float _min, float _max) {
  if (_min < _max) {
    min = _min;
    max = _max;
  }
  float extent = max - min;
  extentbuckets = extent/(float)buckets;
  //setColorMap(current_cm);
}
    
void ColorManager::makeValid() {
  min = mins[currentdim];
  max = maxs[currentdim];
  
  float extent = max - min;
  extentbuckets = extent/(float)buckets;
}
    



void ColorMap::convert_hsv_to_rgb(float hue, float s, float v,  
    float &r, float &g, float &b) {
  float p1, p2, p3, i, f;
  float xh;

  if (hue == 360.0)
    hue = 0.0;           

  xh = hue / 60.;                  // convert hue to be in 0,6     
  i = (float)floor((double)xh);    // i is greatest integer smaller than h  
  f = xh - i;                      // f is the fractional part of h 
  p1 = v * (1 - s);
  p2 = v * (1 - (s * f));
  p3 = v * (1 - (s * (1 - f)));

  switch ((int) i)
  {
    case 0:
      r = v;
      g = p3;
      b = p1;
      break;
    case 1:
      r = p2;
      g = v;
      b = p1;
      break;
    case 2:
      r = p1;
      g = v;
      b = p3;
      break;
    case 3:
      r = p1;
      g = p2;
      b = v;
      break;
    case 4:
      r = p3;
      g = p1;
      b = v;
      break;
    case 5:
      r = v;
      g = p1;
      b = p2;
      break;
  }

}
ColorMap ColorMap::getColorMap(CM map) {
  switch(map) {
    case SOLID:
      return ColorMap();
      break;
    case GREY:
      return GreyMap();
      break;
    case HSV:
      return HSVMap();
      break;
    case JET:
      return JetMap();
      break;
    case HOT:
      return HotMap();
      break;
    default:
      return ColorMap();
      break;
  }
}
