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

ColorManager::ColorManager(unsigned int _buckets, unsigned int types)
  : buckets(_buckets) {
  
  colormap = new float*[buckets + 1];  // allow a color more for values equal to max
  for (unsigned int i = 0; i < buckets; i++) {
    colormap[i] = new float[3];
  }
  colormap[buckets] = new float[3];

  pointdim = 3; // at least 3 dimensions
  if (types & USE_REFLECTANCE) pointdim++;
  if (types & USE_AMPLITUDE)   pointdim++;
  if (types & USE_DEVIATION)   pointdim++;

 
  mins = new float[pointdim];
  maxs = new float[pointdim];
  for (unsigned int i = 0; i < pointdim; i++) { 
    mins[i] = FLT_MAX;
    maxs[i] = FLT_MIN;
  }
  dimensionmap[1] = dimensionmap[2] = dimensionmap[3] = 1; // choose height per default  
  
  dimensionmap[0] = 1;  // height 
  
  int counter = 3;
  if (types & USE_REFLECTANCE) dimensionmap[1] = counter++;  
  if (types & USE_AMPLITUDE) dimensionmap[2] = counter++;  
  if (types & USE_DEVIATION) dimensionmap[3] = counter++;  


  setCurrentType(ColorManager::USE_HEIGHT);
}

void ColorManager::setColorMap(ColorMap &cm) {
  for (unsigned int i = 0; i < buckets; i++) {
    cm.calcColor(colormap[i], i, buckets);
  }
  cm.calcColor(colormap[buckets], buckets-1, buckets);
}

void ColorManager::setCurrentType(unsigned int type) {
  switch (type) {
    case USE_NONE: 
      // TODO  implement no color map
      // setColorMap(0);
     currentdim = dimensionmap[0];
     break;
    case USE_HEIGHT: 
     currentdim = dimensionmap[0];
     break;
    case USE_REFLECTANCE: 
     currentdim = dimensionmap[1];
     break;
    case USE_AMPLITUDE: 
     currentdim = dimensionmap[2];
     break;
    case USE_DEVIATION: 
     currentdim = dimensionmap[3];
     break;
   default:
     break;
  }
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
    
void ColorManager::updateRanges(double *point) {
  for (unsigned int i = 0; i < pointdim; i++) {
    if (point[i] < mins[i]) mins[i] = point[i];
    if (point[i] > maxs[i]) maxs[i] = point[i];
  }
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
