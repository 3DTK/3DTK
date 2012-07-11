/*
 * compacttree implementation
 *
 * Copyright (C) Jan Elseberg, Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#include <windows.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "stdio.h"
#include "slam6d/globals.icc"
#include "float.h"
#include "show/colormanager.h"




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
    case SHSV:
      return SHSVMap();
      break;
    default:
      return ColorMap();
      break;
  }
}

const float DiffMap::cmap[7][3] = {
  {       1.0,       1.0,         0},
  {       1.0,    0.5000,       1.0},
  {         0,       1.0,       1.0},
  {       1.0,    0.2500,    0.2500},
  {    0.2500,       1.0,    0.2500},
  {    0.2500,    0.2500,       1.0},
  {    0.7500,    0.7500,    0.7500} };

void DiffMap::calcColor(float *d, unsigned int i, unsigned int buckets) {
  d[0] = cmap[i%7][0];
  d[1] = cmap[i%7][1];
  d[2] = cmap[i%7][2];
}
