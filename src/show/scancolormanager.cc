#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#include <windows.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include "scancolormanager.h"
#include "show_Boctree.h"
#include "float.h"


float ScanColorManager::colormap[6][3] = {
  {0.3,0,0},
  {0,0.3,0},
  {0,0,0.3},
  {0,0.3,0.3},
  {0.3,0,0.3},
  {0.3,0.3,0}};

ScanColorManager::ScanColorManager(unsigned int _buckets, unsigned int _types) {
  valid = false;
  buckets = _buckets;
  types = _types;

  pointdim = 3;
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
 
  currenttype = USE_HEIGHT;
  currentdim = 0;
    
}

void ScanColorManager::updateRanges(double *point) {
  for (unsigned int i = 0; i < pointdim; i++) {
    if (point[i] < mins[i]) mins[i] = point[i];
    if (point[i] > maxs[i]) maxs[i] = point[i];
  }
}

void ScanColorManager::setMinMax(float min, float max) {
  makeValid();
  for (unsigned int i = 0; i < allManager.size(); i++) {
    allManager[i]->setMinMax(min, max);
  }
}
  
    
void ScanColorManager::registerTree(Show_BOctTree *scan) {
  allScans.push_back(scan);
}
    
void ScanColorManager::makeValid() {
  if (!valid) {
    for (unsigned int i = 0; i < allScans.size(); i++) {
      Show_BOctTree *scan = allScans[i];
      ColorManager *cm = new ColorManager(buckets, types, pointdim, mins, maxs);
      cm->setCurrentDim(currentdim);
      scan->setColorManager(cm);
      staticManager.push_back(cm);
     
      // new colormanager for scan index influenced colorscheme
      ColorManagerC *cmc = new ColorManagerC(buckets, types, pointdim, mins, maxs, colormap[i%6]);
      scanManager.push_back(cmc);

      allManager.push_back(cm);
      allManager.push_back(cmc);
    }
    valid = true;
  }
}


void ScanColorManager::setMode(const unsigned int &mode) {
  switch(mode) {
    case MODE_STATIC:
      for (unsigned int i = 0; i < allScans.size(); i++) {
        allScans[i]->setColorManager(staticManager[i]);
      }
      break;
    case MODE_COLOR_SCAN:
      for (unsigned int i = 0; i < allScans.size(); i++) {
        allScans[i]->setColorManager(scanManager[i]);
      }
      break;
    case MODE_ANIMATION:
      for (unsigned int i = 0; i < allScans.size(); i++) {
        allScans[i]->setColorManager(0);
      }
      break;
    default:
      break;
  }
}
/////////////////////////////////////////////////////////////
    
void ScanColorManager::setColorMap(ColorMap &cm) {
  makeValid();
  for (unsigned int i = 0; i < allManager.size(); i++) {
    allManager[i]->setColorMap(cm);
  }
}

void ScanColorManager::setColorMap(ColorMap::CM &cm) {
  ColorMap cmap = ColorMap::getColorMap(cm); 
  setColorMap(cmap);
}
    
void ScanColorManager::setCurrentType(unsigned int type) {
  makeValid();
  switch (type) {
    case USE_NONE: 
      for (unsigned int i = 0; i < allScans.size(); i++) {
        allScans[i]->setColorManager(0); 
      }
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

  // TODO make sure this is right
  if(type != USE_NONE) {
    for (unsigned int i = 0; i < allManager.size(); i++) {
      allManager[i]->setCurrentDim(currentdim);
    }
    currenttype = type;

  }
}

    
void ScanColorManager::selectColors(Scan::AlgoType type) {
  switch(type) {
    case Scan::ICP:
      glColor4d(0.85, 0.30,0.023, 1.0);
      break;
    case Scan::ICPINACTIVE:
      glColor4d(0.78, 0.63,0.57, 1.0);	
      break;
    case Scan::LUM:
      glColor4d(1.0, 0.0,0.0, 1.0);
      break;
    case Scan::ELCH:
      glColor4d(0.0, 1.0,0.0, 1.0);
      break;
    case Scan::LOOPTORO:
      glColor4d(0.0, 0.0, 1.0, 1.0);
      break;
    case Scan::LOOPHOGMAN:
      glColor4d(0.0, 1.0, 1.0, 1.0);
      break;
    case Scan::GRAPHTORO:
      glColor4d(1.0, 0.0, 1.0, 1.0);
      break;
    case Scan::GRAPHHOGMAN:
      glColor4d(1.0, 1.0, 0.0, 1.0);
      break;
    default:
      glColor4d(1.0, 1.0, 1.0, 1.0);
      break;
  }
}
