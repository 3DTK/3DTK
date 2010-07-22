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

const unsigned int ScanColorManager::USE_NONE = 0;
const unsigned int ScanColorManager::USE_REFLECTANCE = 1;
const unsigned int ScanColorManager::USE_AMPLITUDE = 2;
const unsigned int ScanColorManager::USE_DEVIATION = 4;
const unsigned int ScanColorManager::USE_HEIGHT = 8;
const unsigned int ScanColorManager::USE_TYPE = 16;

const unsigned int ScanColorManager::MODE_STATIC = 0;
const unsigned int ScanColorManager::MODE_COLOR_SCAN = 1;
const unsigned int ScanColorManager::MODE_ANIMATION = 2;

/**
 * a const colormap for when scans are supposed to be colored differently
 *
 */
const float ScanColorManager::colormap[6][3] = {
  {0.3,0,0},
  {0,0.3,0},
  {0,0,0.3},
  {0,0.3,0.3},
  {0.3,0,0.3},
  {0.3,0.3,0}};

/**
 * constructor for the main Scan colormanager for all scans in show.
 * @param _buckets number of colors to use
 * @param _types Point attributes used for coloring
 */
ScanColorManager::ScanColorManager(unsigned int _buckets, unsigned int _types) {
  valid = false;
  inverted = false;
  buckets = _buckets;
  types = _types;

  pointdim = 3;
  if (types & USE_REFLECTANCE) pointdim++;
  if (types & USE_AMPLITUDE)   pointdim++;
  if (types & USE_DEVIATION)   pointdim++;
  if (types & USE_TYPE)   pointdim++;
  
  mins = new float[pointdim];
  maxs = new float[pointdim];
  for (unsigned int i = 0; i < pointdim; i++) { 
    mins[i] = FLT_MAX;
    maxs[i] = FLT_MIN;
  }
  
  dimensionmap[1] = dimensionmap[2] = dimensionmap[3] = dimensionmap[4] = 1; // choose height per default  
  dimensionmap[0] = 1;  // height 
  
  int counter = 3;
  if (types & USE_REFLECTANCE) dimensionmap[1] = counter++;  
  if (types & USE_AMPLITUDE) dimensionmap[2] = counter++;  
  if (types & USE_DEVIATION) dimensionmap[3] = counter++;  
  if (types & USE_TYPE) dimensionmap[4] = counter++;  
 
  currenttype = USE_HEIGHT;
  currentdim = 0;
 
    
}

/**
 * changes maximum and minimum for all attributes appropriately if needed 
 */
void ScanColorManager::updateRanges(double *point) {
  for (unsigned int i = 0; i < pointdim; i++) {
    if (point[i] < mins[i]) mins[i] = point[i];
    if (point[i] > maxs[i]) maxs[i] = point[i];
  }
}

/**
 * sets the minimum and maximum for the current color type
 */
void ScanColorManager::setMinMax(float min, float max) {
  makeValid();
  for (unsigned int i = 0; i < allManager.size(); i++) {
    allManager[i]->setMinMax(min, max);
  }
}
  
/**
 * Called by an Octree to make itself known to this manager
 */
void ScanColorManager::registerTree(Show_BOctTree *scan) {
  allScans.push_back(scan);
}
    

/**
 * Checks wether this manager has been correctly initialized.
 * If not it initializes itself.
 */
void ScanColorManager::makeValid() {
  if (!valid) {
    for (unsigned int i = 0; i < allScans.size(); i++) {
      Show_BOctTree *scan = allScans[i];
      ColorManager *cm = new ColorManager(buckets, pointdim, mins, maxs);
      cm->setCurrentDim(currentdim);
      scan->setColorManager(cm);
      staticManager.push_back(cm);
     
      // new colormanager for scan index influenced colorscheme
      ColorManagerC *cmc = new ColorManagerC(buckets, pointdim, mins, maxs, colormap[i%6]);
      scanManager.push_back(cmc);

      allManager.push_back(cm);
      allManager.push_back(cmc);
    }
    valid = true;
  }
}

/**
 * sets the current mode of the manager. 
 * There are currently three modes:
 * MODE_STATIC for when the scene is static and all scans have essentially 
 * the same colormanager
 * MODE_COLOR_SCAN for when the scene is static and every scan has an 
 * individual colormanager for its points
 * MODE_ANIMATION for when the scene is animated
 */
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
   
/**
 * Sets the current Colormap in use
 */
void ScanColorManager::setColorMap(ColorMap &cm) {
  makeValid();
  for (unsigned int i = 0; i < allManager.size(); i++) {
    allManager[i]->setColorMap(cm);
    if (inverted) allManager[i]->invert();
  }
}

/**
 * Sets the current Colormap in use
 */
void ScanColorManager::setColorMap(ColorMap::CM &cm) {
  ColorMap cmap = ColorMap::getColorMap(cm); 
  setColorMap(cmap);
}
    
/**
 * Sets the current type, i.e. which Point attribute to color according to
 * the current colormap
 */
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
    case USE_TYPE: 
      currentdim = dimensionmap[4];
      break;
    default:
      break;
  }

  if(type != USE_NONE) {
    for (unsigned int i = 0; i < allManager.size(); i++) {
      allManager[i]->setCurrentDim(currentdim);
    }
    currenttype = type;
  }
}


/**
 * sets the Color for all subsequently drawn points. Only works properly when in MODE_ANIMATION 
 */
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
    

/**
 * allocates and initializes a double[] representation of the given Point
 * according to the specified types
 */
double * ScanColorManager::createPoint(const Point &P, const unsigned int types) {
  unsigned int counter = 0;
  
  double *p = new double[pointdim];
  p[counter++] = P.x;
  p[counter++] = P.y;
  p[counter++] = P.z;
  if (types & ScanColorManager::USE_REFLECTANCE) {
    p[counter++] = P.reflectance;
  }
  if (types & ScanColorManager::USE_AMPLITUDE) {
    p[counter++] = P.amplitude;
  }
  if (types & ScanColorManager::USE_DEVIATION) {  
    p[counter++] = P.deviation;
  }
  if (types & ScanColorManager::USE_TYPE) {  
    p[counter++] = P.type;
  }

  return p;

}
    

void ScanColorManager::setInvert(bool invert) {
  if (invert != inverted) {
    for (unsigned int i = 0; i < allManager.size(); i++) {
      allManager[i]->invert();
    }
  }
  inverted = invert;
}
