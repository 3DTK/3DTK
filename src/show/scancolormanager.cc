#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#include <windows.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include "slam6d/point.h"
#include "slam6d/scan.h"
#include "show/colordisplay.h"
#include "show/colormanager.h"
#include "show/scancolormanager.h"
#include <vector>
#include <float.h>
#include "slam6d/point_type.h"
using std::vector;

ScanColorManager::ScanColorManager(unsigned int _buckets, PointType type) : pointtype(type) {
      valid = false;
      inverted = false;
      buckets = _buckets;
//      types = _types;
    
      pointtype = type;
      //pointdim = PointType::PointDim(types);
      //

      
      mins = new float[pointtype.getPointDim()];
      maxs = new float[pointtype.getPointDim()];
      for (unsigned int i = 0; i < pointtype.getPointDim(); i++) { 
        mins[i] = FLT_MAX;
        maxs[i] = FLT_MIN;
      }

      currenttype = PointType::USE_HEIGHT;
      currentdim = 0;
    }

    void ScanColorManager::registerTree(colordisplay *b) { allScans.push_back(b); }
    
    void ScanColorManager::setColorMap(ColorMap &cm) {
      makeValid();
      for (unsigned int i = 0; i < allManager.size(); i++) {
        allManager[i]->setColorMap(cm);
        if (inverted) allManager[i]->invert();
      }
    }

    void ScanColorManager::setColorMap(ColorMap::CM &cm) {
      ColorMap cmap = ColorMap::getColorMap(cm); 
      setColorMap(cmap);
    }

    void ScanColorManager::setCurrentType(unsigned int type) {
      makeValid();
      if (type == PointType::USE_NONE) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          allScans[i]->setColorManager(0); 
        }
      }
      currentdim = pointtype.getType(type);

      if(type != PointType::USE_NONE) {
        for (unsigned int i = 0; i < allManager.size(); i++) {
          allManager[i]->setCurrentDim(currentdim);
        }
        currenttype = type;
      }
    }

    void ScanColorManager::setMinMax(float min, float max) {
      makeValid();
      for (unsigned int i = 0; i < allManager.size(); i++) {
        allManager[i]->setMinMax(min, max);
      }
    }
    void ScanColorManager::setMode(const unsigned int &mode) {
      if (mode == ScanColorManager::MODE_STATIC) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          allScans[i]->setColorManager(staticManager[i]);
        }
      } else if (mode == ScanColorManager::MODE_COLOR_SCAN) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          allScans[i]->setColorManager(scanManager[i]);
        }
      } else if (mode == ScanColorManager::MODE_ANIMATION) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          allScans[i]->setColorManager(0);
        }
      } else if (mode == ScanColorManager::MODE_POINT_COLOR) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          allScans[i]->setColorManager(colorsManager[i]);
        }
      }
    }
    void ScanColorManager::setInvert(bool invert) {
      if (invert != inverted) {
        for (unsigned int i = 0; i < allManager.size(); i++) {
          allManager[i]->invert();
        }
      }
      inverted = invert;
    }

    float ScanColorManager::getMin() { return mins[currentdim];};
    float ScanColorManager::getMax() { return maxs[currentdim];};
    float ScanColorManager::getMin(unsigned int dim) { return mins[dim];};
    float ScanColorManager::getMax(unsigned int dim) { return maxs[dim];};
    unsigned int ScanColorManager::getPointDim() { return pointtype.getPointDim(); };
    void ScanColorManager::makeValid() {
      if (!valid) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          colordisplay *scan = allScans[i];
          ColorManager *cm = new ColorManager(buckets, pointtype.getPointDim(), mins, maxs);
          cm->setCurrentDim(currentdim);
          scan->setColorManager(cm);
          staticManager.push_back(cm);

          // new colormanager for scan index influenced colorscheme
          ColorManagerC *cmc = new ColorManagerC(buckets, pointtype.getPointDim(), mins, maxs, colormap[i%6]);
          scanManager.push_back(cmc);

          // new colormanager for the color based on the color of the points
          CColorManager *ccm = new CColorManager(buckets, pointtype.getPointDim(), mins, maxs, pointtype.getType(PointType::USE_COLOR));
          colorsManager.push_back(ccm);

          allManager.push_back(cm);
          allManager.push_back(cmc);
          allManager.push_back(ccm);

        }
        valid = true;
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


 
const unsigned int ScanColorManager::MODE_STATIC = 0;
const unsigned int ScanColorManager::MODE_COLOR_SCAN = 1;
const unsigned int ScanColorManager::MODE_ANIMATION = 2;
const unsigned int ScanColorManager::MODE_POINT_COLOR = 3;
  

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

