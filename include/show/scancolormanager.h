#ifndef __SCANCOLORMANAGER_H__
#define __SCANCOLORMANAGER_H__

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
#include "show/show_Boctree.h"
#include "show/colormanager.h"
#include <vector>
#include <float.h>
#include "slam6d/point_type.h"
using std::vector;

template <class T> class Show_BOctTree;

/**
 * This class is a special ColorManager that handles a set of Colormanagers.
 * This manager is capable of mapping managers to scans in dependence of the state of the 
 * show program. 
 */
template <class T = float> class ScanColorManager {
  public:

  static const unsigned int MODE_STATIC;
  static const unsigned int MODE_COLOR_SCAN;
  static const unsigned int MODE_ANIMATION;
  static const unsigned int MODE_POINT_COLOR;

    ScanColorManager(unsigned int _buckets, PointType<T> type) : pointtype(type) {
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

      currenttype = PointType<T>::USE_HEIGHT;
      currentdim = 0;
    }

    void registerTree(Show_BOctTree<T> *b) { allScans.push_back(b); }
    
    void setColorMap(ColorMap &cm) {
      makeValid();
      for (unsigned int i = 0; i < allManager.size(); i++) {
        allManager[i]->setColorMap(cm);
        if (inverted) allManager[i]->invert();
      }
    }

    void setColorMap(ColorMap::CM &cm) {
      ColorMap cmap = ColorMap::getColorMap(cm); 
      setColorMap(cmap);
    }

    void setCurrentType(unsigned int type) {
      makeValid();
      if (type == PointType<T>::USE_NONE) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          allScans[i]->setColorManager(0); 
        }
      }
      currentdim = pointtype.getType(type);

      if(type != PointType<T>::USE_NONE) {
        for (unsigned int i = 0; i < allManager.size(); i++) {
          allManager[i]->setCurrentDim(currentdim);
        }
        currenttype = type;
      }
    }

    void updateRanges(T *point) {
      for (unsigned int i = 0; i < pointtype.getPointDim(); i++) {
        if (point[i] < mins[i]) mins[i] = point[i];
        if (point[i] > maxs[i]) maxs[i] = point[i];
      }
    }

    void setMinMax(float min, float max) {
      makeValid();
      for (unsigned int i = 0; i < allManager.size(); i++) {
        allManager[i]->setMinMax(min, max);
      }
    }
    void setMode(const unsigned int &mode) {
      if (mode == ScanColorManager<T>::MODE_STATIC) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          allScans[i]->setColorManager(staticManager[i]);
        }
      } else if (mode == ScanColorManager<T>::MODE_COLOR_SCAN) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          allScans[i]->setColorManager(scanManager[i]);
        }
      } else if (mode == ScanColorManager<T>::MODE_ANIMATION) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          allScans[i]->setColorManager(0);
        }
      } else if (mode == ScanColorManager<T>::MODE_POINT_COLOR) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          allScans[i]->setColorManager(colorsManager[i]);
        }
      }
    }
    void setInvert(bool invert) {
      if (invert != inverted) {
        for (unsigned int i = 0; i < allManager.size(); i++) {
          allManager[i]->invert();
        }
      }
      inverted = invert;
    }

    inline float getMin() { return mins[currentdim];};
    inline float getMax() { return maxs[currentdim];};
    inline float getMin(unsigned int dim) { return mins[dim];};
    inline float getMax(unsigned int dim) { return maxs[dim];};
    inline unsigned int getPointDim() { return pointtype.getPointDim(); };
    void makeValid() {
      if (!valid) {
        for (unsigned int i = 0; i < allScans.size(); i++) {
          Show_BOctTree<T> *scan = allScans[i];
          ColorManager<T> *cm = new ColorManager<T>(buckets, pointtype.getPointDim(), mins, maxs);
          cm->setCurrentDim(currentdim);
          scan->setColorManager(cm);
          staticManager.push_back(cm);

          // new colormanager for scan index influenced colorscheme
          ColorManagerC<T> *cmc = new ColorManagerC<T>(buckets, pointtype.getPointDim(), mins, maxs, colormap[i%6]);
          scanManager.push_back(cmc);

          // new colormanager for the color based on the color of the points
          CColorManager<T> *ccm = new CColorManager<T>(buckets, pointtype.getPointDim(), mins, maxs, pointtype.getType(PointType<T>::USE_COLOR));
          colorsManager.push_back(ccm);

          allManager.push_back(cm);
          allManager.push_back(cmc);
          allManager.push_back(ccm);

        }
        valid = true;
      }
    }

    void selectColors(Scan::AlgoType type) {
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


  protected:

    unsigned int currentdim;

    vector<Show_BOctTree<T> *> allScans;
    vector<ColorManager<T> *> allManager;

    vector<ColorManager<T> *> staticManager;
    vector<ColorManagerC<T> *> scanManager;
    vector<CColorManager<T> *> colorsManager;
//    vector<ColorManager *> scanAnimManager;  // Implement later

    unsigned int currenttype;
    
    static const float colormap[6][3];

    unsigned int buckets;

    /** stores minima and maxima for each point dimension */ 
    float *mins;
    float *maxs;
    /** maps valuetypes to point dimension for easier access */ 
    PointType<T> pointtype;
    
    bool valid;
    bool colorScans;
    bool inverted;
};

 
template <class T> const unsigned int ScanColorManager<T>::MODE_STATIC = 0;
template <class T> const unsigned int ScanColorManager<T>::MODE_COLOR_SCAN = 1;
template <class T> const unsigned int ScanColorManager<T>::MODE_ANIMATION = 2;
template <class T> const unsigned int ScanColorManager<T>::MODE_POINT_COLOR = 3;
  

/**
 * a const colormap for when scans are supposed to be colored differently
 *
 */
template <class T> const float ScanColorManager<T>::colormap[6][3] = {
  {0.3,0,0},
  {0,0.3,0},
  {0,0,0.3},
  {0,0.3,0.3},
  {0.3,0,0.3},
  {0.3,0.3,0}};

#endif
