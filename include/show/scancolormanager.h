#ifndef __SCANCOLORMANAGER_H__
#define __SCANCOLORMANAGER_H__

#ifdef WITH_GLEE
#include "glee/GLee.h"
#endif

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
//#include "show/show_Boctree.h"
#include "show/colormanager.h"
#include <vector>
#include <float.h>
#include "slam6d/point_type.h"
using std::vector;

//template <class T> class Show_BOctTree;

/**
 * This class is a special ColorManager that handles a set of Colormanagers.
 * This manager is capable of mapping managers to scans in dependence of the state of the 
 * show program. 
 */
class ScanColorManager {
  public:

  static const unsigned int MODE_STATIC;
  static const unsigned int MODE_COLOR_SCAN;
  static const unsigned int MODE_ANIMATION;
  static const unsigned int MODE_POINT_COLOR;

    ScanColorManager(unsigned int _buckets, PointType type, bool animation_color = true);
    
    void registerTree(colordisplay *b);
    
    void setColorMap(ColorMap &cm);
    void setColorMap(ColorMap::CM &cm);
    void setCurrentType(unsigned int type);
    void setMinMax(float min, float max);
    void setMode(const unsigned int &mode);
    void setInvert(bool invert);

    float getMin();
    float getMax();
    float getMin(unsigned int dim);
    float getMax(unsigned int dim);
    unsigned int getPointDim();
    void makeValid();

    void selectColors(Scan::AlgoType type);
  
    template<class P>
    void updateRanges(P *point);

  protected:

    unsigned int currentdim;

    vector<colordisplay *> allScans;
    vector<ColorManager *> allManager;

    vector<ColorManager *> staticManager;
    vector<ColorManager *> scanManager;
    vector<CColorManager *> colorsManager;

    unsigned int currenttype;
    
    unsigned int buckets;

    /** stores minima and maxima for each point dimension */ 
    float *mins;
    float *maxs;
    /** maps valuetypes to point dimension for easier access */ 
    PointType pointtype;
    
    bool animationColor;            /**< Alter colors when animating        */

    bool valid;
    bool colorScans;
    bool inverted;
};
    
  template<class P>
    void ScanColorManager::updateRanges(P *point) {
      for (unsigned int i = 0; i < pointtype.getPointDim(); i++) {
        if (point[i] < mins[i]) mins[i] = point[i];
        if (point[i] > maxs[i]) maxs[i] = point[i];
      }
    }


#endif
