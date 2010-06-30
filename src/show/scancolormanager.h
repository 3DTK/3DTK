#ifndef __SCANCOLORMANAGER_H__
#define __SCANCOLORMANAGER_H__

#include "../scan.h"
#include "show_Boctree.h"
#include "colormanager.h"
#include <vector>
using std::vector;

class Show_BOctTree;

/**
 * This class is a special ColorManager that handles a set of Colormanagers.
 * This manager is capable of mapping managers to scans in dependence of the state of the 
 * show program. 
 */
class ScanColorManager {
  public:
    static const unsigned int USE_NONE = 0;
    static const unsigned int USE_REFLECTANCE = 1;
    static const unsigned int USE_AMPLITUDE = 2;
    static const unsigned int USE_DEVIATION = 4;
    static const unsigned int USE_HEIGHT = 8;
//  static const unsigned int USE_TYPE = 16

    static const unsigned int MODE_STATIC = 0;
    static const unsigned int MODE_COLOR_SCAN = 1;
    static const unsigned int MODE_ANIMATION = 2;

    ScanColorManager(unsigned int buckets, unsigned int types);

    void registerTree(Show_BOctTree *b);
    
    void setColorMap(ColorMap &cm);
    void setColorMap(ColorMap::CM &cm);
    void setCurrentType(unsigned int type);

    void updateRanges(double *);

    void setMinMax(float min, float max);
    void setMode(const unsigned int &mode);

    inline float getMin() { return mins[currentdim];};
    inline float getMax() { return maxs[currentdim];};
    inline unsigned int getPointDim() { return pointdim; };
    void makeValid();

    /**
       * color selector for animation
       */
    void selectColors(Scan::AlgoType type);

  protected:

    unsigned int pointdim;
    unsigned int currentdim;

    vector<Show_BOctTree *> allScans;
    vector<ColorManager *> allManager;

    vector<ColorManager *> staticManager;
    vector<ColorManagerC *> scanManager;
//    vector<ColorManager *> scanAnimManager;  // Implement later


    unsigned int currenttype;
    
    static float colormap[6][3];

    unsigned int buckets;
    unsigned int types;

    /** stores minima and maxima for each point dimension */ 
    float *mins;
    float *maxs;
    /** maps valuetypes to point dimension for easier access */ 
    int dimensionmap[4];
    
    bool valid;
    bool colorScans;
};

#endif
