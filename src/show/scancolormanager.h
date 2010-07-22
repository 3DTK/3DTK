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
    static const unsigned int USE_NONE;
    static const unsigned int USE_REFLECTANCE;
    static const unsigned int USE_AMPLITUDE;
    static const unsigned int USE_DEVIATION;
    static const unsigned int USE_HEIGHT;
    static const unsigned int USE_TYPE;

    static const unsigned int MODE_STATIC;
    static const unsigned int MODE_COLOR_SCAN;
    static const unsigned int MODE_ANIMATION;

    ScanColorManager(unsigned int buckets, unsigned int types);

    void registerTree(Show_BOctTree *b);
    
    void setColorMap(ColorMap &cm);
    void setColorMap(ColorMap::CM &cm);
    void setCurrentType(unsigned int type);

    void updateRanges(double *);

    void setMinMax(float min, float max);
    void setMode(const unsigned int &mode);
    void setInvert(bool invert);

    inline float getMin() { return mins[currentdim];};
    inline float getMax() { return maxs[currentdim];};
    inline unsigned int getPointDim() { return pointdim; };
    void makeValid();

    void selectColors(Scan::AlgoType type);

    double *createPoint(const Point &p, const unsigned int types);

  protected:

    unsigned int pointdim;
    unsigned int currentdim;

    vector<Show_BOctTree *> allScans;
    vector<ColorManager *> allManager;

    vector<ColorManager *> staticManager;
    vector<ColorManagerC *> scanManager;
//    vector<ColorManager *> scanAnimManager;  // Implement later


    unsigned int currenttype;
    
    static const float colormap[6][3];

    unsigned int buckets;
    unsigned int types;

    /** stores minima and maxima for each point dimension */ 
    float *mins;
    float *maxs;
    /** maps valuetypes to point dimension for easier access */ 
    int dimensionmap[5];
    
    bool valid;
    bool colorScans;
    bool inverted;
};

#endif
