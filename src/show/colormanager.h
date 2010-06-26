#ifndef __COLORMANAGER_H__
#define __COLORMANAGER_H__


class ColorMap {
  public:
  virtual void calcColor(float *d, unsigned int i, unsigned int buckets) {
    d[0] = d[1] = d[2] = 1.0;
  }

  /**
   * hue is in [0,360], all others in [0,1]
   */
  static void convert_hsv_to_rgb(float hue, float s, float v,  
                                 float &r, float &g, float &b); 
};

class GreyMap : public ColorMap {
  public:
  virtual void calcColor(float *d, unsigned int i, unsigned int buckets) {
    d[0] = (float)i/(float)buckets;
    d[1] = (float)i/(float)buckets;
    d[2] = (float)i/(float)buckets;
  }
};

class HSVMap : public ColorMap {
  public:
  virtual void calcColor(float *d, unsigned int i, unsigned int buckets) {
    float t = (float)i/(float)buckets;
    convert_hsv_to_rgb(360.0*t, 1.0, 1.0,  d[0], d[1], d[2]);
  }
};

class JetMap : public ColorMap {
  public:
  virtual void calcColor(float *d, unsigned int i, unsigned int buckets) {
    float t = (float)i/(float)buckets;
    if (t <= 0.125) {
      d[0] = d[1] = 0.0; d[2] = 0.5 + 0.5*(t/0.125);
    } else if (t < 0.375) {
      d[0] = 0.0; d[2] = 1.0; d[1] = (t-0.125)/0.25;
    } else if (t < 0.625) {
      d[1] = 1.0; d[0] = (t-0.375)/0.25;; d[2] = 1.0 - d[0];
    } else if (t < 0.875) {
      d[0] = 1.0; d[2] = 0.0; d[1] = 1.0 - (t-0.625)/0.25;
    } else {
      d[1] = d[2] = 0.0; d[2] = 1.0 - 0.5*(t/0.125);
    }
  }
};

class HotMap : public ColorMap {
  public:
  virtual void calcColor(float *d, unsigned int i, unsigned int buckets) {
    float t = (float)i/(float)buckets;
    if (t <= 1.0/3.0) {
      d[1] = d[2] = 0.0; d[0] = t/(1.0/3.0);
    } else if (t <= 2.0/3.0) {
      d[0] = 1.0; d[2] = 0.0; d[1] = (t-(1.0/3.0))/(1.0/3.0);
    } else {
      d[0] = 1.0; d[1] = 1.0; d[2] = (t-(2.0/3.0))/(1.0/3.0);
    }
  }
};

class ColorManager {

  public: 
  
    static const unsigned int USE_NONE = 0;
    static const unsigned int USE_REFLECTANCE = 1;
    static const unsigned int USE_AMPLITUDE = 2;
    static const unsigned int USE_DEVIATION = 4;
    static const unsigned int USE_HEIGHT = 8;
//  static const unsigned int USE_TYPE = 16

    ColorManager(unsigned int buckets, unsigned int types);

    void setColor(double *val);

    void setColorMap(ColorMap &cm);
    void setCurrentType(unsigned int type);

    void updateRanges(double *);

    void setMinMax(float min, float max);

    inline float getMin() { return min;};
    inline float getMax() { return max;};
    inline unsigned int getPointDim() { return pointdim; };
  protected:

    unsigned int toIndex(double *val);
  
    void makeValid();
    
    unsigned int buckets;
    
    unsigned int pointdim;
    unsigned int currentdim;
  
    /** stores minima and maxima for each point dimension */ 
    float *mins;
    float *maxs;
    /** maps color to value */ 
    float **colormap;
    /** maps valuetypes to point dimension for easier access */ 
    int dimensionmap[4];

    float min;
    float max;

    float extentbuckets;    

   // ColorMap current_cm;

};


#endif
