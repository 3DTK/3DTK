#ifndef __COLORMANAGER_H__
#define __COLORMANAGER_H__


class ColorMap {
  public:
    enum CM {
      SOLID = 0,
      GREY = 1,
      HSV = 2,
      JET = 3,
      HOT = 4,
      SHSV = 5
    };

    virtual ~ColorMap() {};

  virtual void calcColor(float *d, unsigned int i, unsigned int buckets) {
    d[0] = d[1] = d[2] = 1.0;
  }

  static ColorMap getColorMap(CM map);
  
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

class SHSVMap : public ColorMap {
  public:
  virtual void calcColor(float *d, unsigned int i, unsigned int buckets) {
    float t = (float)i/(float)buckets;
    convert_hsv_to_rgb(240.0*(1.0-t), 1.0, 1.0,  d[0], d[1], d[2]);
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

class DiffMap : public ColorMap {
  public:
  virtual void calcColor(float *d, unsigned int i, unsigned int buckets);
  private:
  static const float cmap[7][3];
};

class ColorManager {

  public: 
  
    ColorManager(unsigned int buckets, unsigned int pointdim, float *mins, float *maxs);
    virtual ~ColorManager();

    void setColor(double *val);

    virtual void setColorMap(ColorMap &cm);

    void invert();
    void setCurrentDim(unsigned int cdim);


    void setMinMax(float min, float max);

  protected:

    unsigned int toIndex(double *val);
  
    void makeValid();
    
    unsigned int buckets;
    
    unsigned int currentdim;
  
    /** stores minima and maxima for each point dimension */ 
    float *mins;
    float *maxs;
    /** maps color to value */ 
    float **colormap;

    float min;
    float max;

    float extentbuckets;    

};

class ColorManagerC : public ColorManager {
  public:
    ColorManagerC(unsigned int buckets, unsigned int pointdim, float *mins, float *maxs, const float _color[3]) : ColorManager(buckets, pointdim, mins, maxs) {
      color[0] = _color[0];
      color[1] = _color[1];
      color[2] = _color[2];
    }

    virtual void setColorMap(ColorMap &cm) {
      for (unsigned int i = 0; i < buckets; i++) {
        cm.calcColor(colormap[i], i, buckets);
        for (unsigned int j = 0; j < 3; j++) {
          colormap[i][j] -= color[j];
          if (colormap[i][j] < 0.0) colormap[i][j] = 0.0;
        }
      }
      cm.calcColor(colormap[buckets], buckets-1, buckets);
      for (unsigned int j = 0; j < 3; j++) {
        colormap[buckets][j] -= color[j];
        if (colormap[buckets][j] < 0.0) colormap[buckets][j] = 0.0;
      }
    }

  private:
    float color[3];
};


#endif
