#ifndef __COLORMANAGER_H__
#define __COLORMANAGER_H__

#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#include <windows.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#include <string.h>

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
  
    ColorManager(unsigned int _buckets, unsigned int pointdim, float *_mins, float *_maxs) : buckets(_buckets) {
      colormap = new float*[buckets + 1];  // allow a color more for values equal to max
      for (unsigned int i = 0; i < buckets; i++) {
        colormap[i] = new float[3];
      }
      colormap[buckets] = new float[3];
      
      mins = new float[pointdim];
      maxs = new float[pointdim];
      for (unsigned int i = 0; i < pointdim; i++) { 
        mins[i] = _mins[i];
        maxs[i] = _maxs[i];
      }

      setCurrentDim(0);
    }

    virtual ~ColorManager() {
      for (unsigned int i = 0; i < buckets; i++) {
        delete[] colormap[i];
      }
      delete[] colormap[buckets];
      delete[] colormap;

      delete[] mins;
      delete[] maxs;
    }

    virtual void setColor(float *val) {
      int index = toIndex(val);
      glColor4f( colormap[index][0], colormap[index][1], colormap[index][2], 1.0 ); 
    }
    virtual void setColor(double *val) {
      int index = toIndex(val);
      glColor4f( colormap[index][0], colormap[index][1], colormap[index][2], 1.0 ); 
    }
    virtual void setColor(short int *val) {
      int index = toIndex(val);
      //cout << "I " << index << " " << val[currentdim] << " " << currentdim << endl; 
      glColor4f( colormap[index][0], colormap[index][1], colormap[index][2], 1.0 ); 
    }

    virtual void setColorMap(ColorMap &cm) {
      for (unsigned int i = 0; i < buckets; i++) {
        cm.calcColor(colormap[i], i, buckets);
      }
      cm.calcColor(colormap[buckets], buckets-1, buckets);
    }

    void invert() {
      for (unsigned int i = 0; i < buckets+1; i++) {
        for (unsigned int j = 0; j < 3; j++) {
          colormap[i][j] = 1.0 - colormap[i][j];
        }
      }
    }


    void setCurrentDim(unsigned int cdim) {
      currentdim = cdim;
      makeValid();
    }


    void setMinMax(float _min, float _max) {
      if (_min < _max) {
        min = _min;
        max = _max;
      }
      float extent = max - min;
      extentbuckets = extent/(float)buckets;
    }

  protected:

    template <class T>
    unsigned int toIndex(T *val) {
      T value = val[currentdim];
      if (value < min) return 0;
      if (value > max) return buckets;
      return (unsigned int)((value-min)/extentbuckets);
    }

    void makeValid() {
      min = mins[currentdim];
      max = maxs[currentdim];

      float extent = max - min;
      extentbuckets = extent/(float)buckets;
    }

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
      for (unsigned int i = 0; i < ColorManager::buckets; i++) {
        cm.calcColor(ColorManager::colormap[i], i, ColorManager::buckets);
        for (unsigned int j = 0; j < 3; j++) {
          ColorManager::colormap[i][j] -= color[j];
          if (ColorManager::colormap[i][j] < 0.0) ColorManager::colormap[i][j] = 0.0;
        }
      }
      cm.calcColor(ColorManager::colormap[ColorManager::buckets], ColorManager::buckets-1, ColorManager::buckets);
      for (unsigned int j = 0; j < 3; j++) {
        ColorManager::colormap[ColorManager::buckets][j] -= color[j];
        if (ColorManager::colormap[ColorManager::buckets][j] < 0.0) ColorManager::colormap[ColorManager::buckets][j] = 0.0;
      }
    }

  private:
    float color[3];
};

class CColorManager : public ColorManager {
  public:
    CColorManager(unsigned int buckets, unsigned int pointdim, float *mins, float *maxs, unsigned int _colordim) : ColorManager(buckets, pointdim, mins, maxs) {
      colordim = _colordim;
    }

    void setColor(double *val) {  
      GLubyte color[3];
      memcpy(color, &val[colordim], 3);      
      glColor3ubv(color); 
    }
    void setColor(float *val) {  
      GLubyte color[3];
      memcpy(color, &val[colordim], 3);      
      glColor3ubv(color); 
    }
    void setColor(short *val) {  
      GLubyte color[3];
      memcpy(color, &val[colordim], 3);      
      glColor3ubv(color); 
    }

  private:
    unsigned int colordim;
};


#endif
