#ifndef __COLORMANAGER_H__
#define __COLORMANAGER_H__

#ifdef _WIN32
#define  _USE_MATH_DEFINES
#include <windows.h>
#endif
#ifdef WITH_OPENGL
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif
#else
#include "show/dummygl.h"
#endif
#include <string.h>
#include <stdio.h>

// This defines the floating point precision of the show program
typedef float sfloat;

static GLuint defaultTexture;

class ColorMap {
  public:
    enum CM {
      SOLID = 0,
      GREY = 1,
      HSV = 2,
      JET = 3,
      HOT = 4,
      SHSV = 5,
      TEMP = 6
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
      d[1] = d[2] = 0.0; d[0] = 1.0 - 0.5*((t - 0.875)/0.125);
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

class TempMap : public ColorMap {
  public:
  virtual void calcColor(float *d, unsigned int i, unsigned int buckets) {
    float t = 1.0 - (float)i/(float)buckets;

    if(t <= 1.0/5.0) {
      d[1] = d[2] = 0.0;
      d[0] = t/(1.0/5.0);
    } else if(t <=2.0/5.0) {
      d[0] = 1.0;
      d[1] = (t-(1.0/5.0))/(1.0/5.0);
      d[2] = 0.0;
    } else if(t <=3.0/5.0) {
      d[0] = 1.0 - ((t-(2.0/5.0))/(1.0/5.0));
      d[1] = 1.0;
      d[2] = (t-(2.0/5.0))/(1.0/5.0);
    } else if(t <=4.0/5.0) {
      d[0] = 0.0;
      d[1] = 1.0 - ((t-(3.0/5.0))/(1.0/5.0));
      d[2] = 1.0;
    } else if(t == 1.0) {
      d[0] = d[1] = d[2] = 0.80;
    } else {
      d[0] = 0.0;
      d[1] = 0.0;
      d[2] = 1.0 - ((t-(4.0/5.0))/(1.0/5.0));
    }
  }
};

/*
 * For collision detection three pointclouds have to be presented and colored
 * differently:
 *   1. colliding points
 *   2. non-colliding points
 *   3. model
 *
 * Volkswagen wants the following coloring scheme:
 *
 *   - environment in grayscale
 *   - model in orange
 *   - colliding points in
 *       - green for 0-3 cm
 *       - blue for 3-5 cm
 *       - red for 5-10 cm
 *
 * Reflectance values encode the above in the following way:
 *
 *   - -1 <= r < 0: colliding
 *   - 0 <= r < 1: non-colliding
 *   - 1: model
 *
 * Which means the intervals for the collidings points are:
 *
 *   - -0.3 <= r < 0: green
 *   - -0.5 <= r < 0.3: blue
 *   - r < 0.5
 *
 * The exact colorvalues for green, blue, red and orange are:
 *
 *   - green: 0x0eee00
 *   - blue: 0x1e90ff
 *   - red: 0xff0000
 *   - orange: 0xff8c00
 */
class VWMap : public ColorMap {
  public:
  virtual void calcColor(float *d, unsigned int i, unsigned int buckets) {
    float t = 2.0 * (float)i/(float)buckets - 1.0;
	if (t < -0.5) {
		// red
		d[0] = 1;
		d[1] = 0;
		d[2] = 0;
	} else if (t < -0.3) {
		// blue
		d[0] = 30.0/255.0;
		d[1] = 144.0 / 255.0;
		d[2] = 1;
	} else if (t < 0) {
		// green
		d[0] = 14.0 / 255.0;
		d[1] = 238.0 / 255.0;
		d[2] = 0;
	} else if (t < 1) {
		d[0] = d[1] = d[2] = t; // gray
	} else {
		d[0] = 1.0;
		d[1] = 0.55;
		d[2] = 0.0;
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

    ColorManager(unsigned int _buckets, unsigned int pointdim, float *_mins, float *_maxs, const float *_color = 0) : buckets(_buckets) {
      if (_color) {
        color[0] = _color[0];
        color[1] = _color[1];
        color[2] = _color[2];
      } else {
        color[0] = 1;
        color[1] = 1;
        color[2] = 1;
      }

      colormap = new float*[buckets + 1];  // allow a color more for values equal to max
      for (unsigned int i = 0; i <= buckets; i++) {
        colormap[i] = new float[3];
      }

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

    virtual void load() {
      glColor3f(color[0], color[1], color[2] );
      glEnable (GL_TEXTURE_1D);
      glBindTexture (GL_TEXTURE_1D, defaultTexture);
    }

    virtual void unload() {
      glDisable (GL_TEXTURE_1D);
    }

    virtual void setColor(float *val) {
      glTexCoord1f( (float)((val[currentdim]-min)/extent) );
    }
    virtual void setColor(double *val) {
      glTexCoord1f( (float)((val[currentdim]-min)/extent) );
    }
    virtual void setColor(short int *val) {
      glTexCoord1f( (float)((val[currentdim]-min)/extent) );
    }
    virtual void setColor(signed char *val) {
      glTexCoord1f( (float)((val[currentdim]-min)/extent) );
    }

    virtual void setColorMap(ColorMap &cm) {
      for (unsigned int i = 0; i <= buckets; i++) {
        cm.calcColor(colormap[i], i, buckets);
      }
      convertToTexture1D();
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
      extent = max - min;
    }

  protected:


    void convertToTexture1D() {
      unsigned char *imageData = new unsigned char[(buckets+1) * 3];
      for (unsigned int i = 0; i < buckets; i++) {
        imageData[3*i+0] = colormap[i][0]*255;
        imageData[3*i+1] = colormap[i][1]*255;
        imageData[3*i+2] = colormap[i][2]*255;
      }

      imageData[3*buckets+0] = colormap[buckets][0]*255;
      imageData[3*buckets+1] = colormap[buckets][1]*255;
      imageData[3*buckets+2] = colormap[buckets][2]*255;

      if(!defaultTexture) {
		  glGenTextures(1, &defaultTexture);
      }
      glBindTexture (GL_TEXTURE_1D, defaultTexture);
      glPixelStorei (GL_UNPACK_ALIGNMENT, 1);
      glTexParameteri (GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_CLAMP);
      glTexParameteri (GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
      glTexParameteri (GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexEnvf (GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
      glTexImage1D (GL_TEXTURE_1D, 0, GL_RGB, buckets+1, 0, GL_RGB, GL_UNSIGNED_BYTE, imageData);
	  delete[] imageData;
    }

    void makeValid() {
      min = mins[currentdim];
      max = maxs[currentdim];

      extent = max - min;
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

    float extent;

    float color[3];

};

class CColorManager : public ColorManager {
  public:
    CColorManager(unsigned int buckets, unsigned int pointdim, float *mins, float *maxs, unsigned int _colordim) : ColorManager(buckets, pointdim, mins, maxs) {
      colordim = _colordim;
    }

    virtual void load() {
      glGetBooleanv(GL_COLOR_LOGIC_OP, &color_state);
      glDisable(GL_COLOR_LOGIC_OP); // this disables inversion of color, but also messes with fog behaviour
      glColor3f(color[0], color[1], color[2] );
      glEnable (GL_TEXTURE_1D);
      glBindTexture (GL_TEXTURE_1D, 0);
    }

    virtual void unload() {
      glDisable (GL_TEXTURE_1D);
      if (color_state) {
        glEnable(GL_COLOR_LOGIC_OP);
      }
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
    virtual void setColor(signed char *val) {
      GLubyte color[3];
      memcpy(color, &val[colordim], 3);
      glColor3ubv(color);
    }

  private:
    unsigned int colordim;
    GLboolean color_state;
};


#endif
