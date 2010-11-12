/**
 *  @file
 *  @brief Representation of a 3D point type
 *  @author Jan Elsberg. Automation Group, Jacobs University Bremen gGmbH, Germany. 
 */
#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <iostream>
#include <fstream>
#include <string.h>

#include "point.h"

#ifndef __POINT_TYPE_H__
#define __POINT_TYPE_H__

template <class T=double> class PointType {
public:

  static const unsigned int USE_NONE;
  static const unsigned int USE_REFLECTANCE;
  static const unsigned int USE_AMPLITUDE;
  static const unsigned int USE_DEVIATION;
  static const unsigned int USE_HEIGHT;
  static const unsigned int USE_TYPE;
  static const unsigned int USE_COLOR;

  PointType() {
    types = USE_NONE;
    pointdim = 3;
    dimensionmap[1] = dimensionmap[2] = dimensionmap[3] = dimensionmap[4] = dimensionmap[5] = 1; // choose height per default  
    dimensionmap[0] = 1;  // height 
  }

  PointType(unsigned int _types) : types(_types) {
    dimensionmap[1] = dimensionmap[2] = dimensionmap[3] = dimensionmap[4] = dimensionmap[5] = 1; // choose height per default  
    dimensionmap[0] = 1;  // height 

    pointdim = 3;
    if (types & PointType::USE_REFLECTANCE) dimensionmap[1] = pointdim++;  
    if (types & PointType::USE_AMPLITUDE) dimensionmap[2] = pointdim++;  
    if (types & PointType::USE_DEVIATION) dimensionmap[3] = pointdim++;  
    if (types & PointType::USE_TYPE) dimensionmap[4] = pointdim++; 
    if (types & PointType::USE_COLOR) dimensionmap[5] = pointdim++; 
  }

  bool hasReflectance() {
    return hasType(USE_REFLECTANCE); 
  }
  bool hasAmplitude() {
    return hasType(USE_AMPLITUDE); 
  }
  bool hasDeviation() {
    return hasType(USE_DEVIATION); 
  }
  bool hasType() {
    return hasType(USE_TYPE); 
  }
  bool hasColor() {
    return hasType(USE_COLOR); 
  }

  unsigned int getType(unsigned int type) {
    if (type == USE_NONE ) {
      return dimensionmap[0];
    } else if (type == USE_HEIGHT) {
      return dimensionmap[0];
    } else if (type == USE_REFLECTANCE) {
      return dimensionmap[1];
    } else if (type == USE_AMPLITUDE) {
      return dimensionmap[2];
    } else if (type == USE_DEVIATION) {
      return dimensionmap[3];
    } else if (type == USE_TYPE) {
      return dimensionmap[4];
    } else if (type == USE_COLOR) {
      return dimensionmap[5];
    } else {
      return 0;
    }
  }
    
  T *createPoint(const Point &P) {
    unsigned int counter = 0;

    T *p = new T[pointdim];
    p[counter++] = P.x;
    p[counter++] = P.y;
    p[counter++] = P.z;
    if (types & USE_REFLECTANCE) {
      p[counter++] = P.reflectance;
    }
    if (types & USE_AMPLITUDE) {
      p[counter++] = P.amplitude;
    }
    if (types & USE_DEVIATION) {  
      p[counter++] = P.deviation;
    }
    if (types & USE_TYPE) {  
      p[counter++] = P.type;
    }
    if (types & USE_COLOR) {  
      memcpy(&p[counter], P.rgb, 3);
      counter++;
    }

    return p;
  }

  unsigned int getPointDim() { return pointdim; }

  static PointType<T> deserialize(std::ifstream &f) {
    unsigned int types;
    f.read(reinterpret_cast<char*>(&types), sizeof(unsigned int));
    return PointType<T>(types);
  }
  
  void serialize(std::ofstream &f) {
    f.write(reinterpret_cast<char*>(&types), sizeof(unsigned int));
  }

private:
  /**
   * collection of flags 
   */
  unsigned int types;
  /**
   * Derived from types: 3 spatial dimensions + 1 for each flag set
   **/
  unsigned int pointdim;

  /**
   * Derived from types, to map type to the array index for each point
   **/
  int dimensionmap[6];

  bool hasType(unsigned int type) {
    return types & type;
  }

};


template <class T> const unsigned int PointType<T>::USE_NONE = 0;
template <class T> const unsigned int PointType<T>::USE_REFLECTANCE = 1;
template <class T> const unsigned int PointType<T>::USE_AMPLITUDE = 2;
template <class T> const unsigned int PointType<T>::USE_DEVIATION = 4;
template <class T> const unsigned int PointType<T>::USE_HEIGHT = 8;
template <class T> const unsigned int PointType<T>::USE_TYPE = 16;
template <class T> const unsigned int PointType<T>::USE_COLOR = 32;

#endif
