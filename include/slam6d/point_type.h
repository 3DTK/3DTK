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

#include "slam6d/point.h"

#ifndef __POINT_TYPE_H__
#define __POINT_TYPE_H__


class PointType {
public:

  static const unsigned int USE_NONE;
  static const unsigned int USE_REFLECTANCE;
  static const unsigned int USE_AMPLITUDE;
  static const unsigned int USE_DEVIATION;
  static const unsigned int USE_HEIGHT;
  static const unsigned int USE_TYPE;
  static const unsigned int USE_COLOR;
  static const unsigned int USE_TIME;
  static const unsigned int USE_INDEX;

  PointType();

  PointType(unsigned int _types);

  bool hasReflectance();
  bool hasAmplitude();
  bool hasDeviation();
  bool hasType();
  bool hasColor();
  bool hasTime();
  bool hasIndex();

  unsigned int getReflectance();
  unsigned int getAmplitude();
  unsigned int getDeviation(); 
  unsigned int getTime();
  unsigned int getIndex();
  unsigned int getType();
  unsigned int getType(unsigned int type);
   

  unsigned int getPointDim();

  static PointType deserialize(std::ifstream &f);
  
  void serialize(std::ofstream &f);
  unsigned int toFlags() const;
  
  template <class T>
  T *createPoint(const Point &P, unsigned int index=0);

  template <class T>
  Point createPoint(T *p);

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
   * Stores the size of each point in bytes
   **/
  unsigned int pointbytes;

  /**
   * Derived from types, to map type to the array index for each point
   **/
  int dimensionmap[8];

  bool hasType(unsigned int type);

};


  template <class T>
  T *PointType::createPoint(const Point &P, unsigned int index ) {
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
    if (types & USE_TIME) {  
//      p[counter++] = P.timestamp;
    }
    if (types & USE_INDEX) {  
      p[counter++] = index; 
    }

    return p;
  }

  template <class T>
  Point PointType::createPoint(T *p) {
    Point P;
    unsigned int counter = 0;

    P.x = p[counter++];
    P.y = p[counter++];
    P.z = p[counter++];
    if (types & USE_REFLECTANCE) {
      P.reflectance = p[counter++];
    }
    if (types & USE_AMPLITUDE) {
      P.amplitude = p[counter++];
    }
    if (types & USE_DEVIATION) {  
      P.deviation = p[counter++];
    }
    if (types & USE_TYPE) {  
      P.type = p[counter++];
    }
    if (types & USE_COLOR) {  
      memcpy(P.rgb, &p[counter], 3);
      counter++;
    }
    if (types & USE_TIME) {  
//      P.timestamp = p[counter++];
    }


    return P;
  }
#endif
