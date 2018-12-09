/*
 * point_type definition
 *
 * Copyright (C) by the 3DTK contributors
 *
 * Released under the GPL version 3.
 *
 */

/**
 *  @file
 *  @brief Representation of a 3D point type
 *  @author Jan Elsberg. Jacobs University Bremen gGmbH, Germany. 
 *  @author Andreas Nuechter. Jacobs University Bremen gGmbH, Germany. 
 */

#ifndef __POINT_TYPE_H__
#define __POINT_TYPE_H__

#include "point.h"
#include "data_types.h"

#include <string>
#include <iostream>

#include <iostream>
#include <fstream>
#include <string.h>

class Scan;

class PointType {
public:

  static const unsigned int USE_NONE = 0;
  static const unsigned int USE_REFLECTANCE = 1;
  static const unsigned int USE_NORMAL = 2;
  static const unsigned int USE_TEMPERATURE = 4;
  static const unsigned int USE_AMPLITUDE = 8;
  static const unsigned int USE_DEVIATION = 16;
  static const unsigned int USE_HEIGHT = 32;
  static const unsigned int USE_TYPE = 64;
  static const unsigned int USE_COLOR = 128;
  static const unsigned int USE_TIME = 256;
  static const unsigned int USE_INDEX = 512;

  PointType();

  PointType(unsigned int _types);

  bool hasReflectance() const;
  bool hasNormal() const;
  bool hasTemperature() const;
  bool hasAmplitude() const;
  bool hasDeviation() const;
  bool hasType() const;
  bool hasColor() const;
  bool hasTime() const;
  bool hasIndex() const;

  unsigned int getReflectance();
  unsigned int getTemperature();
  unsigned int getAmplitude();
  unsigned int getDeviation(); 
  unsigned int getTime();
  unsigned int getIndex();
  unsigned int getType();
  unsigned int getColor();
  unsigned int getType(unsigned int type);
   
  unsigned int getPointDim();

  static PointType deserialize(std::ifstream &f);
  
  void serialize(std::ofstream &f);
  unsigned int toFlags() const;
  
  template <class T>
  T *createPoint(const Point &P, unsigned int index = 0);

  template <class T>
  Point createPoint(T *p);
  
  //! Aquire DataPointer objects from \a scan, determined by its types
  void useScan(Scan* scan);
  
  //! Release the DataPointer objects
  void clearScan();
  
  //! Create a point with attributes via the DataPointers from the scan
  template<typename T>
  T* createPoint(unsigned int i, unsigned int index = 0);
  
  //! Create an array with coordinate+attribute array per point with
  //  transfer of ownership
  template<typename T>
  T** createPointArray(Scan* scan);

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
  int dimensionmap[10];

  bool hasType(unsigned int type) const;

  unsigned int getScanSize(Scan* scan);

  DataXYZ* m_xyz;
  DataXYZ* m_normal;
  DataRGB* m_rgb;
  DataReflectance* m_reflectance;
  DataTemperature* m_temperature;
  DataAmplitude* m_amplitude;
  DataType* m_type;
  DataDeviation* m_deviation;
};

#include "point_type.icc"

#endif
