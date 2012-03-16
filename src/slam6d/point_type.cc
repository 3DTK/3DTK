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
#include "slam6d/point_type.h"



PointType::PointType() {
  types = USE_NONE;
  pointdim = 3;
  dimensionmap[1] = dimensionmap[2] = dimensionmap[3] = dimensionmap[4] = dimensionmap[5] = dimensionmap[6] = dimensionmap[7] = 1; // choose height per default  
  dimensionmap[0] = 1;  // height 
}

PointType::PointType(unsigned int _types) : types(_types) {
  dimensionmap[1] = dimensionmap[2] = dimensionmap[3] = dimensionmap[4] = dimensionmap[5] = dimensionmap[6] = dimensionmap[7] = 1; // choose height per default  
  dimensionmap[0] = 1;  // height 

  pointdim = 3;
  if (types & PointType::USE_REFLECTANCE) dimensionmap[1] = pointdim++;  
  if (types & PointType::USE_AMPLITUDE) dimensionmap[2] = pointdim++;  
  if (types & PointType::USE_DEVIATION) dimensionmap[3] = pointdim++;  
  if (types & PointType::USE_TYPE) dimensionmap[4] = pointdim++; 
  if (types & PointType::USE_COLOR) dimensionmap[5] = pointdim++; 
  if (types & PointType::USE_TIME) dimensionmap[6] = pointdim++; 
  if (types & PointType::USE_INDEX) dimensionmap[7] = pointdim++; 
}

bool PointType::hasReflectance() {
  return hasType(USE_REFLECTANCE); 
}
bool PointType::hasAmplitude() {
  return hasType(USE_AMPLITUDE); 
}
bool PointType::hasDeviation() {
  return hasType(USE_DEVIATION); 
}
bool PointType::hasType() {
  return hasType(USE_TYPE); 
}
bool PointType::hasColor() {
  return hasType(USE_COLOR); 
}
bool PointType::hasTime() {
  return hasType(USE_TIME); 
}

bool PointType::hasIndex() {
  return hasType(USE_INDEX); 
}

unsigned int PointType::getReflectance() {
  return dimensionmap[1];
}

unsigned int PointType::getAmplitude() {
  return dimensionmap[2];
}

unsigned int PointType::getDeviation() {
  return dimensionmap[3];
}

unsigned int PointType::getTime() {
  return dimensionmap[6];
}

unsigned int PointType::getIndex() {
  return dimensionmap[7];
}

unsigned int PointType::getType() {
  return dimensionmap[4];
}

unsigned int PointType::getType(unsigned int type) {
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
  } else if (type == USE_TIME) {
    return dimensionmap[6];
  } else {
    return 0;
  }
}


unsigned int PointType::getPointDim() { return pointdim; }

PointType PointType::deserialize(std::ifstream &f) {
  unsigned int types;
  f.read(reinterpret_cast<char*>(&types), sizeof(unsigned int));
  return PointType(types);
}

void PointType::serialize(std::ofstream &f) {
  f.write(reinterpret_cast<char*>(&types), sizeof(unsigned int));
}

unsigned int PointType::toFlags() const { return types; } 

bool PointType::hasType(unsigned int type) {
  return types & type;
}


const unsigned int PointType::USE_NONE = 0;
const unsigned int PointType::USE_REFLECTANCE = 1;
const unsigned int PointType::USE_AMPLITUDE = 2;
const unsigned int PointType::USE_DEVIATION = 4;
const unsigned int PointType::USE_HEIGHT = 8;
const unsigned int PointType::USE_TYPE = 16;
const unsigned int PointType::USE_COLOR = 32;
const unsigned int PointType::USE_TIME = 64;
const unsigned int PointType::USE_INDEX = 128;

