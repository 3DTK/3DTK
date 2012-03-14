/**
 *  @file
 *  @brief Representation of a 3D point type
 *  @author Jan Elsberg. Automation Group, Jacobs University Bremen gGmbH, Germany. 
 */

#include "slam6d/point_type.h"

#ifdef WITH_SCANSERVER
#include "slam6d/scan.h"
#endif //WITH_SCANSERVER

#include <string>
using std::string;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

#include <fstream>
#include <string.h>

#include <stdexcept>
using std::runtime_error;



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

#ifdef WITH_SCANSERVER
void PointType::useScan(Scan* scan)
{
  // clear pointers first
  m_xyz = 0; m_rgb = 0; m_reflectance = 0; m_amplitude = 0; m_type = 0; m_deviation = 0;
  
  // prefetch data to avoid unneccessary loading times due to split ScanHandlers
  scan->prefetch(DATA_XYZ);
  if(hasColor()) scan->prefetch(DATA_RGB);
  if(hasReflectance()) scan->prefetch(DATA_REFLECTANCE);
  if(hasAmplitude()) scan->prefetch(DATA_AMPLITUDE);
  if(hasType()) scan->prefetch(DATA_TYPE);
  if(hasDeviation()) scan->prefetch(DATA_DEVIATION);
  
  // access data
  try {
    m_xyz = new DataXYZ(scan->getXYZ());
    if(hasColor()) m_rgb = new DataRGB(scan->getRGB());
    if(hasReflectance()) m_reflectance = new DataReflectance(scan->getReflectance());
    if(hasAmplitude()) m_amplitude = new DataAmplitude(scan->getAmplitude());
    if(hasType()) m_type = new DataType(scan->getType());
    if(hasDeviation()) m_deviation = new DataDeviation(scan->getDeviation());
    
    // check if data is available, otherwise reset pointer to indicate that the scan doesn't prove this value
    if(m_rgb && !(*m_rgb)) { delete m_rgb; m_rgb = 0; }
    if(m_reflectance && !(*m_reflectance)) { delete m_reflectance; m_reflectance = 0; }
    if(m_amplitude && !(*m_amplitude)) { delete m_amplitude; m_amplitude = 0; }
    if(m_type && !(*m_type)) { delete m_type; m_type = 0; }
    if(m_deviation && !(*m_deviation)) { delete m_deviation; m_deviation = 0; }
  } catch(runtime_error& e) {
    scan->clearPrefetch();
    // unlock everything again
    clearScan();
    throw e;
  }
  
  // clear prefetch flags
  scan->clearPrefetch();
}

void PointType::clearScan()
{
  // unlock data access
  if(m_xyz) delete m_xyz;
  if(hasColor() && m_rgb) delete m_rgb;
  if(hasReflectance() && m_reflectance) delete m_reflectance;
  if(hasAmplitude() && m_amplitude) delete m_amplitude;
  if(hasType() && m_type) delete m_type;
  if(hasDeviation() && m_deviation) delete m_deviation;
}
#endif //WITH_SCANSERVER
