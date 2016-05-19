#ifndef __B3DPSFILE_H__
#define __B3DPSFILE_H__

#include <stdint.h>
//#include <Eigen/Core>
//#include <Eigen/Geometry>
#include <cstdlib>
#include <iomanip>
#include <map>
using std::map;
#include <vector>
using std::vector;
#include <sstream>
using std::stringstream;
using std::string;
#include <iostream>
#include <fstream>
#include <queue>
using std::queue;
using std::cout;
using std::cerr;
using std::endl;

class B3DPSFile {
protected:
    bool hasreflectance;  
    bool hasamplitude; 
    bool hasintensity;
    bool hasrgb;
    bool hastemperature;
    bool hastype;   
    double calibration[16];
//    Eigen::Affine3d calibration;
    int idxreflectance;
    int idxamplitude;
    int idxintensity; 
    int idxrgb; 
    int idxtemperature;
    int idxtype;
    
    typedef double B3Dfloat64;
    typedef float B3Dfloat32;
    typedef uint16_t B3Dint;
    static const B3Dint EOL = 0xFFFF;
    
    int bytesperpoint;
    int pointdim;

public:
    void initPointType(bool _reflectance = false,
            bool _amplitude = false,
            bool _intensity = false,
            bool _rgb = false,
            bool _temperature = false,
            bool _type = false) {

                // TODO encapsulate the meta point info stuff into its own class
      hasreflectance  = _reflectance;
      hasamplitude    = _amplitude;
      hasintensity    = _intensity; 
      hasrgb          = _rgb; 
      hastemperature  = _temperature;
      hastype         = _type; 
     
      // TODO FIXME use only one of the two (bytesperpoint) and implement a point packing and unpacking function
      bytesperpoint = 4 * sizeof(B3Dfloat64);
      pointdim = 4;
      
        
      if(hasreflectance ) { bytesperpoint += sizeof(B3Dfloat64);idxreflectance = pointdim++; } // TODO FIXME work with float instead of double...
      if(hasintensity   ) { bytesperpoint += sizeof(B3Dfloat64);idxintensity   = pointdim++; } 
      if(hasamplitude   ) { bytesperpoint += sizeof(B3Dfloat64);idxamplitude   = pointdim++; } 
      if(hastemperature ) { bytesperpoint += sizeof(B3Dfloat64);idxtemperature = pointdim++; } 
      if(hastype        ) { bytesperpoint += sizeof(B3Dint)    ;idxtype        = pointdim++; } 
      if(hasrgb         ) { bytesperpoint += 3*sizeof(char)    ;idxrgb         = pointdim++; } 
//      calibration = Eigen::Translation3d(-0.263633, 0.0, 0.004408) * Eigen::AngleAxisd(-2.62743611, Eigen::Vector3d::UnitZ());  // -150
     }

    void setCalibration(double *calimat) {
      for (int i=0; i < 16; i++) { calibration[i] = calimat[i]; }
    }

    void setCalibration(double *rPos, double *rPT) {
      EulerToMatrix4(rPos, rPT, calibration);
    }


};




#endif
