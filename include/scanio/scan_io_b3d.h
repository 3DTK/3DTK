/**
 * @file
 * @brief IO of a 3D scan in uos file format
 */

#ifndef __SCAN_IO_B3D_H__
#define __SCAN_IO_B3D_H__

#include "scan_io.h"



/**
 * @brief 3D scan loader for UOS scans
 *
 * The compiled class is available as shared object file
 */
class ScanIO_b3d : public ScanIO {
    bool hasreflectance;  
    bool hasamplitude; 
    bool hasintensity;
    bool hasrgb;
    bool hastemperature;
    bool hastype;   
    
    int reflectanceidx;  
    int amplitudeidx; 
    int intensityidx;
    int rgbidx;
    int temperatureidx;
    int typeidx;   

public:
  virtual std::list<std::string> readDirectory(const char* dir_path, unsigned int start, unsigned int end);
  virtual void readPose(const char* dir_path, const char* identifier, double* pose);
  virtual time_t lastModified(const char* dir_path, const char* identifier);
  virtual void readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation);
  virtual bool supports(IODataType type);
  bool readEOL(std::ifstream &f);
};

#endif
