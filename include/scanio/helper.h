#ifndef __SCAN_IO_HELPER_H__
#define __SCAN_IO_HELPER_H__

#include <iostream>
#include "slam6d/pointfilter.h"
#include "slam6d/io_types.h"

class ScanDataTransform {
    public:
        virtual bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi) = 0;
};

class ScanDataTransform_identity : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi);
};

class ScanDataTransform_ks : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi);
};

class ScanDataTransform_riegl : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi);
};

class ScanDataTransform_rts : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi);
};

class ScanDataTransform_xyz : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi);
};

std::list<std::string> readDirectoryHelper(
        const char* dir_path,
        unsigned int start,
        unsigned int end,
        const char** data_path_suffix,
        const char* data_path_prefix = "scan");
void readPoseHelper(
        const char* dir_path,
        const char* identifier,
        double* pose,
        const char* pose_path_suffix = ".pose",
        const char* pose_path_prefix = "scan");
void uosHeaderTest(std::istream& infile, std::streamsize bufsize = 128);
bool uosHeaderTest(char *line);
bool readASCII(std::istream& infile,
        IODataType* spec,
        ScanDataTransform& transform,
        PointFilter& filter,
        std::vector<double>* xyz = 0,
        std::vector<unsigned char>* rgb = 0,
        std::vector<float>* reflectance = 0,
        std::vector<float>* temperature = 0,
        std::vector<float>* amplitude = 0,
        std::vector<int>* type = 0,
        std::vector<float>* deviation = 0,
        std::streamsize bufsize = 128);
#endif
