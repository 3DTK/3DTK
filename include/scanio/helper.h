#ifndef __SCAN_IO_HELPER_H__
#define __SCAN_IO_HELPER_H__

#include <boost/filesystem/operations.hpp>
#include <iostream>
#include <string>
#include <sstream>
#include "slam6d/pointfilter.h"
#include "slam6d/io_types.h"
#include "slam6d/scan_settings.h"

class ScanDataTransform {
    public:
      virtual bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3]) {
        return true;
      }
};

class ScanDataTransform_identity : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3]);
};

class ScanDataTransform_ks : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3]);
};

class ScanDataTransform_riegl : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3]);
};

class ScanDataTransform_rts : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3]);
};

class ScanDataTransform_xyz : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3]);
};

class ScanDataTransform_pts : public ScanDataTransform {
    public:
        bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3]);
};

class ScanDataTransform_combined : public ScanDataTransform {
public:
  ScanDataTransform_combined(ScanDataTransform sdt1, ScanDataTransform sdt2) : m_sdt_1(sdt1), m_sdt_2(sdt2) {}
  bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3]);

private:
  ScanDataTransform m_sdt_1;
  ScanDataTransform m_sdt_2;
};

class ScanDataTransform_matrix : public ScanDataTransform {
public:
  ScanDataTransform_matrix(double P[16]) {
    memcpy(m_matrix, P, sizeof(m_matrix));
  }
  bool transform(double xyz[3], unsigned char rgb[3], float*  refl, float* temp, float* ampl, int* type, float* devi, double n[3]);

private:
  double m_matrix[16];
};

time_t lastModifiedHelper(
        const char *dir_path,
        const char *identifier,
        const char **data_path_suffixes,
        const char* data_path_prefix = "scan",
        unsigned int id_len = 3);

std::list<std::string> readDirectoryHelper(
        const char* dir_path,
        unsigned int start,
        unsigned int end,
        const char** data_path_suffix,
        const char* data_path_prefix = "scan",
        unsigned int id_len = 3);
std::list<std::string> readDirectoryHelper(
        dataset_settings& dss,
        const char** data_path_suffix,
        const char* data_path_prefix = "scan",
        unsigned int id_len = 3);
void readPoseHelper(
        const char* dir_path,
        const char* identifier,
        double* pose,
        const char* pose_path_suffix = ".pose",
        const char* pose_path_prefix = "scan");
std::function<bool (std::istream &data_file)> open_uos_file(
        IODataType* spec, ScanDataTransform& transform, PointFilter& filter,
        std::vector<double>* xyz, std::vector<unsigned char>* rgb,
        std::vector<float>* reflectance, std::vector<float>* temperature,
        std::vector<float>* amplitude, std::vector<int>* type,
        std::vector<float>* deviation,
        std::vector<double>* normal);
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
        std::vector<double>* normal = 0,
        std::streamsize bufsize = 128);

unsigned int strtoarray(std:: string opts, char **&opts_array, const char * deliminator=" ");

bool open_path(boost::filesystem::path data_path, std::function<bool (std::istream &)>);
bool open_path_writing(boost::filesystem::path data_path, std::function<bool (std::ostream &)> handler);

#ifdef WITH_LIBZIP
bool write_multiple(std::map<std::string,std::string> contentmap);
#endif

#endif

/* vim: set ts=4 sw=4 et: */
