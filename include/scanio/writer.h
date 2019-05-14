#ifndef __SCAN_IO_WRITER_H__
#define __SCAN_IO_WRITER_H__

#include <iostream>
#include "slam6d/pointfilter.h"
#include "slam6d/io_types.h"
#include "slam6d/fbr/scan_cv.h"

void createdirectory(std::string dir);
void write_uos(std::vector<cv::Vec4f> &points, std::string &dir, std::string id, bool high_precision=false, volatile bool *abort_flag = nullptr);
void write_uosr(std::vector<cv::Vec4f> &points, std::string &dir, std::string id, bool high_precision=false, volatile bool *abort_flag = nullptr);
void write_uos_rgb(std::vector<cv::Vec4f> &points, std::vector<cv::Vec3b> &color, std::string &dir, std::string id, bool high_precision=false, volatile bool *abort_flag = nullptr);

void write_uos(DataXYZ &xyz, FILE *file, double scaleFac = 1.0, bool hexfloat = false, bool high_precision=false, volatile bool *abort_flag = nullptr);
void write_uosr(DataXYZ &xyz, DataReflectance &xyz_reflectance, FILE *file, double scaleFac = 1.0, bool hexfloat = false,bool high_precision=false, volatile bool *abort_flag = nullptr);
void write_uos_rgb(DataXYZ &xyz, DataRGB &rgb, FILE *file, double scaleFac = 1.0, bool hexfloat = false, bool high_precision=false, volatile bool *abort_flag = nullptr);
void write_uos_normal(DataXYZ &xyz, DataNormal &normals, FILE *file, double scaleFac = 1.0, bool hexfloat = false, bool high_precision=false, volatile bool *abort_flag = nullptr);
void write_xyz(DataXYZ &xyz, FILE *file, double scaleFac = 0.01, bool hexfloat = false, bool high_precision=false, volatile bool *abort_flag = nullptr);
void write_xyzr(DataXYZ &xyz, DataReflectance &xyz_reflectance, FILE *file, double scaleFac = 0.01, bool hexfloat = false, bool high_precision=false, volatile bool *abort_flag = nullptr);
void write_xyz_rgb(DataXYZ &xyz, DataRGB &rgb, FILE *file, double scaleFac = 0.01, bool hexfloat = false, bool high_precision=false, volatile bool *abort_flag = nullptr);
void write_xyz_normal(DataXYZ &xyz, DataNormal &normals, FILE *file, double scaleFac = 0.01, bool hexfloat = false, bool high_precision=false, volatile bool *abort_flag = nullptr);
void write_ply_rgb(std::vector<cv::Vec4f> &points, std::vector<cv::Vec3b> &color, std::string &dir, std::string id);
void writeposefile(std::string &dir, const double* rPos, const double* rPosTheta, std::string id);
void writeTrajectoryXYZ(std::ofstream &posesout, const double * transMat, bool mat, double scaleFac = 0.01);
void writeTrajectoryUOS(std::ofstream &posesout, const double * transMat, bool mat, double scaleFac = 1.0);

#endif
