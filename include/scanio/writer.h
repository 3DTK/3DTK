#ifndef __SCAN_IO_WRITER_H__
#define __SCAN_IO_WRITER_H__

#include <iostream>
#include "slam6d/pointfilter.h"
#include "slam6d/io_types.h"
#include "slam6d/fbr/scan_cv.h"

void createdirectory(string dir);
void write_uos(vector<cv::Vec4f> &points, string &dir, string id);
void write_uosr(vector<cv::Vec4f> &points, string &dir, string id);
void write_uos_rgb(vector<cv::Vec4f> &points, vector<cv::Vec3b> &color, string &dir, string id);

void write_uos(DataXYZ &xyz, ofstream &file);
void write_uosr(DataXYZ &xyz, DataReflectance &xyz_reflectance, ofstream &file);
void write_uos_rgb(DataXYZ &xyz, DataRGB &rgb, ofstream &file);
void write_xyz(DataXYZ &xyz, ofstream &file);
void write_xyzr(DataXYZ &xyz, DataReflectance &xyz_reflectance, ofstream &file);
void write_xyz_rgb(DataXYZ &xyz, DataRGB &rgb, ofstream &file);
void writeposefile(string &dir, const double* rPos, const double* rPosTheta, string id);
void writeTrajectoryXYZ(ofstream &posesout, const double * transMat, bool mat);
void writeTrajectoryUOS(ofstream &posesout, const double * transMat, bool mat);

#endif
