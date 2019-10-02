/**
 * @file
 * @brief Collection of functions to deal with GPS measurements
 * @author Dorit Borrmann.
 */
#ifndef GPS_H
#define GPS_H

#include "slam6d/globals.icc"
#include "slam6d/data_types.h"
#include "scanio/helper.h"
#include <vector>

using std::sqrt;

namespace gps {
  const double A = 6378137.0;
  const double B = 6356752.31424518;
  const double com = sqrt((gps::A*gps::A - gps::B*gps::B)/(gps::A*gps::A));
  const double E2 = 0.00669437999014;
  const double K = 0.9996;
  const double F = 1.0/298.257223563;
  const double EP = sqrt((gps::A*gps::A-gps::B*gps::B)/(gps::B*gps::B));
}

/*
void calcAlignmentMatrix(double * n0, double * mat);
void computeOrientation(double x, double y, double z, double *rPosTheta);
void computeOrientation(double x, double y, double z, double ax, double ay, double az, double *rPosTheta);
*/
void ECEF_rtoENU(double lat, double lon, double alt, double cx, double cy, double cz, double &east, double &north, double &up) ;
void ENUto3DTK(double east, double north, double up, double& x, double &y, double& z) ;
void LLAtoECEF_r(double latitude, double longitude, double altitude, double& cx, double& cy, double& cz);
void LLAtoECEF(double latitude, double longitude, double altitude, double& cx, double& cy, double& cz);
void ECEFtoLLA(double cx, double cy, double cz, double& latitude, double& longitude, double& altitude);
void LLAtoUTM(double latitude, double longitude, double altitude, double& cx, double& cy, double& cz);

void ECEFtoUTM(double cx, double cy, double cz, double& east, double& altitude, double& north);
void ECEFtoUTM(DataXYZ &xyz);
void calcECEF_rtoENUMat9(double lat, double lon, double alt, double* mat);
void getENU(double* mat, double x, double y, double z, double& east, double& north, double& up);
void readRTKPos(char* filename, std::vector<double *> &lla_vec);
#endif
