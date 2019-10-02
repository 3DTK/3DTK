/**
 * @file
 * @brief Collection of functions to deal with GPS measurements
 * @author Dorit Borrmann.
 */

#include "gps/gps.h"

#include <fstream>
#include <iostream>

/**
  * Convert from the standard right handed ECEF coordinate system to ENU
  * coordinates
  */
void ECEF_rtoENU(double lat, double lon, double alt, double cx, double cy, double cz, double &east, double &north, double &up) {
  alt *= 100;
  east = sin(rad(lon))*cx + cos(rad(lon))*cy;
  north = -sin(rad(lat))*cos(rad(lon))*cx - sin(rad(lat))*cos(rad(lon))*cy + cos(rad(lat))*cz;
  up = cos(rad(lat))*cos(rad(lon))*cx + cos(rad(lat))*sin(rad(lon))*cy + sin(rad(lat))*cz;
}

/**
  * Calculate the matrix for conversion into ENU coordinates based on latitude,
  * longitude, altitude (=WGS84/ellipsoidal, in m)
  */
void calcECEF_rtoENUMat9(double lat, double lon, double alt, double* mat) {
  alt *= 100;
  mat[0] = sin(rad(lon));
  mat[1] = cos(rad(lon));
  mat[2] = 0;
  mat[3] = -sin(rad(lat))*cos(rad(lon));
  mat[4] = -sin(rad(lat))*cos(rad(lon));
  mat[5] = cos(rad(lat));
  mat[6] = cos(rad(lat))*cos(rad(lon));
  mat[7] = cos(rad(lat))*sin(rad(lon));
  mat[8] = sin(rad(lat));
}

/**
  * Calculate ENU coordinates with x = cx - fx, y = cy - fy, z = cz - fz, where
  * (cx, cy, cz) is the point used to calculate mat.
  */
void getENU(double* mat, double x, double y, double z, double& east, double& north, double&
up) {
  east = mat[0] * x + mat[1] * y + mat[2] * z;
  north = mat[3] * x + mat[4] * y + mat[5] * z;
  up = mat[6] * x + mat[7] * y + mat[8] * z;
}

/**
  * Transform ENU coordinates into a left-handed coordinate system.
  */
void ENUto3DTK(double east, double north, double up, double& x, double &y, double& z) {
  x = east;
  y = up;
  z = north;
}

/**
  * Conversion from latitude, longitude, altitude (=WGS84/ellipsoidal, in m) to
  * right-handed ECEF in cm
  */
void LLAtoECEF_r(double latitude, double longitude, double altitude, double& cx, double& cy, double& cz) {
  double phi = rad(latitude);
  double lambda = rad(longitude);
  double h = altitude;

  double n = (gps::A/(sqrt(1-(gps::E2*(sin(phi)*sin(phi))))));

  cx = (n+h)*cos(phi)*cos(lambda) * 100;  // ECEF X
  cy = (n+h)*cos(phi)*sin(lambda) * 100; // ECEF Y
  cz= ((n*(1-gps::E2))+h)*sin(phi) * 100; // ECEF Z // !!
}

/**
  * Conversion from latitude, longitude, altitude (=WGS84/ellipsoidal, in m) to
  * left-handed ECEF in cm
  */
void LLAtoECEF(double latitude, double longitude, double altitude, double& cx, double& cy, double& cz) {
  double phi = rad(latitude);
  double lambda = rad(longitude);
  double h = altitude;

  double n = (gps::A/(sqrt(1-(gps::E2*(sin(phi)*sin(phi))))));

  cz = (n+h)*cos(phi)*cos(lambda) * 100;  // ECEF X
  cx = (n+h)*cos(phi)*sin(lambda) * -100; // ECEF -Y
  cy= ((n*(1-gps::E2))+h)*sin(phi) * 100; // ECEF Z // !!
}

/**
  * Conversion from left-handed ECEF in cm to
  latitude, longitude, altitude (=WGS84/ellipsoidal, in m)
  */
void ECEFtoLLA(double cx, double cy, double cz, double& latitude, double& longitude, double& altitude) {
  double x, y, z;
  x = cz * -0.01;
  y = cx * 0.01;
  z = cy * 0.01;

  //from ECEF to WGS84

  double lambda = atan2(y,x);
  double p = sqrt(x*x + y*y);
  double theta = atan2(z*gps::A,p*gps::B);
  double phi = atan2((z+gps::EP*gps::EP*gps::B*pow(sin(theta),3)), (p-gps::E2*gps::A*pow(cos(theta),3)));
  double n = gps::A/sqrt(1-gps::com*gps::com*sin(phi)*sin(phi));
  double h = p/cos(phi) - n;
  latitude = deg(phi);
  longitude = deg(lambda);
  altitude = h;
}

/**
  * Conversion from latitude, longitude, altitude (=WGS84/ellipsoidal, in m)
  * to UTM in m (East, Height, North)
  */
void LLAtoUTM(double latitude, double longitude, double altitude, double& east, double& height, double& north) {
  //cerr << latitude << " " << longitude << " " << altitude << endl;
  height = altitude; // height over ellipsoid
  double lambda_0 = (((int)((longitude + 180)/6)*6)-177);
  double phi = rad(latitude);
  double lambda = cos(phi) * rad(longitude - lambda_0);

  double n = (gps::A/(sqrt(1-(gps::E2*(sin(phi)*sin(phi))))));
  double t = tan(phi);
  double n2 = gps::E2/(1.0-gps::E2)*(cos(phi)*cos(phi));

  double x_0 = 500000; // False Easting

  // help variables
  double tmpA = 1.0 + 3.0/4.0*gps::E2 + 45.0/64.0*pow(gps::E2,2) + 175.0/256.0*pow(gps::E2,3) + 11025/16384*pow(gps::E2,4);
  double tmpB = 3.0/8.0*gps::E2 + 15.0/32.0*pow(gps::E2,2) + 525.0/1024.0*pow(gps::E2,3) + 2205.0/4096.0*pow(gps::E2,4);
  double tmpC = 15.0/256.0*pow(gps::E2,2) + 105.0/1024.0*pow(gps::E2,3) + 2205.0/16384.0*pow(gps::E2,4);
  double tmpD = 35.0/3072.0*pow(gps::E2,3) + 105.0/4096.0*pow(gps::E2,4);

  // Meridian arc
  double y = gps::A*(1.0-gps::E2)*(tmpA*phi - tmpB*sin(2.0*phi) + tmpC*sin(4.0*phi) - tmpD*sin(6.0*phi));

  // help variables
  double p = 1.0/12.0*(5.0 - pow(t,2) + 9.0*n2 + 4.0*pow(n2,2));
  double q = 1.0/360.0*(61.0 - 58.0*pow(t,2) + pow(t,4) + 270.0*n2 - 330.0*pow(t,2)*n2);
  double r = 1.0/6.0*(1.0 - pow(t,2) + n2);
  double s = 1.0/120.0*(5.0 - 18.0*pow(t,2) + pow(t,4) + 14.0*n2 - 58.0*pow(t,2)*n2);
  double u = 1.0/5040.0*(61.0 - 479.0*pow(t,2) + 179.0*pow(t,4) - pow(t,6));
  // x and correction to y
  double x = lambda*n*(1.0 + r*pow(lambda,2) + s*pow(lambda,4) + u*pow(lambda,6));
  double dy = 0.5*pow(lambda,2)*n*t*(1.0 + p*pow(lambda,2) + q*pow(lambda,4));
  // Northing and Easting
  north = gps::K*(y+dy);
  east = gps::K*x + x_0;

}

/**
  * Conversion from left-handed ECEF in cm to
  * to UTM in m (East, Height, North)
  */
void ECEFtoUTM(double cx, double cy, double cz, double& east, double& height, double& north) {
  double longitude, latitude, altitude;
  ECEFtoLLA(cx, cy, cz, latitude, longitude, altitude);
  LLAtoUTM(latitude, longitude, altitude, east, height, north);
}


/**
  * Conversion from left-handed ECEF in cm to
  * to UTM in m (East, Height, North)
  */
void ECEFtoUTM(DataXYZ &xyz)
{
  double north, east, height;
  for(unsigned int j = 0; j < xyz.size(); j++) {
    ECEFtoUTM(xyz[j][0], xyz[j][1], xyz[j][2], east, height, north);
    xyz[j][0] = east;
    xyz[j][1] = north;
    xyz[j][2] = height;
  }
}

/**
  * Reading LLA from Emlid Reach RTK
  *
  */
void readRTKPos(char* filename, std::vector<double *> &lla_vec) {
//index,name,longitude,latitude,elevation,collection start,collection end,solution status,lateral rms,sample count,antenna height
  std::ifstream infile;
  unsigned int bufsize = 255;
  char *buffer = (char *)malloc(bufsize);
  unsigned int linenr = 0;
  double latitude, longitude, altitude;


  infile.open(filename);
  for (;;++linenr) {
    if (infile.eof()) break;
    try {
      infile.getline(buffer, bufsize, '\n');
    } catch(std::ios_base::failure e) {
      if (!infile.eof()) {
        std::cerr << "error reading a line in line " << linenr << std::endl;
        std::cerr << e.what() << std::endl;
        break;
      }
    }
    if(linenr == 0) continue;
    char **rtk_array;
    unsigned int count = strtoarray(buffer, rtk_array,",");
    /*
    std::cout << "B" << std::endl;
    std::cout << count << std::endl;
    std::cout << rtk_array[0] << std::endl;
    std::cout << rtk_array[1] << std::endl;
    std::cout << rtk_array[2] << std::endl;
    std::cout << rtk_array[3] << std::endl;
    std::cout << rtk_array[4] << std::endl;
    std::cout << rtk_array[5] << std::endl;

    for(unsigned int jl = 0; jl < count; jl++) {
      std::cout << rtk_array[jl] << std::endl;
    }
    */


    if(count != 11) {
      //std::cerr << count << " error reading RTK pose format in line " << linenr << std::endl;
      break;
    }
    /*
    std::cout << rtk_array[3] << std::endl;
    std::cout << rtk_array[4] << std::endl;
    std::cout << rtk_array[5] << std::endl;
    */
    latitude = atof(rtk_array[4]);
    longitude = atof(rtk_array[3]);
    //TODO need antenna height??
    altitude = atof(rtk_array[5]);
    double *lla = new double[3];
    lla[0] = latitude;
    lla[1] = longitude;
    lla[2] = altitude;
    lla_vec.push_back(lla);

    for(unsigned int i = 0; i < count; i++) {
      delete[] rtk_array[i];
    }
    delete[] rtk_array;

  }
  free(buffer);
  infile.close();
  infile.clear();
}
