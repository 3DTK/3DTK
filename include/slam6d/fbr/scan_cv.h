/**
 * @file scan_cv.h
 * @brief load 3D Scans in opencv format for featuer based registration.
 * This class is an 3D scan container for scans that have different scan
 * file formats and add them to a OpenCV Mat data type also normalize the
 * reflectance.
 * @author HamidReza Houshiar. Jacobs University Bremen gGmbH, Germany.
 * @date Date: 2012/05/9 2:00
 */

#ifndef SCAN_CV_H_
#define SCAN_CV_H_

#include "fbr_global.h"
#include "slam6d/scan.h"
#include "slam6d/managedScan.h"
#undef max
#undef min
#include <limits>
#include "show/scancolormanager.h"
#include "show/show_Boctree.h"
#include "slam6d/point_type.h"
#include "show/show.h"
#include "slam6d/data_types.h"
#include "slam6d/Boctree.h"
#include "slam6d/basicScan.h"

namespace fbr{
  /**
   * @class scan_cv
   * @brief class for 3D scans in open CV Mat data type
   * @param sDir name and dir of the input scan file
   * @param scan Mat data type which contains the 3D Scan
   * @param scanColor Mat data type which contains the color of the 3D Scan
   * @param sNumber input scan number
   * @param nPoints Number of points
   * @param zmax max value in z direction
   * @param zmin min value in z direction
   * @param sFormat input scan file format in IOType
   * @param scanserver 
   * @param loadOct
   * @param saveOct
   */
  class scan_cv{
    string sDir;
    cv::Mat scan;
    cv::Mat scanColor;
    unsigned int sNumber;
    unsigned int nPoints;
    double zMax;
    double zMin;
    IOType sFormat;
    bool scanserver;
    bool loadOct;
    bool saveOct;
    bool reflectance, color;
    unsigned int types;
    double voxelSize, red, scale;
    scanner_type sType;
    int maxDist, minDist;
  public:
    /**
     * constructor of class scan_cv
     * @param dir directory of the input scan file
     * @param number input scan number
     * @param format input scan file format
     * @param scanServer 
     * @param loadOct
     * @param saveOct
     */
    scan_cv (string dir, unsigned int number, IOType format, bool scanServer, scanner_type type, bool lOct, bool sOct, bool Reflectance, bool Color, int MaxDist, int MinDist);
    scan_cv (string dir, unsigned int number, IOType format, bool scanServer, scanner_type type, bool lOct, bool sOct, bool Reflectance, bool Color);
    scan_cv (string dir, unsigned int number, IOType format, bool scanServer);
    scan_cv (string dir, unsigned int number, IOType format);

    /**
     * @brief read scan file and convert it to open cv Mat
     */
    void convertScanToMat();
    string getScanDir();
    unsigned int getScanNumber();
    unsigned int getNumberOfPoints();
    double getZMin();
    double getZMax();
    IOType getScanFormat();
    cv::Mat getMatScan();
    cv::Mat getMatScanColor();
    void getDescription();
  };
}
#endif /* SCAN_CV_H_ */
