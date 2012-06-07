#include "slam6d/fbr/scan_cv.h"

using namespace std;

namespace fbr{

  scan_cv::scan_cv(string dir, unsigned int number, IOType format){
    sDir = dir;
    sNumber = number;
    sFormat = format;
    zMax = numeric_limits<double>::min(); 
    zMin = numeric_limits<double>::max();
    nPoints = 0;
  } 
  
  void scan_cv::convertScanToMat(){
    bool scanserver = false;
    Scan::openDirectory(scanserver, sDir, sFormat, sNumber, sNumber);
    cout<<"loading "<<sDir<<" with scan number " <<sNumber<<"."<<endl;
    Scan * source = * Scan::allScans.begin();
    DataXYZ xyz = source->get("xyz");
    DataReflectance xyz_reflectance = source->get("reflectance");
    nPoints = xyz.size();
    scan.create(nPoints,1,CV_32FC(4));
    scan = cv::Scalar::all(0); 
    cv::MatIterator_<cv::Vec4f> it;
    it = scan.begin<cv::Vec4f>();
    for(unsigned int i = 0; i < nPoints; i++){
      float x, y, z, reflectance;
      x = xyz[i][0];
      y = xyz[i][1];
      z = xyz[i][2];
      reflectance = xyz_reflectance[i];
      
      //normalize the reflectance                                     
      reflectance += 32;
      reflectance /= 64;
      reflectance -= 0.2;
      reflectance /= 0.3;
      if (reflectance < 0) reflectance = 0;
      if (reflectance > 1) reflectance = 1;
      
      (*it)[0] = x;
      (*it)[1] = y;
      (*it)[2] = z;
      (*it)[3] = reflectance;
      
      //finding min and max of z                                      
      if (z  > zMax) zMax = z;
      if (z  < zMin) zMin = z;
      
      ++it;
    }
    Scan::closeDirectory();
  }

  string scan_cv::getScanDir(){
    return sDir;
  }
  
  unsigned int scan_cv::getScanNumber(){
    return sNumber;
  }
  
  unsigned int scan_cv::getNumberOfPoints(){
    return nPoints;
  }

  double scan_cv::getZMin(){
    return zMin;
  }

  double scan_cv::getZMax(){
    return zMax;
  }

  IOType scan_cv::getScanFormat(){
    return sFormat;
  }
  
  cv::Mat scan_cv::getMatScan()
  {
    return scan;
  }
  
  void scan_cv::getDescription(){
    cout<<"load "<<sDir<<", with scan number: " <<sNumber<<", with "<<nPoints<<" points, sFormat: "<<scanFormatToString(sFormat)<<"."<<endl;
    cout<<endl;
  }
}
