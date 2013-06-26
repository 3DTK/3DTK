/*
 * scan_cv implementation
 *
 * Copyright (C) HamidReza Houshiar
 *
 * Released under the GPL version 3.
 *
 */

#include "slam6d/fbr/scan_cv.h"

using namespace std;

namespace fbr{

  scan_cv::scan_cv(string dir, unsigned int number, IOType format, bool scanServer, scanner_type type, bool lOct, bool sOct, bool Reflectance, bool Color, int MaxDist, int MinDist){
    sDir = dir;
    sNumber = number;
    sFormat = format;
    zMax = numeric_limits<double>::min(); 
    zMin = numeric_limits<double>::max();
    nPoints = 0;
    sType = type;
    scanserver = scanServer;
    loadOct = lOct;
    saveOct = sOct;
    reflectance = Reflectance;
    color = Color;
    types = PointType::USE_NONE;
    if(reflectance)
      types |= PointType::USE_REFLECTANCE;
    if(color)
      types |= PointType::USE_COLOR;
    maxDist = MaxDist;
    minDist = MinDist;
  }
  scan_cv::scan_cv(string dir, unsigned int number, IOType format, bool scanServer, scanner_type type, bool lOct, bool sOct, bool Reflectance, bool Color){
    sDir = dir;
    sNumber = number;
    sFormat = format;
    zMax = numeric_limits<double>::min(); 
    zMin = numeric_limits<double>::max();
    nPoints = 0;
    sType = type;
    scanserver = scanServer;
    loadOct = lOct;
    saveOct = sOct;
    reflectance = Reflectance;
    color = Color;
    types = PointType::USE_NONE;
    if(reflectance)
      types |= PointType::USE_REFLECTANCE;
    if(color)
      types |= PointType::USE_COLOR;
    maxDist = -1;
    minDist = -1;
  }

  scan_cv::scan_cv(string dir, unsigned int number, IOType format, bool scanServer){
    sDir = dir;
    sNumber = number;
    sFormat = format;
    zMax = numeric_limits<double>::min(); 
    zMin = numeric_limits<double>::max();
    nPoints = 0;
    sType = stringToScannerType("RIEGL");
    scanserver = scanServer;
    loadOct = false;
    saveOct = false;
    reflectance = true;
    color = false;
    types = PointType::USE_NONE;
    types |= PointType::USE_REFLECTANCE;
    maxDist = -1;
    minDist = -1;
  }

  scan_cv::scan_cv(string dir, unsigned int number, IOType format){
    sDir = dir;
    sNumber = number;
    sFormat = format;
    zMax = numeric_limits<double>::min(); 
    zMin = numeric_limits<double>::max();
    nPoints = 0;
    sType = stringToScannerType("RIEGL");
    scanserver = false;
    loadOct = false;
    saveOct = false;
    reflectance = true;
    color = false;
    types = PointType::USE_NONE;
    types |= PointType::USE_REFLECTANCE;
    maxDist = -1;
    minDist = -1;
  }

  void scan_cv::convertScanToMat(){
    Scan::openDirectory(scanserver, sDir, sFormat, sNumber, sNumber);
    if(Scan::allScans.size() == 0){
      cerr << "No scans found. Did you use the correct format?" <<endl;
      exit(-1);
    }
    cout<<"loading "<<sDir<<" with scan number " <<sNumber<<"."<<endl;
    /*
    Scan * source = * Scan::allScans.begin();
    source->get(DATA_XYZ | DATA_REFLECTANCE | DATA_RGB);
    DataXYZ xyz = source->get("xyz");
    DataReflectance xyz_reflectance = (((DataReflectance)source->get("reflectance")).size() == 0) ?
      source->create("reflectance", sizeof(float)*xyz.size())
      : source->get("reflectance"); 
    if(((DataReflectance)source->get("reflectance")).size() == 0){
      for(unsigned int i = 0; i < xyz.size(); i++)
	xyz_reflectance[i] = 255;
    }
    DataRGB xyz_rgb = source->get("rgb");

    nPoints = xyz.size();
    cv::MatIterator_<cv::Vec4f> it;
    scan.create(nPoints,1,CV_32FC(4));
    scan = cv::Scalar::all(0); 
    it = scan.begin<cv::Vec4f>();
    
    cv::MatIterator_<cv::Vec3f> itColor;
    if(xyz_rgb.size() != 0){
      scanColor.create(nPoints,1,CV_32FC(3));
      scanColor = cv::Scalar::all(0); 
      itColor = scanColor.begin<cv::Vec3f>();
    }

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
      //reflectance -= 1300;
      //reflectance /= 300;
      
      if (reflectance < 0) reflectance = 0;
      if (reflectance > 1) reflectance = 1;
      
      (*it)[0] = x;
      (*it)[1] = y;
      (*it)[2] = z;
      (*it)[3] = reflectance;
      if(xyz_rgb.size() != 0){
	(*itColor)[0] = xyz_rgb[i][2];
	(*itColor)[1] = xyz_rgb[i][1];
	(*itColor)[2] = xyz_rgb[i][0];
	++itColor;
      }

      //finding min and max of z                                      
      if (z  > zMax) zMax = z;
      if (z  < zMin) zMin = z;
      
      ++it;
    */

    PointType pointtype = PointType(types);
    //if we want to load display file get pointtypes from the files first
    if(loadOct) {
      string scanFileName = sDir + "scan" + to_string(sNumber,3) + ".oct";
      pointtype = BOctTree<sfloat>::readType(scanFileName);
    }
    ScanColorManager *scm = new ScanColorManager(4096, pointtype);  

    Scan * source =  Scan::allScans[0];

    //set Range filteres for scan
    source->setRangeFilter(maxDist, minDist);

    ///
    red = -1;
    scale = 0.01;
    voxelSize = 0.2 / scale;
    ///
    //set the parameters for the oct
    source->setOcttreeParameter(red, voxelSize, pointtype, loadOct, saveOct);
    
    //loading oct
    BOctTree<float>* btree = ((BasicScan*)source)->convertScanToShowOcttree();
    
    // show structures
    // associate show octtree with the scan and hand over octtree pointer ownership
    Show_BOctTree<sfloat>* tree = new Show_BOctTree<sfloat>(btree, scm);
    
    //get points from octtree and create panorama images
    vector<float*> points;
    tree->getTree()->AllPoints(points);

    
    nPoints = points.size();
    cv::MatIterator_<cv::Vec4f> it;
    scan.create(nPoints,1,CV_32FC(4));
    scan = cv::Scalar::all(0); 
    it = scan.begin<cv::Vec4f>();

    cv::MatIterator_<cv::Vec3f> itColor;
    if(pointtype.hasColor()){
      scanColor.create(nPoints,1,CV_32FC(3));
      scanColor = cv::Scalar::all(0); 
      itColor = scanColor.begin<cv::Vec3f>();
    }

    for(unsigned int i = 0; i < nPoints; i++){
      float x, y, z, reflectance;
      x = points[i][0];
      y = points[i][1];
      z = points[i][2];

      if(pointtype.hasReflectance()){
	int idx = pointtype.getReflectance();
	reflectance = points[i][idx];
      }
      else
	reflectance = 255;
      
                 
      //normalize the reflectance                                     
      if(sType == RIEGL){
	reflectance += 32;
	reflectance /= 64;
	reflectance -= 0.2;
	reflectance /= 0.3;
      }
      if(sType == FARO){
	reflectance -= 1250;
	reflectance /= 800;
      }
      
      if(sType != NONE){
	if (reflectance < 0) reflectance = 0;
	if (reflectance > 1) reflectance = 1;
      }
      
      (*it)[0] = x;
      (*it)[1] = y;
      (*it)[2] = z;
      (*it)[3] = reflectance;
      
      if(pointtype.hasColor()){
	int idx = pointtype.getColor();
	(*itColor)[2] = ((unsigned char*) &(points[i][idx]))[0];
	(*itColor)[1] = ((unsigned char*) &(points[i][idx]))[1];
	(*itColor)[0] = ((unsigned char*) &(points[i][idx]))[2];
	++itColor;
      }

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

  cv::Mat scan_cv::getMatScanColor()
  {
    return scanColor;
  }
  
  void scan_cv::getDescription(){
    cout<<"load "<<sDir<<", with scan number: " <<sNumber<<", with "<<nPoints<<" points, sFormat: "<<scanFormatToString(sFormat)<<" with scanner type: "<<scannerTypeToString(sType)<<"."<<endl;
    cout<<"additional info: "<<endl;
    cout<<" scanserver= "<<scanserver<<", loadOct= "<<loadOct<<", saveOct= "<<saveOct<<", reflectance = "<<reflectance<<", color= "<<color<<", voxelsize= "<<voxelSize<<", reduction= "<<red<<", scale= "<<scale<<"."<<endl;
    cout<<endl;
  }  
}
