/*
 * ConfigFileHough implementation
 *
 * Copyright (C) Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#include "shapes/ConfigFileHough.h"
#include <string.h>
#include <stdio.h>
#include "shapes/parascan.h"
#include <iostream>

using namespace std;

ConfigFileHough::ConfigFileHough() {

  strcpy(CfgFileName, DEF_CfgFileName);
  MaxDist = DEF_MaxDist;
  MinDist = DEF_MinDist;
  AccumulatorMax = DEF_AccumulatorMax;
  MinSizeAllPoints = DEF_MinSizeAllPoints;
  RhoNum = DEF_RhoNum;
  ThetaNum = DEF_ThetaNum;
  PhiNum = DEF_PhiNum;
  RhoMax = DEF_RhoMax;
  MaxPointPlaneDist = DEF_MaxPointPlaneDist;
  MaxPlanes = DEF_MaxPlanes;
  MinPlaneSize = DEF_MinPlaneSize;
  MinPlanarity = DEF_MinPlanarity;
  PlaneRatio = DEF_PlaneRatio;
  PointDist = DEF_PointDist;
  PeakWindow = DEF_PeakWindow;
  WindowSize = DEF_WindowSize;
  TrashMax = DEF_TrashMax;
  AccumulatorType = DEF_AccumulatorType;
  strcpy(PlaneDir, DEF_PlaneDir);

}

void ConfigFileHough::ShowConfiguration() {
  cout << "MaxDist " << Get_MaxDist() << endl;
  cout << "MinDist " << Get_MinDist() << endl;
  cout << "AccumulatorMax " << Get_AccumulatorMax() << endl;
  cout << "MinSizeAllPoints " << Get_MinSizeAllPoints() << endl;
  cout << "RhoNum " << Get_RhoNum() << endl;
  cout << "ThetaNum " << Get_ThetaNum() << endl;
  cout << "PhiNum " << Get_PhiNum() << endl;
  cout << "RhoMax " << Get_RhoMax() << endl;
  cout << "MaxPointPlaneDist " << Get_MaxPointPlaneDist() << endl;
  cout << "MaxPlanes " << Get_MaxPlanes() << endl;
  cout << "MinPlaneSize " << Get_MinPlaneSize() << endl;
  cout << "MinPlanarity " << Get_MinPlanarity() << endl;
  cout << "PlaneRatio " << Get_PlaneRatio() << endl;
  cout << "PointDist " << Get_PointDist() << endl;
  cout << "PeakWindow " << Get_PeakWindow() << endl;
  cout << "WindowSize " << Get_WindowSize() << endl;
  cout << "TrashMax " << Get_TrashMax() << endl;
  cout << "AccumulatorType " << Get_AccumulatorType() << endl;
  cout << "PlaneDir " << Get_PlaneDir() << endl;
}

unsigned int ConfigFileHough::LoadCfg(const char* CfgFile) {

  FILE *Cfg;

  /* Opens the configuration file */
  if ((Cfg = fopen(CfgFile, "r")) == NULL) {
    printf("Could not open configuration file '%s'\n", CfgFile);
    return 0;
  }
 
  
  MaxDist           = paramtr_scan_double(Cfg, "MaxDist", DEF_MaxDist);
  MinDist           = paramtr_scan_double(Cfg, "MinDist", DEF_MinDist);
  AccumulatorMax    = paramtr_scan_int(Cfg, "AccumulatorMax", DEF_AccumulatorMax);
  MinSizeAllPoints  = paramtr_scan_int(Cfg, "MinSizeAllPoints", DEF_MinSizeAllPoints);
  RhoNum            = paramtr_scan_int(Cfg, "RhoNum", DEF_RhoNum);
  ThetaNum          = paramtr_scan_int(Cfg, "ThetaNum", DEF_ThetaNum);
  PhiNum            = paramtr_scan_int(Cfg, "PhiNum", DEF_PhiNum);
  RhoMax            = paramtr_scan_int(Cfg, "RhoMax", DEF_RhoMax);
  MaxPlanes         = paramtr_scan_int(Cfg, "MaxPlanes", DEF_MaxPlanes);
  MinPlaneSize      = paramtr_scan_int(Cfg, "MinPlaneSize", DEF_MinPlaneSize);
  MinPlanarity      = paramtr_scan_double(Cfg, "MinPlanarity", DEF_MinPlanarity);
  PlaneRatio        = paramtr_scan_double(Cfg, "PlaneRatio", DEF_PlaneRatio);
  MaxPointPlaneDist = paramtr_scan_double(Cfg, "MaxPointPlaneDist", DEF_MaxPointPlaneDist);
  PeakWindow        = paramtr_scan_int(Cfg, "PeakWindow", DEF_PeakWindow);
  WindowSize        = paramtr_scan_int(Cfg, "WindowSize", DEF_WindowSize);
  TrashMax          = paramtr_scan_int(Cfg, "TrashMax", DEF_TrashMax);
  PointDist         = paramtr_scan_double(Cfg, "PointDist", DEF_PointDist);
  
  AccumulatorType   = paramtr_scan_int(Cfg, "AccumulatorType", DEF_AccumulatorType);
  paramtr_scan_str(Cfg, "PlaneDir", PlaneDir);

  fclose(Cfg);
  return 1;

}
