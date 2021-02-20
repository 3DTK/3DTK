#ifndef __DETECTCYLINDER_CONFIGFILECYLINDERDETECTOR_H__
#define __DETECTCYLINDER_CONFIGFILECYLINDERDETECTOR_H__

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#define CFG_FILENAME "include/detectCylinder/cylinderDetector.cfg"
#define MAX_CYL_AXIS_NUM            100
#define MIN_CYL_POINT_LATERAL       100

#define MIN_DIFFANGLE_AXIS_DEG      1
#define PHI_NUM                     1000
#define THETA_NUM                   1000
#define WINDOWSIZE                  20

#define MAX_DIS_POINT2GREATCIRCLE   0.01
#define MIN_POINTS_CIRCLE           100
#define MIN_COST_CIRCLE             5.0
#define MAX_DIS_PINT2CIRCLE         0.1

#define VALIDATION_THRES            2
#define RHT_THRES_RADIUS            1.0
#define RHT_THRES_XY_M              1.0
#define RHT_ROUND_VALUE             1.0

#define LS_LENGTHOFARC              0.15
#define LS_P_SECTOR                 0.3

struct ParasCylDetection{
  std::string cfgFileName;
  unsigned int maxCylAxisN;
  unsigned int minCylPLateralSurface;

  float minDiffAngleAxisDeg;
  unsigned int nPhi;
  unsigned int nTheta;
  unsigned int windowSize;

  float maxDisP2GreatCircle;
  float minPointsCircle;
  float maxDisPoint2Circle;
  unsigned int minCostCircle;
  unsigned int validationThres;
  float rhtThresRadius;
  float rhtThresPosMiddle;
  float rhtRoundValue;
  float lsLengthOfArc;
  float lsPositivSectors;
};
class ConfigFileCylinderDetector {
public:
  ConfigFileCylinderDetector();

  void loadCFG(std::string cfgFileName = CFG_FILENAME);
  void printConfigParas();
  void setCfgParaFile(std::string paraId, std::string para);


  ParasCylDetection paras;
private:
};

#endif
