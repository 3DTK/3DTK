#include "detectCylinder/configFileCylinderDetector.h"

ConfigFileCylinderDetector::ConfigFileCylinderDetector(){
  paras.cfgFileName = CFG_FILENAME;
  paras.maxCylAxisN = MAX_CYL_AXIS_NUM;
  paras.minCylPLateralSurface = MIN_CYL_POINT_LATERAL;

  paras.minDiffAngleAxisDeg = MIN_DIFFANGLE_AXIS_DEG;
  paras.nPhi = PHI_NUM;
  paras.nTheta = THETA_NUM;
  paras.windowSize = WINDOWSIZE;

  paras.maxDisP2GreatCircle = MAX_DIS_POINT2GREATCIRCLE;
  paras.minPointsCircle = MIN_POINTS_CIRCLE;
  paras.minCostCircle = MIN_COST_CIRCLE;
  paras.maxDisPoint2Circle = MAX_DIS_PINT2CIRCLE;
  paras.validationThres = VALIDATION_THRES;
  paras.rhtThresRadius = RHT_THRES_RADIUS;
  paras.rhtThresPosMiddle = RHT_THRES_XY_M;
  paras.rhtRoundValue = RHT_ROUND_VALUE;
  paras.lsLengthOfArc = LS_LENGTHOFARC;
  paras.lsPositivSectors = LS_P_SECTOR;
}

void ConfigFileCylinderDetector::printConfigParas(){
  std::cout << "Config File Name: " << paras.cfgFileName << '\n';
  std::cout << "# Max Cyl Axis: " << paras.maxCylAxisN << '\n';
  std::cout << "# Min points lateral surface: " << paras.minCylPLateralSurface << '\n';
  std::cout << "Min diff angle between axis: " << paras.minDiffAngleAxisDeg << '\n';
  std::cout << "# Phi (Akk cyl axis): " << paras.nPhi << '\n';
  std::cout << "# Theta (Akk cyl axis): " << paras.nTheta << '\n';
  std::cout << "Window size (Akk cyl axis): "<< paras.windowSize << '\n';
  std::cout << "Max distance between point + great circle: "<< paras.maxDisP2GreatCircle << '\n';
  std::cout << "# Min points circle (Pos. + radius): " << paras.minPointsCircle << '\n';
  std::cout << "# Min cost circle (Pos. + radius): "<< paras.minCostCircle << '\n';
  std::cout << "Max distance point and circle (Pos. + radius):" << paras.maxDisPoint2Circle << '\n';
  std::cout << "Validation threshold (Pos + radius): " << paras.validationThres << '\n';
  std::cout << "RHT Threshold radius (Pos + radius): " << paras.rhtThresRadius<< '\n';
  std::cout << "RHT Threshold circle middle (Pos + radius): " << paras.rhtThresPosMiddle << '\n';
  std::cout << "RHT Round value for akk entry (Pos + radius): " << paras.rhtRoundValue << '\n';
  std::cout << "LS Length of arc (Pos + radius): " << paras.lsLengthOfArc << '\n';
  std::cout << "LS Min. positive circle sectors (Pos + radius): " << paras.lsPositivSectors << '\n';
}

void ConfigFileCylinderDetector::loadCFG(std::string cfgFileName) {
  // input file stream
  std::ifstream cfgFile;

  // open the output file
  cfgFile.open(cfgFileName, std::ios::in);

  // if file not found then show error
  if(!cfgFile){
    std::cerr << "Error reading the cfg file: "<< cfgFileName << std::endl;
    return;
  }else{
    std::cout << "Read Parameters from File: "<< cfgFileName << '\n';
  }

  std::string line;
  while(std::getline(cfgFile, line, '\n')){
    std::stringstream ss1(line); std::string cfgPara;
    std::string paraId = "NULL";
    std::string para;
    while(std::getline(ss1, cfgPara, ' ')){
      if(cfgPara.at(0) == '#'){
        paraId = cfgPara;
      }else{
        para = cfgPara;
        setCfgParaFile(paraId, para);
      }
    }
  }
}

void ConfigFileCylinderDetector::setCfgParaFile(std::string paraId, std::string para){
  //Test if paraId init
  if(paraId.compare("NULL") == 0) return;

  std::cout << "paraId: "<< paraId << " value: "<< para << '\n';
  //get right para and set it
  if(paraId.compare("#MAX_CYL_AXIS_NUM") == 0) paras.maxCylAxisN = stoi(para);
  if(paraId.compare("#MIN_CYL_POINT_LATERAL") == 0) paras.minCylPLateralSurface = stoi(para);

  if(paraId.compare("#MIN_DIFFANGLE_AXIS_DEG") == 0) paras.minDiffAngleAxisDeg = stof(para);
  if(paraId.compare("#PHI_NUM") == 0) paras.nPhi = stof(para);
  if(paraId.compare("#THETA_NUM") == 0) paras.nTheta = stof(para);
  if(paraId.compare("#WINDOWSIZE") == 0) paras.windowSize = stoi(para);

  if(paraId.compare("#MAX_DIS_POINT2GREATCIRCLE") == 0) paras.maxDisP2GreatCircle = stof(para);
  if(paraId.compare("#MIN_POINTS_CIRCLE") == 0) paras.minPointsCircle = stoi(para);
  if(paraId.compare("#MIN_COST_CIRCLE") == 0) paras.minCostCircle = stof(para);
  if(paraId.compare("#MAX_DIS_PINT2CIRCLE") == 0) paras.maxDisPoint2Circle = stof(para);
  if(paraId.compare("#VALIDATION_THRES") == 0) paras.validationThres = stoi(para);
  if(paraId.compare("#RHT_THRES_RADIUS") == 0) paras.rhtThresRadius = stof(para);
  if(paraId.compare("#RHT_THRES_XY_M") == 0) paras.rhtThresPosMiddle = stof(para);
  if(paraId.compare("#RHT_ROUND_VALUE") == 0) paras.rhtRoundValue = stof(para);
  if(paraId.compare("#LS_LENGTHOFARC") == 0) paras.lsLengthOfArc = stof(para);
  if(paraId.compare("#LS_P_SECTOR") == 0) paras.lsPositivSectors = stof(para);
}
