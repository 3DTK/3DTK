#ifndef _ConfigFileHough_
#define _ConfigFileHough_

#define DEF_CfgFileName         "bin/hough.cfg"
#define DEF_MaxDist             200
#define DEF_MinDist             25
#define DEF_AccumulatorMax      100
#define DEF_MinSizeAllPoints    2000
#define DEF_RhoNum              100
#define DEF_ThetaNum            120
#define DEF_PhiNum              60
#define DEF_RhoMax              30
#define DEF_MaxPointPlaneDist   2000
#define DEF_MaxPlanes           20
#define DEF_MinPlaneSize        1
#define DEF_MinPlanarity        30.0
#define DEF_PlaneRatio          0.9
#define DEF_PointDist           30.0
#define DEF_PeakWindow          false
#define DEF_WindowSize          8
#define DEF_TrashMax            20

#define DEF_AccumulatorType     1
#define DEF_PlaneDir            "/tmp/planes/"
#define FNAME_LENGTH            100

struct maxcompare {
  bool operator()(int* ip1, int* ip2) const {
    return (ip1[0] >ip2[0]);
  }
};

class ConfigFileHough {

public:
  
  ConfigFileHough();

  int LoadCfg(char *CfgFile);
  int ShowConfiguration();

  inline char* Get_CfgFileName();

  inline double Get_MaxDist();
  inline double Get_MinDist();             
  inline int Get_AccumulatorMax();   
  inline int Get_MinSizeAllPoints();    
  inline int Get_RhoNum();              
  inline int Get_ThetaNum();           
  inline int Get_PhiNum();         
  inline int Get_RhoMax();           
  inline double Get_MaxPointPlaneDist();   
  inline int Get_MaxPlanes();           
  inline int Get_MinPlaneSize(); 
  inline double Get_MinPlanarity();
  inline double Get_PlaneRatio();
  inline double Get_PointDist();           
  inline bool Get_PeakWindow(); 
  inline int Get_WindowSize();
  inline int Get_TrashMax();
                    
  inline int Get_AccumulatorType();     
  inline char* Get_PlaneDir();  

private:

  char CfgFileName[FNAME_LENGTH];
  double MaxDist;
  double MinDist;
  int AccumulatorMax;
  int MinSizeAllPoints;
  int RhoNum;
  int ThetaNum;
  int PhiNum;
  int RhoMax;
  double MaxPointPlaneDist;
  int MaxPlanes;
  int MinPlaneSize;
  double MinPlanarity;
  double PlaneRatio;
  double PointDist;
  bool PeakWindow;
  int WindowSize;
  int TrashMax;
  int AccumulatorType;
  char PlaneDir[FNAME_LENGTH];

};
#include "slam6d/ConfigFileHough.icc"

#endif
