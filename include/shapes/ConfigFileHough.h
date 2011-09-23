#ifndef _ConfigFileHough_
#define _ConfigFileHough_

#define DEF_CfgFileName         "bin/hough.cfg"
#define DEF_MaxDist             500
#define DEF_MinDist             50
#define DEF_AccumulatorMax      100
#define DEF_MinSizeAllPoints    20
#define DEF_RhoNum              500
#define DEF_ThetaNum            360
#define DEF_PhiNum              176
#define DEF_RhoMax              1500
#define DEF_MaxPointPlaneDist   1.5
#define DEF_MaxPlanes           20
#define DEF_MinPlaneSize        100
#define DEF_MinPlanarity        0.3
#define DEF_PlaneRatio          0.5
#define DEF_PointDist           5.0
#define DEF_PeakWindow          false
#define DEF_WindowSize          8
#define DEF_TrashMax            20

#define DEF_AccumulatorType     3
#define DEF_PlaneDir            "dat/planes/"
#define FNAME_LENGTH            100

struct maxcompare {
  bool operator()(int* ip1, int* ip2) const {
    return (ip1[0] >ip2[0]);
  }
};

class ConfigFileHough {

public:
  
  ConfigFileHough();

  unsigned int LoadCfg(const char *CfgFile);
  void ShowConfiguration();

  inline char* Get_CfgFileName();

  inline double Get_MaxDist();
  inline double Get_MinDist();             
  inline unsigned int Get_AccumulatorMax();   
  inline unsigned int Get_MinSizeAllPoints();    
  inline unsigned int Get_RhoNum();              
  inline unsigned int Get_ThetaNum();           
  inline unsigned int Get_PhiNum();         
  inline unsigned int Get_RhoMax();           
  inline double Get_MaxPointPlaneDist();   
  inline unsigned int Get_MaxPlanes();           
  inline unsigned int Get_MinPlaneSize(); 
  inline double Get_MinPlanarity();
  inline double Get_PlaneRatio();
  inline double Get_PointDist();           
  inline bool Get_PeakWindow(); 
  inline unsigned int Get_WindowSize();
  inline unsigned int Get_TrashMax();
                    
  inline unsigned int Get_AccumulatorType();     
  inline char* Get_PlaneDir();  

//private: // Parameters are public to avoid having to create mutator functions

  char CfgFileName[FNAME_LENGTH];
  double MaxDist;
  double MinDist;
  unsigned int AccumulatorMax;
  unsigned int MinSizeAllPoints;
  unsigned int RhoNum;
  unsigned int ThetaNum;
  unsigned int PhiNum;
  unsigned int RhoMax;
  double MaxPointPlaneDist;
  unsigned int MaxPlanes;
  unsigned int MinPlaneSize;
  double MinPlanarity;
  double PlaneRatio;
  double PointDist;
  bool PeakWindow;
  unsigned int WindowSize;
  unsigned int TrashMax;
  unsigned int AccumulatorType;
  char PlaneDir[FNAME_LENGTH];

};
#include "ConfigFileHough.icc"

#endif
