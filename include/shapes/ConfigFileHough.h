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

  unsigned int LoadCfg(char *CfgFile);
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

private:

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
#include "shapes/ConfigFileHough.icc"

#endif
