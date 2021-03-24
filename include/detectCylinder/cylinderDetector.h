#ifndef __DETECTCYLINDER_CYLINDERDETECTOR_H__
#define __DETECTCYLINDER_CYLINDERDETECTOR_H__
#include <detectCylinder/cylinderAxisDetector.h>
#include <detectCylinder/circleDetector.h>
#include <detectCylinder/configFileCylinderDetector.h>

#include <slam6d/kd.h>
#include <slam6d/scan.h>
#include <slam6d/normals.h>

#define SORT_ROUND_VALUE 1000
#define F_2_PROJ_CYLPOINTS 25

struct Cylinder{
  int cylinderNumber; //Cylinder number for identification
  std::vector<origPoint3D>* cylinderPoints;
  unsigned int inlierPoints;

  float axis[3]; //Cylinder axis
  float pAxis[3]; //Point on cylinder axis
  float radius;

  float cylinderAxisStart[3]; //Start point of cylinder on axis
  float cylinderAxisEnd[3]; //End point of cylinder on axis
};

struct CylinderDetectorPara{
  unsigned int maxCylAxis;                  //Maximum number of detected cylinder axis maxima
  float sameAngleDeg;                       //Min angle between two unique cylinder axis in deg
  unsigned int nPhi;                        //N Phi used for creating cylinder axis accumulator
  unsigned int nTheta;                      //N Theta used for creating cylinder axis accumulator
  unsigned int windowSize;                  //Window Size for moving window

  unsigned int minPLateralSurface;          //Min points on cylinder Lateral Surface
  float th_maxDisSPoint2Plane;              //Maximum distance between spherical normal point and plane with cylinder axis as normal

  unsigned int minNumberPointsCircle;       //Min points on projected cyl
  unsigned int minCostCircle;               //Min circle cost (bigger cost --> circle more likely)
  float thresDisCircle2Point;               //Max distance between circle point and circle edge

  //RHT Paras
  unsigned int valThres;                    //Validation threshold --> Validate circle if it was randomly detected valThres time
  float thresRHT_radius;                    //Max radius diff (is radius same as akk entry radius)
  float thresRHTXY_m;                       //Max distance between x/y circle middle value (is position same as akk entry)
  float roundValue;                         //RHT round values for akk entries

  //Least-Square Paras
  float lengthOfArc;                        //Length of an arc of a circle sector
  float p_nSegment;                         //Minum percent of all sectors have to be positiv for valid circle
};

class CylinderDetector{
public:
  //CylinderDetector(unsigned int maxCylAxis = 120, float sameAngleDeg = 1, unsigned int nPhi = 1000, unsigned int nTheta = 1000, unsigned int windowSize = 20, unsigned int minPLateralSurface = 50, float th_maxDisSPoint2Plane = 0.01);
  CylinderDetector();
  ~CylinderDetector();

  //Input function to add 1 scan (multiple scans can be added)
  void addScanPoints(DataXYZ* points, DataNormal* normals);
  //Detect cylinders in Point Cloud with RHT
  std::vector<Cylinder*> detectCylinder();

private:
  //Parameters for RHT
  CylinderDetectorPara para;
  CylinderAxisDetector* cad;
  CircleDetector* cd;
  bool initCAD_CD;

  // All Points + normals + spherical Normals
  std::vector<InputPoint> v_inputPoints;
  //All found cylinders
  std::vector<Cylinder*> v_cylinder;

  //Functions
  void calcOrthnormalCoorSys(CylinderAxis* ca, float orthoSysMat[3][3]);
  std::vector<origPoint3D> getPossibleCylinderPoints(CylinderAxis ca);
  bool findCylinderEndsNew(Cylinder *c);
  void deleteCylinderIntersectingAxes(bool deleteBadCylinder = true);
  bool sameCylinder(Cylinder& c1, Cylinder& c2);
  bool intersectCylinder(Cylinder &cyl1, Cylinder& cyl2);

};

void crossProduct(NEWMAT::Real* a, NEWMAT::Real* b, NEWMAT::Real* c);
double dist2Point(double* p1, double* p2);

#endif
