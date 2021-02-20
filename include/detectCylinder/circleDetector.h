#ifndef __DETECTCYLINDER_CIRCLEDETECTOR_H__
#define __DETECTCYLINDER_CIRCLEDETECTOR_H__

#include <vector>
#include <math.h>
#include <iostream>
#include <stdlib.h>
#include <time.h>
#include "newmat/newmatap.h"                // for matrix applications


struct CircleDetectorPara{
  unsigned int minNumberPointsCircle = 100; //Min points on projected cyl
  float minCost = 5;                        //Min circle cost (bigger cost --> circle more likely)
  float thresDisCircle2Point = 0.1;         //Max distance between circle point and circle edge

  //RHT Paras
  unsigned int valThres = 2;                //Validation threshold --> Validate circle if it was randomly detected valThres time
  float thresRHT_radius = 1;                //Max radius diff (is radius same as akk entry radius)
  float thresRHTXY_m = 1;                   //Max distance between x/y circle middle value (is position same as akk entry)
  float roundValue = 1;                     //RHT round values for akk entries


  //Least-Square Paras
  float lengthOfArc = 0.15;                 //Length of an arc of a circle sector
  float p_nSegment = 0.3;                   //Minum percent of all sectors have to be positiv for valid circle
};

struct Point2D{
  float x;
  float y;
  int index3D;
};

struct origPoint3D{
  // Point coordinates
  double point[3];
  // Index in set of all Cylinderdetection Inputpoints
  unsigned int indexOrig;
  int indexVPossibleCircle = -1;
};

struct ProjPoint{
  //Projected Coordinates
  float x;
  float y;

  //Point Index used for checking if Point was already used in Ransac
  int index2D;

  //Point Index from original Input Points
  int index3D;
};

//Struct used for rht, calculates possible circle candidate
struct CircleCandidate{
  Point2D middle;             //Circle middle point
  float r;                    //Radius
  unsigned int nPoints;       //#Points on circle
  unsigned int inlierPoints; //#Points in circle
  unsigned int nCircleFound;  //How often was circle randomly calculated
  float cost;                 //Circle cost
  bool wasValidated;          //Circle entry was validated
};

struct Circle3D{
  //Circle Parameters
  float r;                             //Radius
  float middleP[3];                    //Circle Middle point
  std::vector<origPoint3D> v_origPoints; //Circle Points
  unsigned int inlierPoints;          //#Inlier

  //Circle Variables for Ransac
  float cost;
  int nVotingPoints;
};

class CircleDetector{
public:
  CircleDetector();
  ~CircleDetector();

  //Ransac Paramter/Functions
  CircleDetectorPara para;
  std::vector<Circle3D> detectCylinderCircles(const std::vector<origPoint3D>& v_inputPoints, float orthoSysMat[3][3]);
private:


  //Input Points (will be init with const std::vector<origPoint3D>& (detectCylinderCircles))
  std::vector<origPoint3D> v_origPoints;
  //Detected circles
  std::vector<Circle3D> v_validatedCircles;
  //Project points along cylinder axis
  std::vector<ProjPoint> v_projectedPoints;

  void projectPointsAlongCylAxis(float orthoSysMat[3][3]);
  void rhtNew();
  void leastSquareCircle(std::vector<ProjPoint> circlePoints, float meanX, float meanY, float *middlePoint, float * radius);
  void backProjection(float p_projected[2], float orthoSysMat[3][3], float back[3]);

  void calcCircleCost(CircleCandidate *c, float threshold, bool *pointsUsed, unsigned int &nc);
  bool validateAndCorrectCircleNew(CircleCandidate& c, double thresholdR = 2.5);

};

//Help functions
bool testCollinear2D(ProjPoint p1, ProjPoint p2, ProjPoint p3);
CircleCandidate calculateCircleThrough3Points(ProjPoint p1, ProjPoint p2, ProjPoint p3);

#endif
