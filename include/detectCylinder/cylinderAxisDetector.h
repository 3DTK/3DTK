#ifndef __DETECTCYLINDER_CYLINDERAXISDETECTOR_H__
#define __DETECTCYLINDER_CYLINDERAXISDETECTOR_H__
#include <iostream>
#include <algorithm>
#include <vector>
#include <math.h>
#include "newmat/newmatap.h"                // need matrix applications



struct InputPoint{
  double p[3];  //Point
  double n[3];  //Surface Normal of Point
  float pN[3]; //Polar Normals on unit half sphere

  InputPoint(double* point, double* normal){
    p[0] = point[0];
    p[1] = point[1];
    p[2] = point[2];
    n[0] = normal[0];
    n[1] = normal[1];
    n[2] = normal[2];
    pN[0] = 0;
    pN[1] = 0;
    pN[2] = 0;
  }
};

struct CylinderAxisDetectorPara{
  float sameAngleDeg; // min Deg between 2 maxima Orientation in Hough

  //Accumulator Design
  unsigned int nTheta;
  unsigned int nPhi;

  //Find Maxima Function
  unsigned int windowSize;
};

struct CylinderAxis{
    float axis[3];
    unsigned int nVotes;
  };

class CylinderAxisDetector{
public:
  CylinderAxisDetector(std::vector<InputPoint>* v_inputPoints, float sameAngleDeg, unsigned int nPhi, unsigned int nTheta, unsigned int windowSize);
  ~CylinderAxisDetector();

  void calculateCylAxis();
  CylinderAxis getCylAxisHypothese(int index);
private:
  //Var:
  CylinderAxisDetectorPara para; //Paras used for RHT
  std::vector<InputPoint>* v_inputPoints; //Input Points
  unsigned int **accumulator; //Accumulator
  unsigned int *nThetaInPhiRow;  //Number of cells in accumulator row
  std::vector<CylinderAxis*> v_cylinderAxis;   // Cylinder Axis hypothese (sort by number of votings)

  //Functions
  void calcSphereNormals();
  void createAccumulator();
  void detectCylinderAxisRHT();
  void sortPossibleCylinderAxis();
};

//Help Functions
void cart2sph(float* cart, float* sphere);
float euclideanNorm3D(float* v3d);
float dist2Point(float* p1, float* p2);
bool calcOriginPlaneThrough2Points(float* p1, float* p2, double &phi, double &theta);
void crossProduct(float* v1, float* v2, float* v1cv2);
bool cartToPolarUnit(double n[3], double &phi, double &theta);
#endif
