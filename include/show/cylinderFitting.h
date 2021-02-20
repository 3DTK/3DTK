#ifndef __SHOW_CYLINDERFITTING_H__
#define __SHOW_CYLINDERFITTING_H__

#include "show/show_common.h"

#include "newmat/newmatio.h"

#include <stdint.h>

struct CylPoint {
     float x, y, z;

     CylPoint() {}
     CylPoint(float x, float y, float z) : x(x), y(y), z(z) {}

     bool operator<(const CylPoint &o) const {
         if (x != o.x)
             return x < o.x;
         if (y != o.y)
             return y < o.y;
         return z < o.z;
     }
     bool operator>(const CylPoint &o) const {
         if (x != o.x)
             return x > o.x;
         if (y != o.y)
             return y > o.y;
         return z > o.z;
     }
     bool operator==(const CylPoint &o) const {
         if (x != o.x)
             return 0;
         if (y != o.y)
             return 0;
         if (z != o.z)
           return 0;
         return 1;
     }
     bool operator!=(const CylPoint &o) const {
       if (x != o.x)
           return 1;
       if (y != o.y)
           return 1;
       if (z != o.z)
         return 1;
       return 0;
     }
};

struct FittingParameters{
  NEWMAT::Real average[3];
  NEWMAT::ColumnVector* mu;
  NEWMAT::Matrix* F0;
  NEWMAT::Matrix* F1;
  NEWMAT::Matrix* F2;
};

struct Cylinder{
  //Deconstructor + Constructor
  Cylinder(){
    //Init cyl paras Used for Fitting
    para.mu = new NEWMAT::ColumnVector(6);  *(para.mu) = 0.0;
    para.F0 = new NEWMAT::Matrix(3,3);      *(para.F0) = 0.0;
    para.F1 = new NEWMAT::Matrix(3,6);      *(para.F1) = 0.0;
    para.F2 = new NEWMAT::Matrix(6,6);      *(para.F2) = 0.0;
  }

  ~Cylinder() {
    if(!selectedPoints.empty()){
      for (std::vector<CylPoint*>::iterator it = selectedPoints.begin();it != selectedPoints.end(); it++) {
        delete (*it);
      }
      //std::cout << "Selected cylinder points, ";
    }
    if(!cylinderPoints.empty()){
      for (std::vector<CylPoint*>::iterator it = cylinderPoints.begin();it != cylinderPoints.end(); it++) {
        delete (*it);
      }
      //std::cout << "cylinder points, ";
    }

    delete para.mu;
    delete para.F0;
    delete para.F1;
    delete para.F2;
    //std::cout << "cylinder paras deleted" << '\n';
  }

  //Cylinder Parameter
  uint16_t cylIndex; // also used for GUI

  std::vector<CylPoint*> selectedPoints; //Selected User Points
  std::vector<CylPoint*> cylinderPoints; //Cylinder Points (Selected + Growing)

  float radius; //Radius
  float axis[3];  //Cylinder axis vector
  float axisUser[3]; //not normed user axis

  float cylStartP[3]; //Cylinder axis start point
  float cylEndP[3]; // Cylinder axis end point

  float pAxis[3]; //Point on cylinder axis

  //Parameters used for cyl Fitting
  FittingParameters para;
};


//Igel Real: 1 true 1 250 0.1 1 0.5
struct GUICylinderParas{
  //GUI Scale
  unsigned int scaleGUI = 1; //Scale GUI values if to big

  //Cylinder Fitting LS
  float axisResFirst = 1.0;   //Resolution[°] of Phi/Theta for first LS guess
  bool randomized = true;
  float angleMaxCone = 1.5;   //Max Angle[°] of cone with axis as middle
  unsigned int nAxis = 250;
  float axisResCone = 0.1;    //Resolution[°] of Phi/Theta for cone LS

  //Cylinder Fitting Growing
  float cylEps = 1;
  float radiusThreshold = 0.5;
};


class CylinderFitting{
public:
  std::set<Cylinder *> generated_Cylinders;
  GUICylinderParas cylParasGUI;

  CylinderFitting();
  ~CylinderFitting();

  //User Interface Functions (QT GUI)
  void fitCylinderFromUserSelection(int cylinderIndex);
  bool deleteCylinder(int cylinderIndex);
  void deleteAllCylinder();
  bool setCurrentCylinder(int cylinderIndex);
  void changeCylinderParas(double radius, double length, double* axis, double* cylStartP, double* cylEndP);
  void saveCylinderToFile();
  void changeCylinderParas();
  void loadCylindersFromFile();
private:
  //Function use for creating, deleting, updating
  struct Cylinder* createCylinderFromSelectedPoints(int cylIndex);
  bool deleteCylinderFromSet(int cylIndex);
  struct Cylinder* getCylinder(int cylinderIndex);
  void setDefaultCurrentCylParas(int cylinderIndex);
  void setCurrentCylParas(Cylinder* cyl);
  bool newGUIValue(double valueGUI, double cylinderValue);

  //Function used for fitting cylinder to segmented points
  void fitCylinderToPoints(Cylinder *c);
  bool fitCylinderAlongLateralSurfacePoints(Cylinder* c);
  bool prepareFittingValues(Cylinder *c);
  std::vector<CylPoint> prepareCylAxisForLS(Cylinder *c, float resDeg, bool firstGuess = true, float coneAngleDeg = 0.0);
  void fitCylinderLeastSquare(Cylinder *c, std::vector<CylPoint> axis);
  std::vector<float*> cylinderAxisSearch(double middlePoint[3], double dir[3], float length, float radiusMax, float radiusMin);
  void calcCylinderEndsMinMax(Cylinder *c);
};
float dist2Points3D(float p1[3], float p2[3]);
void crossProduct(NEWMAT::Real* a, NEWMAT::Real* b, NEWMAT::Real* c);
extern CylinderFitting cylFit;
#endif
