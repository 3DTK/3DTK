#ifndef __CURVES_H__
#define __CURVES_H__

#include <string>
#include <vector>
#include <math.h>
#include<map>
//#include "curvefusion/common.h"
//class Curves;

#include <Eigen/Core>
#include <Eigen/SVD>

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;

#define POINT3D
//#define PUBLISHTF
//#define UNIFORM
//#define CORRESPONDENCE
//#define SEPERATE_ROSCORE
//#define OPTIMAL_CORRESPODENCE
#define GPS_HECTOR
//#define READBAG 
//#define READFILE
#ifdef  POINT3D
#define DIMENSIONS   3
#else
#define DIMENSIONS   2
#endif 
//#define DIMENSIONS   2

class Curves;
typedef std::vector<Curves*> curvesVector;
//typedef std::map<int,curvesVector> Fus_Vector;
class Curves {

public:

  Curves() {};
 // virtual ~Curves();

  //! Holder of all scans
  //  also used in transform for adding frames for each scan at the same time

  //static std::vector<Curves*> Interpoints;
  static std::vector<Curves*> Oripoints;
  static std::vector<Curves*> allpoints;
  static std::vector<Curves*> newpoints;
  static std::vector<Curves*> Samplepoints;
  static std::vector<Curves*> gpspoints;
  static std::vector<Curves*> laserpoints;
  //static std::vector<Curves*> geodesicPoint;
  //typedef std::shared_ptr<Curves> Ptr;
  Curves(std::vector<double> *point ,int type,unsigned int identifier);
  Curves(Vector3d point3d ,int type, unsigned int identifier,string t);
 // Curves(std::vector<MatrixXd>&poses_result_final,Vector3d &t_i, int identifier);
  void readTrajectory(const char* dir_path, double* pose,unsigned int identifiers);
  void Trans_Mat(Curves* CurrentPoint,Curves* NextPoint);
  //void Trans_Mat2(Curves* CurrentPoint,Curves* NextPoint);
  MatrixXd Opt_rot(VectorXd A,VectorXd B);
 
public:
    std::vector<double> *m_point;
    int  m_type;
    string str;
    
    std::vector<double> cached_poses;
    //std::vector<MatrixXd>&poses_final;

   
    Vector3d points;
    Vector3d points1; 
    Vector3d points2;   //!< 3D position of trajectory1 and trajectory2
    Vector3d Fus_points;
    double   time_stamps;

    unsigned int  identifiers;
#ifdef POINT3D 
    Matrix3d rot1;
    Matrix3d rot2; 
    Vector3d trans1;
    Vector3d trans2;
   // Matrix3cd R;
    Vector3d trans;
    Vector4d Fuspoints;
    Vector4d cr_points1;
    Vector4d cr_points2;
 // bool type;    //type of curve ( 0 - for open, 1- for closed)
  //double start_point[3];
    Matrix4d transformation1;
    Matrix4d transformation2;
    Matrix4d Fus_transfor;

#else
    Matrix2d rot1;
    Matrix2d rot2; 
    Vector2d trans1;
    Vector2d trans2;
   // Matrix2cd R;
    Vector2d trans;
    Vector3d Fuspoints;
    Vector3d cr_points1;
    Vector3d cr_points2;
 // bool type;    //type of curve ( 0 - for open, 1- for closed)
  //double start_point[3];
    Matrix3d transformation1;
    Matrix3d transformation2;
    Matrix3d Fus_transfor;

#endif
};

#endif
