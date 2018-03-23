#ifndef __CURVES_H__
#define __CURVES_H__

#include <string>
#include <vector>
#include <math.h>
#include<map>
//#include <boost/thread/mutex.hpp>
//#include <boost/thread/locks.hpp>
#include <iostream>
//class Curves;
#include <Eigen/Dense>
#include <bits/stringfwd.h>
//#include <unsupported/Eigen/MatrixFunctions>
using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;

class Curves;
typedef std::vector<Curves*> curvesVector;
//typedef std::map<int,curvesVector> Fus_Vector;
class Curves {

public:

  Curves() {};
 // virtual ~Curves();

  //! Holder of all scans
  //  also used in transform for adding frames for each scan at the same time
  static std::vector<Curves*> allpoints;
  static std::vector<Curves*> Samplepoints;
  static std::vector<Curves*> Oripoints;
  static std::vector<Curves*> geodesicPoint;
  //typedef std::shared_ptr<Curves> Ptr;
  Curves(const std::string& path ,int type,int identifier);
  void readTrajectory(const char* dir_path, double* pose,int identifiers);
  void Trans_Mat(Curves* CurrentPoint,Curves* NextPoint);
  //void Trans_Mat2(Curves* CurrentPoint,Curves* NextPoint);
  MatrixXd Opt_rot(VectorXd A,VectorXd B);
  //int parseArgs(int argc,char **argv, char dir[255], int& num_points, int& sample_points,int& type)
  //void inv_rep(curvesVector &Samplepoints);
  //void plot_curves( Fus_Vector &Curvesnum);
public:
    std::string m_path;
    int  m_type;
    int  identifiers;
    std::vector<double> cached_poses;
    Matrix2d rot1;
    Matrix2d rot2; 
    Vector2d trans1;
    Vector2d trans2;
    Matrix2cd R;
    Vector2d trans;
  //  std::vector<double> cached_poses;
  /**
   * The pose of the scan
   * Note: rPos/rPosTheta and transMat _should_
   *       always represent the same pose!!!
   */
  Vector3d points1; 
  Vector3d points2;   //!< 3D position of trajectory1 and trajectory2
  Vector3d Fus_points;
 // bool type;    //type of curve ( 0 - for open, 1- for closed)
  //double start_point[3];
  Matrix3d transformation1;
  Matrix3d transformation2;
  Matrix3d Fus_transfor;
};

#endif
