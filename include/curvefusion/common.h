#ifndef _COMMON_H_
#define _COMMON_H_

#include <unsupported/Eigen/MatrixFunctions>
#include <limits>
#include <iostream>
#include <Eigen/Dense>
#include "curvefusion/curves.h"
#include "getopt.h"
#include "slam6d/globals.icc"
#include "curvefusion/extractTrajectory.h"
#include "curvefusion/tf_broadcaster.h"
#include <boost/foreach.hpp>

#include "slam6d/globals.icc"
#include "slam6d/icp6Dsvd.h"
#include "curvefusion/common.h"
#include <Eigen/Eigenvalues>


extern string pose_dir;
extern string point_dir;
extern string point_dir_final;
extern string point_dir_final;
extern std::vector<Vector3d> V_point1;
extern std::vector<Vector3d> V_point2;
extern unsigned int seq_interval[60000];

void curvspace(unsigned int s_num,curvesVector &points);
void readkitti(std::vector<double> *hector_matrix,std::vector<double> *point_corr,std::string dir);
void curve_rep(curvesVector &points,int type);
void general_align(curvesVector &points,int type);
double Aligndata(const vector<PtPair>& pairs,
                        Matrix3d &R_icp,
                        Vector3d &translation_icp,
                        const double centroid_m[3],
                        const double centroid_d[3]);


void interpintv(Vector3d pt1,Vector3d pt2,double intv,Vector3d &newpt);
void inv_rep(curvesVector &Samplepoints,int fuse_flag);
void QuatToMatrix3(const double *quat,
                                 MatrixXd &mat);
void Matrix3ToQuat( MatrixXd &mat,
                                 double *quat);
double deg(const double rad);
void Matrix4ToEuler( double *alignxf,
                                  double *rPosTheta,
                                  double *rPos);

void save_fusion_pose(vector<MatrixXd> poses_result_fusion,string dir,int count);
void ros2frames(vector<MatrixXd> poses_result_fusion,string dir,int count);
void matrix_tf_final(vector<MatrixXd> poses_result_final,vector<VectorXd> point_result_final,int count,int steps,std::vector<double> *h_matrix,vector<MatrixXd> poses_result_final_h, string dir);
void save_final_result(vector<MatrixXd> poses_result_final,vector<VectorXd> point_result_final,string &pose_dir_final,string &point_dir_final,int count,int steps);
void interpolation(curvesVector &Samplepoints,string dir,string &pose_dir_final,string &point_dir_final,int count,int steps,int pose_index,unsigned int *h_seq);

void savedata(curvesVector &Samplepoints,string dir,string &pose_dir,string &point_dir,int count,int steps);

void geodesic_path(curvesVector &Samplepoints,int steps,string dir,int pose_index,std::vector<double> *h_matrix,unsigned int *h_seq);

vector<int32_t> computeRoi1 (vector<MatrixXd> &poses_result);

vector<int32_t> computeRoi2 (vector<VectorXd> &point_result);

void savePathPlot (vector<MatrixXd> &poses_result,vector<VectorXd> &point_result,string file1_name,string file2_name);

void plotPathPlot(string dir,vector<int32_t> &roi,int idx,int steps);

vector<MatrixXd> loadPoses(string file_name);

vector<VectorXd> loadPoses1(string file_name);
void save_correspodence(std::vector<double> *point_s,string dir);

void timestamp_correspondence(std::vector<double> *point_g,std::vector<double> *point_curve2,std::vector<double> *point_corr,unsigned int *h_seq,string dir);
void closest_correspondence(std::vector<double> *point_g,std::vector<double> *point_curve2,std::vector<double> *point_corr,unsigned int *h_seq,string dir);
void Optimal_correspondence(std::vector<double> *point_curve1,std::vector<double> *point_curve2,std::vector<double> *point_corr,unsigned int *h_seq,string dir,int type,double beta,int window_size);

void dp_optimal_point_sampling_Single(std::vector<double> *point_curve1,std::vector<double> *point_curve2,unsigned int *opt_index,double alpha,double beta,int window_size,int type);
void dp_optimal_point_sampling_Both(std::vector<double> *point_g,std::vector<double> *hector_matrix,int *opt_index,double alpha,double beta,int window_size);
void Curve_area(std::vector<double> *point_curve2,int sliding_rate,double *uni_rate,int type);

void Curve_length(std::vector<double> *point_curve2,int sliding_rate,double *uni_rate,int type);

void opt_subStructure(int j,std::vector<double> *point_g,std::vector<double> *hector_matrix,int pre,double *uni_rate,double alpha,double beta,int i,VectorXd &temp1,VectorXd &temp2,VectorXd &temp3,int uni);

double geo_dist_SE(MatrixXd mat1,MatrixXd mat2);

double matirx_norm2(MatrixXd rot);
#endif




