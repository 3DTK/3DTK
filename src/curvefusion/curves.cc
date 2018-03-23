#include "curvefusion/curves.h"
#include <list>
#include <utility>
#include <fstream>

std::vector<Curves*> Curves::allpoints;
std::vector<Curves*> Curves::Samplepoints;
std::vector<Curves*> Curves::Oripoints;
Curves::Curves(const std::string& path,int type, int identifier) :
  m_path(path),m_type(type),identifiers(identifier)
{
  
  double point[6];
  readTrajectory(m_path.c_str(), point,identifiers);

  
  points1(0) = point[0];
  points1(1) = point[1];
  points1(2) = point[2];
  points2(0) = point[3];
  points2(1) = point[4];
  points2(2) = point[5]; 
}


void Curves::readTrajectory(const char* dir_path, double* pose, int identifiers)

{
  
   // open pose file once and read all poses
   // string dir_path=dir_path-"/";
   // string FileName = dir_path;
    char FileName[255];
    snprintf(FileName,255,"%s%.2d.txt",dir_path,1);
    ifstream pose_file(FileName);
    //ifstream pose_file(dir_path);
    vector<double> poses;
    double p[6], timestamp;

    while(pose_file.good()) {
      try {
        pose_file >> timestamp
          >> p[0] >> p[1] >> p[2] // trajectory1 x, y, z
          >> p[3] >> p[4] >> p[5]; // trajectory2 x, y, z
      } catch(std::ios_base::failure& e) {
        break;
      }
      
      // add in poses
      for(int i = 0; i < 6; ++i) poses.push_back(p[i]);
    }

    // after success, set the cache
    cached_poses.swap(poses);
  
  // get index from the identifier and pick the pose
 
  if(cached_poses.size()<identifiers*6 + 6)
    throw std::runtime_error(std::string("There is no pose entry for scan"));
  
  for(unsigned int i=0; i<6; ++i)
    pose[i] = cached_poses[identifiers*6+i];
   
}


void Curves::Trans_Mat(Curves* CurrentPoint,Curves* NextPoint)
{
  //computer transformation matrix from current point to next point.
   
   //double point_c[6],point_n[6];
   
   MatrixXd traj_point1(2,2);
   MatrixXd traj_mean1(2,2);
   MatrixXd H1(2,2);
   VectorXd colpoint1_1(2),colpoint1_2(2);
   VectorXd col_vec1_1(2), col_vec1_2(2);
  
  //trajectory 1
   for(int i=0;i<2;i++){
   traj_point1(0,i)=(CurrentPoint->points1(i));
   col_vec1_1(i)=traj_point1(0,i);
   }
 
   for(int j=0;j<2;j++){
   traj_point1(1,j)=(NextPoint->points1(j));
   col_vec1_2(j)=traj_point1(1,j);
   }

  //caculate mean of currentpoint and nextpoint
   traj_mean1(0,0)=(traj_point1(0,0)+traj_point1(1,0))/2;
   traj_mean1(0,1)=(traj_point1(0,1)+traj_point1(1,1))/2;
   //traj_mean1(1,3)=(traj_point1(1,3)+traj_point1(2,3))/2;
   for(int i=0;i<2;i++)
   traj_mean1(1,i)=traj_mean1(0,i);

  //subtract the mean of trajecotry points
   for(int j = 0; j < 2; j++){
    for(int k = 0; k < 2; k++){
     H1(j, k) =(traj_point1(j, k)-traj_mean1(j, k)) ;     
    }
   }
  
   // Make SVD
   JacobiSVD<MatrixXd> svd1(H1, ComputeThinU | ComputeThinV);
   Matrix2d U1 = svd1.matrixU(); 
   Matrix2d V1 = svd1.matrixV();  
   colpoint1_1=(V1.transpose())*col_vec1_1;
   colpoint1_2=(V1.transpose())*col_vec1_2;

   MatrixXd R1=Opt_rot(colpoint1_1,colpoint1_2);
   
  // IdentityMatrix E1(2);
   rot1 = V1*R1*(V1.transpose()); 
   trans1 = col_vec1_2 - rot1*col_vec1_1;
    for(int j = 0; j < 2; j++){
    for(int k = 0; k < 2; k++){
     transformation1(j, k) =rot1(j, k);     
    }
   }

   for(int j = 0; j < 2; j++){
   transformation1(j, 2)=trans1(j);
   }
   
   transformation1(2, 0)=0;
   transformation1(2, 1)=0;
   transformation1(2, 2)=1.0;


  //trajectory 2
   MatrixXd traj_point2(2,2);
   MatrixXd traj_mean2(2,2);
   MatrixXd H2(2,2);
   VectorXd colpoint2_1(2),colpoint2_2(2);
   VectorXd col_vec2_1(2), col_vec2_2(2);
  
  //trajectory 1
   for(int i=0;i<2;i++){
   traj_point2(0,i)=(CurrentPoint->points2(i));
   col_vec2_1(i)=traj_point2(0,i);
   }
 
   for(int j=0;j<2;j++){
   traj_point2(1,j)=(NextPoint->points2(j));
   col_vec2_2(j)=traj_point2(1,j);
   }

  //caculate mean of currentpoint and nextpoint
   traj_mean2(0,0)=(traj_point2(0,0)+traj_point2(1,0))/2;
   traj_mean2(0,1)=(traj_point2(0,1)+traj_point2(1,1))/2;
   //traj_mean1(1,3)=(traj_point1(1,3)+traj_point1(2,3))/2;
   for(int i=0;i<2;i++)
   traj_mean2(1,i)=traj_mean2(0,i);

  //subtract the mean of trajecotry points
   for(int j = 0; j < 2; j++){
    for(int k = 0; k < 2; k++){
     H2(j, k) =(traj_point2(j, k)-traj_mean2(j, k)) ;     
    }
   }
  
   // Make SVD
   JacobiSVD<MatrixXd> svd2(H2, ComputeThinU | ComputeThinV);
   Matrix2d U2 = svd2.matrixU(); 
   Matrix2d V2 = svd2.matrixV();  
   colpoint2_1=(V2.transpose())*col_vec2_1;
   colpoint2_2=(V2.transpose())*col_vec2_2;

   MatrixXd R2=Opt_rot(colpoint2_1,colpoint2_2);
   
  // IdentityMatrix E1(2);
   rot2 = V2*R2*(V2.transpose()); 
   trans2 = col_vec2_2 - rot2*col_vec2_1;
    for(int j = 0; j < 2; j++){
    for(int k = 0; k < 2; k++){
     transformation2(j, k) =rot2(j, k);     
    }
   }

   for(int j = 0; j < 2; j++){
   transformation2(j, 2)=trans2(j);
   }
   
   transformation2(2, 0)=0;
   transformation2(2, 1)=0;
   transformation2(2, 2)=1.0;
   
   //return 0;
}


MatrixXd Curves::Opt_rot(VectorXd A,VectorXd B)

{
   
   MatrixXd R(2,2),rot(2,2);
   Matrix2d corr=A*(B.transpose());
   
   JacobiSVD<MatrixXd> svd(corr, ComputeThinU | ComputeThinV);
   MatrixXd U = svd.matrixU(); 
   MatrixXd V = svd.matrixV(); 
   
   R = V*(U.transpose()); 
   Matrix2d E= Matrix2d::Identity(); 
   E(1,1)= R.determinant();
   rot = V*E*(U.transpose());  
   return rot;
}








