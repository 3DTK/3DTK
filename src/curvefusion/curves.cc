#include "curvefusion/curves.h"
#include <list>
#include <utility>
#include <fstream>
#include "curvefusion/extractTrajectory.h"

#include <iomanip>
using std::ios;

//std::vector<Curves*> Curves::Interpoints;
std::vector<Curves*> Curves::Oripoints;
std::vector<Curves*> Curves::allpoints;
std::vector<Curves*> Curves::newpoints;
std::vector<Curves*> Curves::Samplepoints;
std::vector<Curves*> Curves::gpspoints;
std::vector<Curves*> Curves::laserpoints;
Curves::Curves(std::vector<double> *point ,int type, unsigned int identifier) :
  m_point(point),m_type(type),identifiers(identifier)
{
  
 // double point[6];
 // readTrajectory(m_path.c_str(), point,identifiers);
  time_stamps = m_point[0][identifiers];
  points1(0) = m_point[4][identifiers];
  points1(1) = m_point[5][identifiers];
  points1(2) = m_point[6][identifiers];    //hector laser x y z
  points2(0) = m_point[1][identifiers];    //GPS x y z
  points2(1) = m_point[2][identifiers];
  points2(2) = m_point[3][identifiers]; 
}

Curves::Curves(Vector3d point3d ,int type, unsigned int identifier,string t) :
  points(point3d),m_type(type),identifiers(identifier),str(t)
{
  
 // double point[6];
 // readTrajectory(m_path.c_str(), point,identifiers);
  if(str=="gps")
   points1=points;

  else if(str=="laser")
   points2=points;
}


void Curves::readTrajectory(const char* dir_path, double* pose, unsigned int identifiers)

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
   
   MatrixXd traj_point1(2,DIMENSIONS);
   MatrixXd traj_mean1(2,DIMENSIONS);
   MatrixXd H1(2,DIMENSIONS);

  
   VectorXd colpoint1_1(DIMENSIONS),colpoint1_2(DIMENSIONS);
   VectorXd col_vec1_1(DIMENSIONS), col_vec1_2(DIMENSIONS);
   
  //trajectory 1


   for(int i=0;i<DIMENSIONS;i++){
   traj_point1(0,i)=(CurrentPoint->points1(i));
   col_vec1_1(i)=traj_point1(0,i);
   }
 
   for(int j=0;j<DIMENSIONS ;j++){
   traj_point1(1,j)=(NextPoint->points1(j));
   col_vec1_2(j)=traj_point1(1,j);
   }

  //caculate mean of currentpoint and nextpoint
   traj_mean1(0,0)=(traj_point1(0,0)+traj_point1(1,0))*0.5;
   traj_mean1(0,1)=(traj_point1(0,1)+traj_point1(1,1))*0.5;

#ifdef POINT3D
   traj_mean1(0,2)=(traj_point1(0,2)+traj_point1(1,2))*0.5;
#endif
 
   //traj_mean1(1,3)=(traj_point1(1,3)+traj_point1(2,3))/2;
   for(int i=0;i<DIMENSIONS;i++)
   traj_mean1(1,i)=traj_mean1(0,i);

  //subtract the mean of trajecotry points
   for(int j = 0; j < 2; j++){
    for(int k = 0; k < DIMENSIONS; k++){
     H1(j, k) =(traj_point1(j, k)-traj_mean1(j, k)) ;     
    }
   }
  
  
   JacobiSVD<MatrixXd>svd1(H1, ComputeFullU | ComputeFullV); 
   MatrixXd U1 = svd1.matrixU();

   MatrixXd V1 = svd1.matrixV();
 
  // colpoint1_2=(V1.transpose())*col_vec1_2;
   colpoint1_1=(V1.transpose())*col_vec1_1;
   colpoint1_2=(V1.transpose())*col_vec1_2;
   MatrixXd R1=Opt_rot(colpoint1_1,colpoint1_2);
  
  // IdentityMatrix E1(2);
   MatrixXd E1= MatrixXd::Identity(DIMENSIONS,DIMENSIONS);
   
   for(int i = 0; i < 2; i++){
    for(int j = 0; j < 2; j++){
     E1(i, j) =R1(i,j);     
    }
   }

   rot1=E1;
   rot1 = V1*rot1*(V1.transpose());
  
   
   trans1 = col_vec1_2 - rot1*col_vec1_1;


    for(int j = 0; j < DIMENSIONS; j++){
    for(int k = 0; k < DIMENSIONS; k++){
     transformation1(j, k) =rot1(j, k);     
    }
   }

   for(int j = 0; j < DIMENSIONS; j++){
   transformation1(j, DIMENSIONS)=trans1(j);
   }
   
   for(int k = 0; k < DIMENSIONS; k++)
   transformation1(DIMENSIONS, k)=0;
   
   transformation1(DIMENSIONS, DIMENSIONS)=1.0;

 
  //trajectory 2
   MatrixXd traj_point2(2,DIMENSIONS);
   MatrixXd traj_mean2(2,DIMENSIONS);
   MatrixXd H2(2,DIMENSIONS);
   VectorXd colpoint2_1(DIMENSIONS),colpoint2_2(DIMENSIONS);
   VectorXd col_vec2_1(DIMENSIONS), col_vec2_2(DIMENSIONS);
  

   for(int i=0;i<DIMENSIONS;i++){
   traj_point2(0,i)=(CurrentPoint->points2(i));
   col_vec2_1(i)=traj_point2(0,i);
   }
 
   for(int j=0;j<DIMENSIONS;j++){
   traj_point2(1,j)=(NextPoint->points2(j));
   col_vec2_2(j)=traj_point2(1,j);
   }

  //caculate mean of currentpoint and nextpoint
   traj_mean2(0,0)=(traj_point2(0,0)+traj_point2(1,0))*0.5;
   traj_mean2(0,1)=(traj_point2(0,1)+traj_point2(1,1))*0.5;

#ifdef POINT3D
   traj_mean2(0,2)=(traj_point2(0,2)+traj_point2(1,2))*0.5;
#endif
   
   for(int i=0;i<DIMENSIONS;i++)
   traj_mean2(1,i)=traj_mean2(0,i);

  //subtract the mean of trajecotry points
   for(int j = 0; j < 2; j++){
    for(int k = 0; k < DIMENSIONS; k++){
     H2(j, k) =(traj_point2(j, k)-traj_mean2(j, k)) ;     
    }
   }
  
   // Make SVD
   JacobiSVD<MatrixXd> svd2(H2, ComputeFullU | ComputeFullV);
   Matrix2d U2 = svd2.matrixU(); 

 
   MatrixXd V2 = svd2.matrixV();  
   
  // Matrix2d V2 = svd2.matrixV();  
   colpoint2_1=(V2.transpose())*col_vec2_1;
   colpoint2_2=(V2.transpose())*col_vec2_2;

   MatrixXd R2=Opt_rot(colpoint2_1,colpoint2_2);
   
   MatrixXd E2= MatrixXd::Identity(DIMENSIONS,DIMENSIONS);
   
   for(int i = 0; i < 2; i++){
    for(int j = 0; j < 2; j++){
     E2(i, j) =R2(i,j);     
    }
   }

   rot2=E2;
   rot2 = V2*rot2*(V2.transpose()); 

  // IdentityMatrix E1(2);
  // rot2 = V2*R2*(V2.transpose()); 
   trans2 = col_vec2_2 - rot2*col_vec2_1;
    for(int j = 0; j < DIMENSIONS; j++){
    for(int k = 0; k < DIMENSIONS; k++){
     transformation2(j, k) =rot2(j, k);     
    }
   }

   for(int j = 0; j < DIMENSIONS; j++){
   transformation2(j, DIMENSIONS)=trans2(j);
   }

   for(int k = 0; k < DIMENSIONS; k++)
   transformation2(DIMENSIONS, k)=0;
   
   transformation2(DIMENSIONS, DIMENSIONS)=1.0;
   
   //return 0;
}


MatrixXd Curves::Opt_rot(VectorXd A,VectorXd B)

{
   Vector2d A1,B1;
   for(int k = 0; k < 2; k++){
     A1(k)=A(k);
     B1(k)=B(k);
   }

   MatrixXd R(2,2),rot(2,2);
   MatrixXd corr=A1*(B1.transpose());
   
   JacobiSVD<MatrixXd> svd(corr, ComputeThinU | ComputeThinV);
   MatrixXd U = svd.matrixU(); 
   MatrixXd V = svd.matrixV(); 
   
   R = V*(U.transpose()); 
   Matrix2d E= Matrix2d::Identity(); 
   E(1,1)= R.determinant();
   rot = V*E*(U.transpose());  
   return rot;
}







