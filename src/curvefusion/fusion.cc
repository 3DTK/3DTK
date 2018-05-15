
/* Program:
 * This program is written for fusing two similar trajectories from different sensors.
 * History:
 * 16/03/2018	
 * Copyright (C) Shitong Du
 * 
 *
 * By set UNIFORM,different sample ways are established.
 *  
 * POINT3D represents different dimension. (3D or 2D)
 *
 */

//#define num_points    566
//#define sample_points 283;

#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <Eigen/Dense>
#include "curvefusion/curves.h"
#include <limits> 
#include "getopt.h"
#include "slam6d/globals.icc"
#include "curvefusion/timestamps.h"
#include "curvefusion/tf_broadcaster.h"
//#define UNIFORM
//#define CORRESPONDENCE
#define SENDTF

using namespace std;
using namespace Eigen;

int start_flag=0;
using std::cout;
using std::cerr;
using std::endl;
using std::ifstream;
using std::ofstream;
using std::string;
using std::ios;
std::map<int,vector<Curves*> > Curvesnum;
static std::vector<Vector3d> V_point1;
static std::vector<Vector3d> V_point2;
static std::vector<double> V_time;
std::vector<Matrix2d> AbelianR;
 

string pose_dir;
string point_dir;

int parseArgs(int argc,char **argv, string &dir,  unsigned int &num_points,  unsigned int &sample_points,int &type,int & steps,int &pose_index){
  num_points   = 1;
  sample_points = 1; // -1 indicates no limitation
  type=1;
  steps=5;
  pose_index=1;
  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  std::cout << std::endl;
  while ((c = getopt (argc, argv, "n:s:t:p:i:")) != -1)
    switch (c)
   {
   case 'n':
     num_points = atoi(optarg);
     if (num_points <= 0) { std::cerr << "Error: Cannot start at a negative or zero point number.\n"; exit(1); }
     break;
   case 's':
     sample_points = atoi(optarg);
     if (sample_points <=0)     { std::cerr << "Error: Cannot sample with a negative or zero point number.\n"; exit(1); }
     if (sample_points > num_points) { std::cerr << "Error: <sample> cannot be larger than <num_points>.\n"; exit(1); }
     break;
   case 't':
     type = atoi(optarg);
     //if ((type!=0)||(type!=1))     { std::cerr << "Error: type should be 1 or 0.\n"; exit(1); }  
    // break;
     if(type>1)        { std::cerr << "Error: type should be 1 or 0.\n"; exit(1); } 
   case 'p':
      steps = atoi(optarg);
     if (steps==0)     { std::cerr << "Error: steps cannot be 0.\n"; exit(1); }  
     break;
   case 'i':
     pose_index = atoi(optarg);
     if ((pose_index>(steps+1))||(pose_index<0))     { std::cerr << "Error: pose_index cannot exceed pose_num and be less than 0.\n"; exit(1); }  
     break;
   }

  if (optind != argc-1) {
    std::cerr << "\n*** Directory missing ***\n" << std::endl; 
    std::cout << std::endl
              << "Usage: " << argv[0] << "  [-n NR] [-s NR] [-t NR] [-p NR] [-i NR] directory" << std::endl << std::endl;
    std::cout << "  -n NR   the number of points"  << std::endl  
              << "  -s NR   the number of sample points" << "" << std::endl
              << "  -t NR   the type of the curve. 0:open curve; 1: closed curve" << "" << std::endl
	      << std::endl;
    std::cout <<" -p NR the number of fusion steps,and steps+1 represents the number of all fusion trajectories"<< std::endl;
    std::cout <<" -i NR choose the optimal trajecotry index"<< std::endl;
    std::cout << "using Girum'method to fuse two trajectories.." << std::endl;
    abort();
  }
   dir = argv[optind];

#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif
  return 0;
}

void inv_rep(curvesVector &Samplepoints,int fuse_flag)
{
  
 // unsigned int l=Samplepoints.size();
  //Curves* s_point;
  VectorXd im(DIMENSIONS+1);
  
  for(int k = 0; k < DIMENSIONS; k++)
  im(k)=0;
  im(DIMENSIONS)=1.0; 
  

  Curves* s_point = Samplepoints[0];

  if(fuse_flag==1){
  s_point->Fuspoints=im;
  for(int j=0;j<3;j++)
  s_point->Fus_points(j)=s_point->Fuspoints(j);

  }
  else {
  s_point->cr_points1=im;
  s_point->cr_points2=im;
  }
  for(size_t i = 1; i < Samplepoints.size(); ++i){
   
  //这里matlab用的l是rep的个数，所以闭曲线和开曲线是有差别的，但是我用的是点的个数，所以可以跳过判断曲线开闭的步骤。
  Curves* c_point = Samplepoints[i];
  Curves* p_point = Samplepoints[i-1];
  
    if(fuse_flag==1){

    
     c_point->Fuspoints=(p_point->Fus_transfor)*(p_point->Fuspoints);
     for(int j=0;j<3;j++)
     c_point->Fus_points(j)=c_point->Fuspoints(j);
    }
   
    else {

     c_point->cr_points1=(p_point->transformation1)*(p_point->cr_points1);
     c_point->cr_points2=(p_point->transformation2)*(p_point->cr_points2);

     for(int j=0;j<3;j++){

     c_point->points1(j)=c_point->cr_points1(j);
     c_point->points2(j)=c_point->cr_points2(j);
     }

    }

  }

}

void savedata(curvesVector &Samplepoints,string dir,string &pose_dir,string &point_dir,int count,int steps)
{
   
  
   
     string PoseFileName = pose_dir +to_string(count,2)+ ".txt";
     string PointFileName = point_dir +to_string(count,2)+ ".txt";
     
     // open file  
     FILE *fp1 = fopen(PoseFileName.c_str(),"w");
     FILE *fp2 = fopen(PointFileName.c_str(),"w");

     if(count==0){

        //cout<<fuse_curve.size()<<endl;
       for(unsigned int i = 0; i < Samplepoints.size(); i++)
       {
     
         Curves* c_point = Samplepoints[i];

         MatrixXd posematrix(DIMENSIONS+1,DIMENSIONS+1);
         posematrix=(c_point->transformation1);
         VectorXd point=c_point->points1;
        
        // parameters
#ifdef POINT3D  
  

         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                posematrix(0,0),posematrix(0,1),posematrix(0,2),posematrix(0,3),
                                posematrix(1,0),posematrix(1,1),posematrix(1,2),posematrix(1,3),
                                posematrix(2,0),posematrix(2,1),posematrix(2,2),posematrix(2,3),
                                posematrix(3,0),posematrix(3,1),posematrix(3,2),posematrix(3,3));
         fprintf(fp2,"%lf %lf %lf\n",point(0),point(1),point(2));

#else
         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                posematrix(0,0),posematrix(0,1),posematrix(0,2),
                                posematrix(1,0),posematrix(1,1),posematrix(1,2),
                                posematrix(2,0),posematrix(2,1),posematrix(2,2));
         fprintf(fp2,"%lf %lf\n",point(0),point(1));
#endif


       }

      

      }

    else if(count==steps){
     
       
         for(unsigned int i = 0; i < Samplepoints.size(); i++)
       {
     
         Curves* c_point = Samplepoints[i];
         MatrixXd posematrix=c_point->transformation2;
         VectorXd point=c_point-> points2;
        //MatrixXd P=MatrixXd::Identity(3,3);
        // parameters
#ifdef POINT3D
         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                 posematrix(0,0),posematrix(0,1),posematrix(0,2),posematrix(0,3),
                                 posematrix(1,0),posematrix(1,1),posematrix(1,2),posematrix(1,3),
                                 posematrix(2,0),posematrix(2,1),posematrix(2,2),posematrix(2,3),
                                 posematrix(3,0),posematrix(3,1),posematrix(3,2),posematrix(3,3));
         fprintf(fp2,"%lf %lf %lf\n",point(0),point(1),point(2));
#else
        fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                 posematrix(0,0),posematrix(0,1),posematrix(0,2),
                                 posematrix(1,0),posematrix(1,1),posematrix(1,2),
                                 posematrix(2,0),posematrix(2,1),posematrix(2,2));
         fprintf(fp2,"%lf %lf\n",point(0),point(1));
#endif
       }
          

     } 

     else {
        for(unsigned int i = 0; i < Samplepoints.size(); i++)
        {
     
         Curves* c_point = Samplepoints[i];
         MatrixXd posematrix=c_point->Fus_transfor;
         VectorXd point=c_point->Fus_points;
         //MatrixXd P=MatrixXd::Identity(3,3);
         // parameters
#ifdef POINT3D
         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                posematrix(0,0),posematrix(0,1),posematrix(0,2),posematrix(0,3),
                                posematrix(1,0),posematrix(1,1),posematrix(1,2),posematrix(1,3),
                                posematrix(2,0),posematrix(2,1),posematrix(2,2),posematrix(2,3),
                                posematrix(3,0),posematrix(3,1),posematrix(3,2),posematrix(3,3));
                                fprintf(fp2,"%lf %lf %lf\n",point(0),point(1),point(2));
#else
        fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",
                                posematrix(0,0),posematrix(0,1),posematrix(0,2),
                                posematrix(1,0),posematrix(1,1),posematrix(1,2),
                                posematrix(2,0),posematrix(2,1),posematrix(2,2));
         fprintf(fp2,"%lf %lf\n",point(0),point(1));
#endif
        }
     }
    // close file
     fclose(fp1);
     fclose(fp2);
 // }
   

}


void geodesic_path(curvesVector &Samplepoints,int steps,string dir,int pose_index)
{
   double timestep=1/((double)steps);
   double step=0;

   //Matrix2d R_c;
   
   MatrixXcd Rc(DIMENSIONS, DIMENSIONS);
   VectorXd trans(DIMENSIONS);
   MatrixXd R1(DIMENSIONS, DIMENSIONS);
   MatrixXd R2(DIMENSIONS, DIMENSIONS);
   VectorXd tra1(DIMENSIONS);
   VectorXd tra2(DIMENSIONS); 
   MatrixXd Ri(DIMENSIONS, DIMENSIONS);
   
   MatrixXcd R5(DIMENSIONS, DIMENSIONS);
   MatrixXd R(DIMENSIONS, DIMENSIONS);
   pose_dir  = dir + "fuse_pose/";
   point_dir  = dir + "new_point/";

   // create output directories
   system(("mkdir " + pose_dir).c_str());
   system(("mkdir " + point_dir).c_str());
   //int i;

  
  for(unsigned int j = 0; j < Curves::Samplepoints.size(); j++){

   Curves *curves;
   curves=Curves::Samplepoints[j];
   Curves::Oripoints.push_back(curves);
  }

   for(int i=0;i<=steps;i++){

      if(i==0){
      savedata(Curves::Oripoints,dir,pose_dir,point_dir,i,steps);
      
      }
      else if(i==steps){
      savedata(Curves::Oripoints,dir,pose_dir,point_dir,i,steps);
     
      }
      else{
   
      step = step + timestep;
      for(curvesVector::iterator it = Curves::Samplepoints.begin();
      it != Curves::Samplepoints.end();
      ++it) {
       
       Curves *C_Point = *it;
       R1=C_Point->rot1;
       R2=C_Point->rot2;
       tra1=C_Point->trans1;
       tra2=C_Point->trans2;
       
       //这里matlab中用的R1的转置，但是论文中写的是逆。
      Ri=(R1.transpose())*R2;
   
      Rc.real()= Ri;
      
#ifdef POINT3D
      Rc.imag()<<0,0,0,0,0,0,0,0,0; 
        
      MatrixPower<Matrix3cd> Apow(Rc); 
#else
      Rc.imag()<<0,0,0,0; 
      MatrixPower<Matrix2cd> Apow(Rc);
#endif  
       
      R= (R1*Apow(step)).real();
     
      trans=tra1+(tra2-tra1)*step;  
      
      for(int j = 0; j < DIMENSIONS; j++){
       for(int k = 0; k < DIMENSIONS; k++){
        C_Point->Fus_transfor(j, k) =R(j, k);     
        }
       }

       for(int j = 0; j < DIMENSIONS; j++){
        C_Point->Fus_transfor(j, DIMENSIONS)=trans(j);
       }

       for(int k = 0; k < DIMENSIONS; k++)
        C_Point->Fus_transfor(DIMENSIONS, k)=0; 

        C_Point->Fus_transfor(DIMENSIONS, DIMENSIONS)=1.0;
              
     }
         
        inv_rep(Curves::Samplepoints,1);
        
        savedata(Samplepoints,dir,pose_dir,point_dir,i,steps);
   }   

//#ifdef SENDTF
       if(i==pose_index) {
         // tf_broadcaster broadcaster;
          
         // unsigned int s_num=Samplepoints.size();
         for(unsigned int k = 0; k < Samplepoints.size(); k++)
          {
     
            Curves* c_point = Samplepoints[k];
            MatrixXd T=c_point->Fus_transfor;
            time_s[k]=c_point->time_stamps;
            matrix_new.push_back(T);
            
          } 
            //broadcaster.readtf(posematrix);
       }    
//#endif      
  }
       
}

vector<int32_t> computeRoi1 (vector<MatrixXd> &poses_result){
  
  float x_min = numeric_limits<int32_t>::max();
  float x_max = numeric_limits<int32_t>::min();
  float y_min = numeric_limits<int32_t>::max();
  float y_max = numeric_limits<int32_t>::min();
 
  for(unsigned int i = 0; i < poses_result.size(); i++){

#ifdef POINT3D  
   Matrix4d P=poses_result[i];
   double x = P(0,3);
   double y = P(2,3);// the y represents z in 3D.
#else
   Matrix3d P=poses_result[i];
   double x = P(0,2);
   double y = P(1,2);
#endif
   if (x<x_min) x_min = x; if (x>x_max) x_max = x;
   if (y<y_min) y_min = y; if (y>y_max) y_max = y;
  }

  float dx = 1.1*(x_max-x_min);
  float dy = 1.1*(y_max-y_min);
  float mx = 0.5*(x_max+x_min);
  float my = 0.5*(y_max+y_min);
  float r  = 0.5*max(dx,dy);
  
  vector<int32_t> roi;
  roi.push_back((int32_t)(mx-r));
  roi.push_back((int32_t)(mx+r));
  roi.push_back((int32_t)(my-r));
  roi.push_back((int32_t)(my+r));
  return roi;
}

vector<int32_t> computeRoi2 (vector<VectorXd> &point_result){
  
  float x_min = numeric_limits<int32_t>::max();
  float x_max = numeric_limits<int32_t>::min();
  float y_min = numeric_limits<int32_t>::max();
  float y_max = numeric_limits<int32_t>::min();
  
  for(unsigned int i = 0; i < point_result.size(); i++){

#ifdef POINT3D  
   Vector3d P=point_result[i];
   double x = P(0);
   double y = P(2);
#else
   Vector2d P=point_result[i];
   double x = P(0);
   double y = P(1);
#endif
   if (x<x_min) x_min = x; if (x>x_max) x_max = x;
   if (y<y_min) y_min = y; if (y>y_max) y_max = y;
  }

  float dx = 1.1*(x_max-x_min);
  float dy = 1.1*(y_max-y_min);
  float mx = 0.5*(x_max+x_min);
  float my = 0.5*(y_max+y_min);
  float r  = 0.5*max(dx,dy);
  
  vector<int32_t> roi;
  roi.push_back((int32_t)(mx-r));
  roi.push_back((int32_t)(mx+r));
  roi.push_back((int32_t)(my-r));
  roi.push_back((int32_t)(my+r));
  return roi;
}



void savePathPlot (vector<MatrixXd> &poses_result,vector<VectorXd> &point_result,string file1_name,string file2_name){

  // parameters
  int32_t step_size = 3;

  // open file  
  FILE *fp1 = fopen(file1_name.c_str(),"w");
  // save x/y coordinates of all frames to file
  for (unsigned int i=0; i<poses_result.size(); i+=step_size)

#ifdef POINT3D
  fprintf(fp1,"%f %f %f\n",(poses_result[i])(0,3),(poses_result[i])(1,3),(poses_result[i])(2,3));
#else
  fprintf(fp1,"%f %f\n",(poses_result[i])(0,2),(poses_result[i])(1,2));
#endif
  fclose(fp1);

  FILE *fp2 = fopen(file2_name.c_str(),"w");
  for (unsigned int j=0; j<point_result.size(); j++)

#ifndef POINT3D
  fprintf(fp2,"%f %f\n",(point_result[j])(0),(point_result[j])(1));
  // close file
#else
  fprintf(fp2,"%f %f %f\n",(point_result[j])(0),(point_result[j])(1),(point_result[j])(2));
#endif  
  fclose(fp2);
}

void plotPathPlot(string dir,vector<int32_t> &roi,int idx,int steps)
{
        // gnuplot file name
  char command[1024];
  char file_name[256];
  sprintf(file_name,"%02d.gp",idx);
  string full_name = dir + "/" + file_name;
  
  // create png + eps
  for (int i=0; i<=steps; i++){

    // open file  
    FILE *fp = fopen(full_name.c_str(),"w");

    // save gnuplot instructions
    if (i==0) {
      fprintf(fp,"set term png size 900,900\n");
      fprintf(fp,"set output \"%02d.png\"\n",idx);
    } else {
      fprintf(fp,"set term postscript eps enhanced color\n");
      fprintf(fp,"set output \"%02d.eps\"\n",idx);
    }

    fprintf(fp,"set size ratio -1\n");
    fprintf(fp,"set xrange [%d:%d]\n",roi[0],roi[1]);
    fprintf(fp,"set yrange [%d:%d]\n",roi[2],roi[3]);
    fprintf(fp,"set xlabel \"x [m]\"\n");
    fprintf(fp,"set ylabel \"y [m]\"\n");
    
    fprintf(fp,"plot \"%02d.txt\" using 1:2 lc rgb \"#FF0000\" title '' w lines,",idx);
    fprintf(fp,"\"< head -1 %02d.txt\" using 1:2 lc rgb \"#000000\" pt 4 ps 1 lw 2 title 'Sequence Start' w points\n",idx);
    
    // close file
    fclose(fp);
    
    // run gnuplot => create png + eps
    sprintf(command,"cd %s; gnuplot %s",dir.c_str(),file_name);
    system(command);
  }
  
  // create pdf and crop
  sprintf(command,"cd %s; ps2pdf %02d.eps %02d_large.pdf",dir.c_str(),idx,idx);
  system(command);
  sprintf(command,"cd %s; pdfcrop %02d_large.pdf %02d.pdf",dir.c_str(),idx,idx);
  system(command);
  sprintf(command,"cd %s; rm %02d_large.pdf",dir.c_str(),idx);
  system(command);

}

vector<MatrixXd> loadPoses(string file_name){
  vector<MatrixXd> poses;
  FILE *fp = fopen(file_name.c_str(),"r");
  if (!fp)
    return poses;
  while (!feof(fp)) {

    

#ifdef POINT3D  
    MatrixXd P = MatrixXd::Identity(4, 4);
    if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P(0,0), &P(0,1), &P(0,2), &P(0,3),
                   &P(1,0), &P(1,1), &P(1,2), &P(1,3),
                   &P(2,0), &P(2,1), &P(2,2), &P(2,3),
                   &P(3,0), &P(3,1), &P(3,2), &P(3,3))==16) {
      poses.push_back(P);
    }

#else
    MatrixXd P = MatrixXd::Identity(3, 3);
    if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P(0,0), &P(0,1), &P(0,2),
                   &P(1,0), &P(1,1), &P(1,2),
                   &P(2,0), &P(2,1), &P(2,2))==9) {
      poses.push_back(P);
    }

#endif
  }
  fclose(fp);
  return poses;
}

vector<VectorXd> loadPoses1(string file_name) {
  vector<VectorXd> point;
  FILE *fp = fopen(file_name.c_str(),"r");
  if (!fp)
    return point;
#ifdef POINT3D
   Vector3d P;
#else
   Vector2d P;
#endif
  while (!feof(fp)) {
   // MatrixXd P = MatrixXd::Identity(3, 3);
#ifdef POINT3D
    if (fscanf(fp, "%lf %lf %lf",
                  &P(0),&P(1),&P(2))==3) {
      point.push_back(P);
    }

#else
     if (fscanf(fp, "%lf %lf",
                  &P(0),&P(1))==2) {
      point.push_back(P);

  }
#endif
}
  fclose(fp);
  return point;
}


void interpintv(Vector3d pt1,Vector3d pt2,double intv,Vector3d &newpt)
{
  Vector3d dirvec=pt2-pt1;

#ifndef POINT3D  
  dirvec(2)=0;
#endif

  double value=dirvec.norm();
  dirvec=dirvec/value;
  double l=dirvec(0);
  double m=dirvec(1);
  newpt(0)=intv*l+pt1(0);
  newpt(1)=intv*m+pt1(1);
  newpt(2)=intv*dirvec(2)+pt1(2);  
}


void curvspace(unsigned int s_num,curvesVector &points)
{
   Curves *curves_ini;
   curves_ini=points[0];
   Vector3d currentpt1=curves_ini->points1;
   Vector3d currentpt2=curves_ini->points2;
   //double   current_time=curves_ini->time_stamps;
   Vector3d newpt1(0,0,0);
   unsigned int indfirst1=1;
   unsigned int indfirst2=1;
   Vector3d q1=currentpt1;
   Vector3d q2=currentpt2;
   V_point1.clear();
   V_point2.clear();
   //V_time.clear();
   V_point1.push_back(q1);
   V_point2.push_back(q2);
   //V_time.push_back(current_time);
   double distsum1;
   Vector3d ptnow1;
   int kk1;
   Vector3d pttarget1;
   double remainder1;
   Vector3d diss1;   
   double disttmp1;
  
   unsigned int len=points.size();
   double dist1[len-1];
   double dist2[len-1];
   double totaldist1=0;
   double totaldist2=0;
   double intv1=0;
   double intv2=0;
 
   for(unsigned int i=0;i<len-1;i++){
   
     Curves* c_point = points[i];
     Curves* n_point = points[i+1];
     Vector3d c_p1=c_point->points1;
     Vector3d n_p1=n_point->points1;

     Vector3d c_p2=c_point->points2;
     Vector3d n_p2=n_point->points2;

     Vector3d dis1=c_p1-n_p1;
     Vector3d dis2=c_p2-n_p2;
#ifdef POINT3D
     dist1[i]=sqrt(dis1(0)*dis1(0)+dis1(1)*dis1(1)+dis1(2)*dis1(2));
     dist2[i]=sqrt(dis2(0)*dis2(0)+dis2(1)*dis2(1)+dis2(2)*dis2(2));
#else
     dist1[i]=sqrt(dis1(0)*dis1(0)+dis1(1)*dis1(1));
     dist2[i]=sqrt(dis2(0)*dis2(0)+dis2(1)*dis2(1));
#endif
    }
   
    for(unsigned int j=0;j<len-1;j++){
     totaldist1+=dist1[j];
     totaldist2+=dist2[j]; 
    }
     intv1=totaldist1/(s_num-1);
     intv2=totaldist2/(s_num-1);
    
    for(unsigned int k = 1; k <= s_num-1; k++){
    
      newpt1<<0,0,0;
      distsum1=0;
      ptnow1=currentpt1;
      //pt_time=current_time;
      kk1=0;
      pttarget1=points[indfirst1]->points1;
      remainder1 = intv1;
      while ((newpt1(0)==0)&&(newpt1(1)==0)&&(newpt1(2)==0)){
      
       diss1=ptnow1-pttarget1;

#ifdef POINT3D        
       disttmp1=sqrt(diss1(0)*diss1(0)+diss1(1)*diss1(1)+diss1(2)*diss1(2));
#else
       disttmp1=sqrt(diss1(0)*diss1(0)+diss1(1)*diss1(1));
#endif       
       distsum1=distsum1+disttmp1;
       
       if (distsum1>=intv1){
         interpintv(ptnow1,pttarget1,remainder1,newpt1);
        // new_time=pt_time;
       }
       else {
         remainder1=remainder1 - disttmp1;
         ptnow1=pttarget1;
         kk1=kk1+1;
         if((indfirst1+kk1)>(len-1))
            newpt1=((points[len-1])->points1);
         
         else  pttarget1 = ((points[indfirst1+kk1])->points1);
       
       }

       }

       Vector3d newpt2(0,0,0);
       double distsum2=0;
       Vector3d ptnow2=currentpt2;
       int kk2=0;
       Vector3d pttarget2=points[indfirst2]->points2;
       double remainder2 = intv2;
       while ((newpt2(0)==0)&&(newpt2(1)==0)&&(newpt2(2)==0)){

       Vector3d diss2=ptnow2-pttarget2;

       double disttmp2=sqrt(diss2(0)*diss2(0)+diss2(1)*diss2(1)+diss2(2)*diss2(2));
       distsum2=distsum2+disttmp2;
        if (distsum2>=intv2)
         interpintv(ptnow2,pttarget2,remainder2,newpt2);
        else {
         remainder2=remainder2 - disttmp2;
         ptnow2=pttarget2;
         kk2=kk2+1;

         if((indfirst2+kk2)>(len-1))
            newpt2=((points[len-1])->points2);
         else  pttarget2 = ((points[indfirst2+kk2])->points2);
       
         }

        }
         
         V_point1.push_back(newpt1);
         V_point2.push_back(newpt2);
         currentpt1=newpt1;
         currentpt2=newpt2;
        
         indfirst1 = indfirst1 + kk1;
         indfirst2 = indfirst2 + kk2;
        
      }
       
}

void curve_rep(curvesVector &points,int type)
{
  
  if(type==1){
  
  for(curvesVector::iterator it = points.begin();
      it != points.end();
      ++it) {
    
       Curves *CurrentPoint = *it;

       if(it==(points.end()-1)){
       Curves *NextPoint = points.front();
       CurrentPoint->Trans_Mat(CurrentPoint,NextPoint);
       }     
       else{
       Curves *NextPoint = *(it+1);
       CurrentPoint->Trans_Mat(CurrentPoint,NextPoint);
       }     
             
   }
  }

  else{

    for(curvesVector::iterator it = points.begin();
      it != (points.end()-1);
      ++it) {
        
       Curves *CurrentPoint = *it;
       Curves *NextPoint = *(it+1);
       CurrentPoint->Trans_Mat(CurrentPoint,NextPoint);
    }
       
  }
      
}       

/*void correspondence(curvesVector &points,int type)
{
  inv_rep(points,0);
  if(type==1) 
  {
    




  }


}*/

void general_align(curvesVector &points,int type)
{
  
   Matrix2d m;

   m(0,0) = 1;
   m(0,1) = 0;
   m(1,0) = 0;
   m(1,1) = 1;
   AbelianR.push_back(m);

   m(0,0) = -1;
   m(0,1) = 0;
   m(1,0) = 0;
   m(1,1) = 1;
   AbelianR.push_back(m);

   m(0,0) = 1;
   m(0,1) = 0;
   m(1,0) = 0;
   m(1,1) = -1;
   AbelianR.push_back(m);

   m(0,0) = -1;
   m(0,1) = 0;
   m(1,0) = 0;
   m(1,1) = -1;
   AbelianR.push_back(m);

   for(int i=0;i<4;i++){
     
      for(unsigned int j = 0; j < points.size(); j++){
          
          Vector2d point2d;
          Curves *curves;
          curves=points[j];
          for(int k=0; k<2;k++)
          point2d(k)=(curves->points2(k));
          point2d=(AbelianR[i]*point2d);
          for(int h=0;h<2;h++ )
          (curves->points2(h))=point2d(h);    
      }
      
      curve_rep(points,type);
     // correspondence(points,type);
 

   }



}

int main(int argc, char **argv)
{
  
  ros::init(argc,argv, "curve");
  ros::NodeHandle node;
  ROS_INFO("Node created!\n");

  unsigned int num_points=1,sample_points=1;
  int type=1;
  int steps=5;
  int pose_index=0;
  int interval;
  string dir,bagfile; 
  std::vector<double> point[7];

  parseArgs(argc, argv, dir, num_points,sample_points,type,steps,pose_index);
  node.param<std::string>("/log/bagfile",bagfile,"hector.bag");
  bagfile=dir+bagfile;
  
  rosbag::Bag bag(bagfile);
  timestamps stamps=timestamps(&bag);

  stamps.extractTrajectory(point);
  //message_filters
  
  double n=double(num_points);
  double s=double(sample_points); 
  //reading sampled  points of trajectories

  interval=ceil(n/s);

  cout<<"the number of sample point is"<<" "<<ceil(n/interval)<<endl;

  for(unsigned int i=0; i< point[0].size()-1; i++)    
  Curves::allpoints.push_back(new Curves(point,type,i));
  //cout<<point[0].size()<<endl;
  cout<<"read data from file"<<endl;
   

#ifdef UNIFORM
   curvspace(num_points,Curves::allpoints);
   
   for(unsigned int j = 0; j < Curves::allpoints.size(); j++){

    Curves *curves;
    curves=Curves::allpoints[j];
    curves->points1=V_point1[j];
    curves->points2=V_point2[j];
    //Curves::newpoints.push_back(curves);
   } 
        
#endif
    
    curve_rep(Curves::allpoints,type);
  

#ifndef CORRESPONDENCE
   
#ifndef POINT3D          //for planar curves 
   general_align(Curves::allpoints,type);  
#else                    //for non-planar curves
   //c2 = correspondence(c1,c2); 
   //c2 = align_curves(c1,c2);  
#endif

#endif
 

#ifdef UNIFORM
   curvspace(sample_points,Curves::allpoints);

   for(int k = 0; k < sample_points; k++){

    Curves *curves;
    curves=Curves::allpoints[k];
    curves->points1=V_point1[k];
    curves->points2=V_point2[k];
    Curves::Samplepoints.push_back(curves);
   } 
#else

   for(unsigned int j = 0; j < Curves::allpoints.size(); j=j+interval){

    Curves *curves;
    curves=Curves::allpoints[j];
    Curves::Samplepoints.push_back(curves);
   } 
#endif

     cout<<"sample data is finished"<<endl;
     num=Curves::Samplepoints.size();
     curve_rep(Curves::Samplepoints,type);

     geodesic_path(Curves::Samplepoints,steps,dir,pose_index); 
      
     cout<<"get fusion trajectory"<<endl;



     cout<<"start to save result"<<endl;
     // for all sequences do
     string plot_pose_dir  = dir + "/pose_path";
     string plot_point_dir  = dir + "/point_path";


     system(("mkdir " + plot_pose_dir).c_str());
     system(("mkdir " + plot_point_dir).c_str());

       
     for (int i=0; i<=steps; i++) {
   
      // file name
        char file_name[256];
        sprintf(file_name,"%02d.txt",i);
        
      // read ground truth and result poses
        vector<MatrixXd> poses_result = loadPoses(pose_dir  + file_name);
        vector<VectorXd> point_result = loadPoses1(point_dir + file_name);

      // check for errors
     if (poses_result.size()==0 || point_result.size()==0) {
        cout<<"ERROR: pose file and point cloud file are empty %s"<<endl;
        abort();
     }
    
      // save + plot bird's eye view trajectories
      savePathPlot(poses_result,point_result,plot_pose_dir + "/" + file_name,plot_point_dir + "/" + file_name);
      
      vector<int32_t> roi_pose = computeRoi1(poses_result);
      vector<int32_t> roi_point = computeRoi2(point_result);
      
      //plotPathPlot(plot_pose_dir,roi_pose,i,steps);
      plotPathPlot(plot_point_dir,roi_point,i,steps);
    }
     
     cout<<"the process is complete"<<endl<<endl<<endl;

     
    
     cout<<"please input any key to continue next step! "<<endl<<endl<<endl;
     
     cin.get();
     
     //#ifdef PUBLISHTF
     cout<<"plase open a new teminal to replay bag file"<<endl<<endl<<endl;
     cout<<"start to broacaste /tf"<<endl<<endl<<endl;
    
     tf_broadcaster broadcaster(matrix_new); 
     matrix_new.clear();
        
//#endif

     return 0;
}










