
/* Program:
 * This program is written for fusing GPS with hector.
 * History:
 * 16/03/2018	
 * Copyright (C) Shitong Du
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

string pose_dir;
string point_dir;
int parseArgs(int argc,char **argv, string &dir,  int &num_points,  int &sample_points,int &type,int &steps){
  num_points   = 1;
  sample_points = 1; // -1 indicates no limitation
  type=1;
  steps=5;
  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  std::cout << std::endl;
  while ((c = getopt (argc, argv, "n:s:t:f:")) != -1)
    switch (c)
   {
   case 'n':
     num_points = atoi(optarg);
     if (num_points <= 0) { std::cerr << "Error: Cannot start at a negative or zero point number.\n"; exit(1); }
     break;
   case 's':
     sample_points = atoi(optarg);
     if (sample_points <=0)     { std::cerr << "Error: Cannot sample with a negative or zero point number.\n"; exit(1); }
     if (sample_points > num_points) { std::cerr << "Error: <sample> cannot be bigger than <num_points>.\n"; exit(1); }
     break;
   case 't':
     type = atoi(optarg);
     //if ((type!=0)||(type!=1))     { std::cerr << "Error: type should be 1 or 0.\n"; exit(1); }  
    // break;
     if(type>1)        { std::cerr << "Error: type should be 1 or 0.\n"; exit(1); } 
   case 'f':
     steps = atoi(optarg);
     if (steps==0)     { std::cerr << "Error: steps cannot be 0.\n"; exit(1); }  
     break;
   }

  if (optind != argc-1) {
    std::cerr << "\n*** Directory missing ***\n" << std::endl; 
    std::cout << std::endl
              << "Usage: " << argv[0] << "  [-n NR] [-s NR] [-t NR] [-f NR] directory" << std::endl << std::endl;
    std::cout << "  -n NR   the number of points"  << std::endl<< "  -s NR   the number of sample points" << "" << std::endl
          << "  -t NR   the type of the curve. 0:open curve; 1: closed curve" << "" << std::endl
	  << std::endl;
    std::cout <<" -f NR the number of fusion steps"<< std::endl;
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

void inv_rep(curvesVector &Samplepoints)
{
  
 // unsigned int l=Samplepoints.size();
  //Curves* s_point;
  Vector3d start_point;
  start_point(0)=0;
  start_point(1)=0;
  start_point(2)=1;

  Curves* s_point = Curves::Samplepoints[0];
 

  s_point->Fus_points=start_point;
  for(size_t i = 1; i < Curves::Samplepoints.size(); ++i){
   
  //这里matlab用的l是rep的个数，所以闭曲线和开曲线是有差别的，但是我用的是点的个数，所以可以跳过判断曲线开闭的步骤。
  Curves* c_point = Curves::Samplepoints[i];
  Curves* p_point = Curves::Samplepoints[i-1];
  c_point->Fus_points=(p_point->Fus_transfor)*(p_point->Fus_points);
 
  }

}

void savedata(curvesVector &Samplepoints,string dir,string &pose_dir,string &point_dir,int count,int steps)
{
   
  
   //std::map<int,vector<Curves*> >::iterator fuse_traje;
   //for(fuse_traje=Curvesnum.begin();fuse_traje!=Curvesnum.end();fuse_traje++){
     //int stamp = fuse_traje->first;

     //char file_name[255];
     string PoseFileName = pose_dir +to_string(count,2)+ ".txt";
     string PointFileName = point_dir +to_string(count,2)+ ".txt";
     //curvesVector fuse_curve = fuse_traje->second;

     // open file  
     FILE *fp1 = fopen(PoseFileName.c_str(),"w");
     FILE *fp2 = fopen(PointFileName.c_str(),"w");

     if(count==0){

        //cout<<fuse_curve.size()<<endl;
       for(unsigned int i = 0; i < Samplepoints.size(); i++)
       {
     
         Curves* c_point = Samplepoints[i];
         MatrixXd posematrix=c_point->transformation1;
         VectorXd point=c_point->points1;
        //MatrixXd P=MatrixXd::Identity(3,3);
        // parameters
         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",posematrix(0,0),posematrix(0,1),posematrix(0,2),
                                                posematrix(1,0),posematrix(1,1),posematrix(1,2),
                                                posematrix(2,0),posematrix(2,1),posematrix(2,2));
         fprintf(fp2,"%lf %lf\n",point(0),point(1));
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
         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",posematrix(0,0),posematrix(0,1),posematrix(0,2),
                                                posematrix(1,0),posematrix(1,1),posematrix(1,2),
                                                posematrix(2,0),posematrix(2,1),posematrix(2,2));
         fprintf(fp2,"%lf %lf\n",point(0),point(1));
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
         fprintf(fp1,"%lf %lf %lf %lf %lf %lf %lf %lf %lf\n",posematrix(0,0),posematrix(0,1),posematrix(0,2),
                                                posematrix(1,0),posematrix(1,1),posematrix(1,2),
                                                posematrix(2,0),posematrix(2,1),posematrix(2,2));
         fprintf(fp2,"%lf %lf\n",point(0),point(1));
        }
     }
    // close file
     fclose(fp1);
     fclose(fp2);
 // }
   

}


void geodesic_path(curvesVector &Samplepoints,int steps,string dir)
{
   double timestep=1/((double)steps);
   double step=0;
   //Matrix2d R_c;
   Matrix2cd Rc;
   Matrix2d R;
   Vector2d trans;
   pose_dir  = dir + "/fuse_pose/";
   point_dir  = dir + "/new_point/";

   // create output directories
   system(("mkdir " + pose_dir).c_str());
   system(("mkdir " + point_dir).c_str());
   //int i;

   //std::vector<Curves*> Oripoints;
   //Curvesnum.insert(pair<int, vector<Curves*> >(0,Samplepoints));
  // Curvesnum.insert(pair<int, vector<Curves*> >(steps,Samplepoints));
  for(unsigned int j = 0; j < Curves::Samplepoints.size(); j++){

   Curves *curves;
   curves=Curves::Samplepoints[j];
   Curves::Oripoints.push_back(curves);
  }

   for(int i=0;i<=steps;i++){

     //if(i!=0||i!=steps){
      if(i==0)
      savedata(Curves::Oripoints,dir,pose_dir,point_dir,i,steps);

      else if(i==steps)
      savedata(Curves::Oripoints,dir,pose_dir,point_dir,i,steps);
      
      else{

      step = step + timestep;
      for(curvesVector::iterator it = Curves::Samplepoints.begin();
      it != Curves::Samplepoints.end();
      ++it) {
       
       Curves *C_Point = *it;
       MatrixXd R1=C_Point->rot1;
       MatrixXd R2=C_Point->rot2;
       VectorXd tra1=C_Point->trans1;
       VectorXd tra2=C_Point->trans2;
       //这里matlab中用的R1的转置，但是论文中写的是逆。
       MatrixXd Ri=(R1.inverse())*R2;
   
        Rc.real()= Ri;
        Rc.imag()<<0,0,0,0;   
       MatrixPower<Matrix2cd> Apow(Rc);
       R= (R1*Apow(step)).real();
       trans=tra1+(tra2-tra1)*step;  
       //Curves::geodesicPoint.push_back(allpoints[j]);
       for(int j = 0; j < 2; j++){
        for(int k = 0; k < 2; k++){
        C_Point->Fus_transfor(j, k) =R(j, k);     
        }
       }

       for(int j = 0; j < 2; j++){
        C_Point->Fus_transfor(j, 2)=trans(j);
       }

        C_Point->Fus_transfor(2, 0)=0;
        C_Point->Fus_transfor(2, 1)=0;
        C_Point->Fus_transfor(2, 2)=1.0;
        //(C_Point->Fus_transfor)=transformation;  
     }
        inv_rep(Curves::Samplepoints);
        savedata(Samplepoints,dir,pose_dir,point_dir,i,steps);
   }

   

  }
       
}

vector<int32_t> computeRoi1 (vector<MatrixXd> &poses_result){
  
  float x_min = numeric_limits<int32_t>::max();
  float x_max = numeric_limits<int32_t>::min();
  float y_min = numeric_limits<int32_t>::max();
  float y_max = numeric_limits<int32_t>::min();
  cout<<5<<endl;
  for(unsigned int i = 0; i < poses_result.size(); i++){
  
   Matrix3d P=poses_result[i];
   double x = P(0,2);
   double y = P(1,2);
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
  
   Vector2d P=point_result[i];
   double x = P(0);
   double y = P(1);
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
  fprintf(fp1,"%f %f\n",(poses_result[i])(0,2),(poses_result[i])(1,2));
  fclose(fp1);

  FILE *fp2 = fopen(file2_name.c_str(),"w");
  for (unsigned int j=0; j<point_result.size(); j++)
    fprintf(fp2,"%f %f\n",(point_result[j])(0),(point_result[j])(1));
  // close file
  
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
    MatrixXd P = MatrixXd::Identity(3, 3);
    if (fscanf(fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf",
                   &P(0,0), &P(0,1), &P(0,2),
                   &P(1,0), &P(1,1), &P(1,2),
                   &P(2,0), &P(2,1), &P(2,2))==9) {
      poses.push_back(P);
    }
  }
  fclose(fp);
  return poses;
}

vector<VectorXd> loadPoses1(string file_name) {
  vector<VectorXd> point;
  FILE *fp = fopen(file_name.c_str(),"r");
  if (!fp)
    return point;
   Vector2d P;
  while (!feof(fp)) {
   // MatrixXd P = MatrixXd::Identity(3, 3);
    if (fscanf(fp, "%lf %lf",
                  &P(0),&P(1))==2) {
      point.push_back(P);
    }
  }
  fclose(fp);
  return point;
}

int main(int argc, char **argv)
{
   int num_points = 1, sample_points =1;
  int type=1;
  int steps=5;
  int interval;
  string dir; 
  //double 
  parseArgs(argc, argv, dir, num_points,sample_points,type,steps);
  double n=double(num_points);
  double s=double(sample_points); 
  //reading sampled  points of trajectories
 // 这里需要转化格式
  interval=ceil(n/s);

  cout<<"the number of sample point is"<<" "<<ceil(n/interval)<<endl;

  for(int i=0; i< num_points; i++){
     
   Curves::allpoints.push_back(new Curves(dir,type,i));

  }
   
  cout<<"read data from file"<<endl;
  for(unsigned int j = 0; j < Curves::allpoints.size(); j=j+interval){

   Curves *curves;
   curves=Curves::allpoints[j];
   Curves::Samplepoints.push_back(curves);
  }

  cout<<"sample data is finished"<<endl;

  if(type==1){
  
  for(curvesVector::iterator it = Curves::Samplepoints.begin();
      it != Curves::Samplepoints.end();
      ++it) {
    
       Curves *CurrentPoint = *it;

       if(it==(Curves::Samplepoints.end()-1)){
       Curves *NextPoint = Curves::Samplepoints.front();
       CurrentPoint->Trans_Mat(CurrentPoint,NextPoint);
       }     
       else{
       Curves *NextPoint = *(it+1);
       CurrentPoint->Trans_Mat(CurrentPoint,NextPoint);
       }     
             
   }
  }

  else{

    for(curvesVector::iterator it = Curves::Samplepoints.begin();
      it != (Curves::Samplepoints.end()-1);
      ++it) {
        
       Curves *CurrentPoint = *it;
       Curves *NextPoint = *(it+1);
       CurrentPoint->Trans_Mat(CurrentPoint,NextPoint);
    }
       
  }
      
       geodesic_path(Curves::Samplepoints,steps,dir); 
      
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
     
     cout<<"the process is complete"<<endl;
    return 0;
}










