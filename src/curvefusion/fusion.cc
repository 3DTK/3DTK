
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

#include "curvefusion/curves.h"
#include "getopt.h"
#include "slam6d/globals.icc"
#include "curvefusion/extractTrajectory.h"
#include "curvefusion/tf_broadcaster.h"

#include "slam6d/icp6Dsvd.h"
#include "curvefusion/common.h"

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

static std::vector<double> V_time;


//unsigned int seq_interval[1000]; 

int parseArgs(int argc,char **argv, string &dir,  unsigned int &num_points,  unsigned int &sample_points,int &type,int & steps,int &pose_index,double &beta,int &window_size){
  num_points   = 1;
  sample_points = 1; // -1 indicates no limitation
  type=1;
  steps=5;
  pose_index=1;
  int  c;
  beta=0.1;
  window_size=50;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  std::cout << std::endl;
  while ((c = getopt (argc, argv, "n:s:t:p:i:b:w:")) != -1)
    switch (c)
    { 
   
   case 'n':
     num_points = atoi(optarg);
     if (num_points <= 0) { std::cerr << "Error: Cannot start at a negative or zero point number.\n"; exit(1); }
     break;
   case 's':
     sample_points = atoi(optarg);
     if (sample_points <=0)     { std::cerr << "Error: Cannot sample with a negative or zero point number.\n"; exit(1); }
   case 't':
     type = atoi(optarg);
     //if ((type!=0)||(type!=1))     { std::cerr << "Error: type should be 1 or 0.\n"; exit(1); }  
    // break;
    // if(type>=2)        { std::cerr << "Error: type should be 1 or 0.\n"; exit(1); } 
   case 'p':
      steps = atoi(optarg);
     //if (steps==0)     { std::cerr << "Error: steps cannot be 0.\n"; exit(1); }  
     break;
   case 'i':
    pose_index = atoi(optarg);
     if ((pose_index>(steps+1))||(pose_index<0))     { std::cerr << "Error: pose_index cannot exceed pose_num and be less than 0.\n"; exit(1); }  
     break;
   case 'b':
     beta = atof(optarg);
   //  if ((pose_index>(steps+1))||(pose_index<0))     { std::cerr << "Error: pose_index cannot exceed pose_num and be less than 0.\n"; exit(1); }  
     break;
   case 'w':
     window_size = atoi(optarg);
    // if ((pose_index>(steps+1))||(pose_index<0))     { std::cerr << "Error: pose_index cannot exceed pose_num and be less than 0.\n"; exit(1); }  
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






int main(int argc, char **argv)
{
  
  ros::init(argc,argv, "curve");
  ros::NodeHandle node;
  ROS_INFO("Node created!\n");

  unsigned int num_points=1,sample_points=1;
  int type=1;
  int steps=5;
  int pose_index=0;
  //int interval;
  double beta=0.1;
  int window_size=50;
  string dir,bagfile_ros1,bagfile_ros2,rieglfile;
  string plot_pose_dir;
  string plot_point_dir;
  std::vector<double> point_curve1[4];
  std::vector<double> point_curve2[4];
  std::vector<double> hector_matrix[8];
  std::vector<double> point_corr[7];
  parseArgs(argc, argv, dir, num_points,sample_points,type,steps,pose_index,beta,window_size);
  ///////node.param<std::string>("/log/bagfile1",bagfile_ros,"ros.bag");
  node.param<std::string>("/log/bagfile1",bagfile_ros1,"ros1.bag");
  //node.param<std::string>("/log/bagfile2",bagfile_odroid,"odroid.bag");
  node.param<std::string>("/log/bagfile2",bagfile_ros2,"ros2.bag");
  node.param<std::string>("/log/rieglfile",rieglfile,"raw.rxp");
 
  bagfile_ros1=dir+bagfile_ros1;
  bagfile_ros2=dir+bagfile_ros2;
  rieglfile=dir+rieglfile;
  ////rosbag::Bag bag1(bagfile_ros1);
 //// rosbag::Bag bag2(bagfile_ros2);
  string str1="/slam_out_pose";
  string str2="/fix";
  // extract two curves from .bag file.
  //extractTrajectory stamps=extractTrajectory(&bag);


#ifdef GPS_HECTOR
#ifdef READBAG 
  
#ifdef SEPERATE_ROSCORE   
  rosbag::Bag bag2(bagfile_ros2);
  extract2hector_gps *trajectory=new extract2hector_gps(&bag1,&bag2);
#else
  extract2hector_gps *trajectory=new extract2hector_gps(&bag1);
#endif
  trajectory->aligncurve(point_curve1,point_curve2,hector_matrix,dir);
#else
   //extract2hector_gps *trajectory=new extract2hector_gps(&bag1);
   readkitti(hector_matrix,point_corr,dir);
#endif
  //the coordinate system of gps and laser data are not in the same coordinate system. 
  //so,we use aligncurve function to align these two coordiante systems.
  //trajectory->aligncurve(point_curve1,point_curve2,hector_matrix,dir);
#else
 // #define OPTIMAL_CORRESPODENCE
#ifdef READFILE
  extract2curve1_curve2 *trajectory=new extract2curve1_curve2(&bag1,dir,1);
#else
  //extract2curve1_curve2 *trajectory=new extract2curve1_curve2(&bag1,rieglfile);
  //extract2curve1_curve2 *trajectory=new extract2curve1_curve2(&bag1,rieglfile);
   extract2curve1_curve2 *trajectory=new extract2curve1_curve2(&bag1,&bag2);
#endif
  trajectory->aligncurve(point_curve1,point_curve2,dir);
#endif

 unsigned int h_seq[point_corr[0].size()-1];

#ifdef OPTIMAL_CORRESPODENCE
  
  cout<<"use the optimal method to find the point correspondence"<<endl;
  
  Optimal_correspondence(point_curve1,point_curve2,point_corr,h_seq,dir,type,beta, window_size);
#endif 
  //closest_correspondence(point_curve2,point_curve1,point_corr,h_seq,dir);  
#ifdef TIMESTAMPS_CORRESPONDENCE

   cout<<"use the timestamp to find the point correspondence"<<endl;
   timestamp_correspondence(point_curve1,point_curve2,point_corr,h_seq,dir);
#endif
   for(unsigned int i=0; i<=(point_corr[0].size()-1);i++)
   h_seq[i]=1;
  //message_filters


	   
/********************************************************************************************/  

#ifdef GPS_HECTOR
//  double n=double(num_points);
//  double s=double(sample_points); 
  //reading sampled  points of trajectories

 // interval=ceil(n/s);

 // cout<<"the number of sample point is"<<" "<<ceil(n/interval)<<endl;
  
  
  cout<<"read data from file"<<endl;

  for(unsigned int i=0; i<= point_corr[0].size()-1; i++)    
  Curves::allpoints.push_back(new Curves(point_corr,type,i));

  cout<< "read data complete"<<endl;
#ifdef UNIFORM
   curvspace(num_points,Curves::allpoints);
   
   for(unsigned int j = 0; j < Curves::allpoints.size(); j++){

    Curves *curves;
    curves=Curves::allpoints[j];
    curves->points1=V_point1[j];
    curves->points2=V_point2[j];
   } 
        
#endif
    
    curve_rep(Curves::allpoints,type);

#ifndef UNIFORM_CORRESPODENCE
   
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

   for(unsigned int j = 0; j < Curves::allpoints.size(); j++){

    Curves *curves;
    curves=Curves::allpoints[j];
    Curves::Samplepoints.push_back(curves);
   } 
#endif
     //cout<<Curves::allpoints.size()-1<<endl;

     curve_rep(Curves::Samplepoints,type);
     geodesic_path(Curves::Samplepoints,steps,dir,pose_index,hector_matrix,h_seq); 
      
     cout<<"get fusion trajectory"<<endl;

     cout<<"start to save result"<<endl;
     // for all sequences do

     // system(("mkdir " + pose_dir).c_str());
  // system(("mkdir " + pose_dir_final).c_str()); 

  // system(("mkdir " + point_dir).c_str());
  // system(("mkdir " + point_dir_final).c_str()); 


     plot_pose_dir  = dir + "pose_path";
     plot_point_dir  = dir + "point_path";


     system(("mkdir" + plot_pose_dir).c_str());
     system(("mkdir" + plot_point_dir).c_str());
    
       
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

    
    // cout<<"please input any key to continue next step! "<<endl<<endl<<endl;
     
    // cin.get();
     
#ifdef PUBLISHTF
     cout<<"plase open a new teminal to replay bag file"<<endl<<endl<<endl;
     cout<<"start to broacaste /tf"<<endl<<endl<<endl;
     system("gnome-terminal -e /home/du/Documents/xinde/record.sh");
     tf_broadcaster broadcaster;
     broadcaster.broadcaster_tf();
#endif    	
    // matrix_new.clear();
      
#endif
     return 0;
}










