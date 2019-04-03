/* Program:
 * This program is written for finding correspondence point by timestamps from different sensors.
 * History:
 * 3/05/2018	
 * Copyright (C) Shitong Du
 */
#include "curvefusion/common.h"
using namespace std;  
int firstIndex=0;
//std::map<double, geometry_msgs::PoseStamped> gps_timetable;
extract2hector_gps::extract2hector_gps(rosbag::Bag *_bag1):extractTrajectory(_bag1,_bag1) {

   rosbag::View view_hector(*bag1, rosbag::TopicQuery(curve1_topic));
   BOOST_FOREACH(rosbag::MessageInstance const m, view_hector) {
   
    if(m.isType<geometry_msgs::PoseStamped>() ) {
      geometry_msgs::PoseStampedConstPtr poseptr = m.instantiate<geometry_msgs::PoseStamped>();
      geometry_msgs::PoseStamped pose;
      pose.header	= poseptr->header;
      pose.pose	=poseptr->pose;

      hector_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(pose.header.stamp.toSec(),pose));
    }
  }
  int gps_flag=1;
  rosbag::View view_gps(*bag1, rosbag::TopicQuery(curve2_topic));
  BOOST_FOREACH(rosbag::MessageInstance const m,view_gps) {
		
    if(m.isType<sensor_msgs::NavSatFix>() ) {
      sensor_msgs::NavSatFixConstPtr fixptr = m.instantiate<sensor_msgs::NavSatFix>();
      callback_gps(fixptr,gps_flag);
     }
  }


} 
  
extract2hector_gps::extract2hector_gps(rosbag::Bag *_bag1,rosbag::Bag *_bagod):extractTrajectory(_bag1,_bag1),bagod(_bagod){

   rosbag::View view_hector(*bag1, rosbag::TopicQuery(curve1_topic));
   BOOST_FOREACH(rosbag::MessageInstance const m, view_hector) {
   
    if(m.isType<geometry_msgs::PoseStamped>() ) {
      geometry_msgs::PoseStampedConstPtr poseptr = m.instantiate<geometry_msgs::PoseStamped>();
      geometry_msgs::PoseStamped pose;
      pose.header	= poseptr->header;
      pose.pose	=poseptr->pose;

      hector_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(pose.header.stamp.toSec(),pose));
    }
  }
   int gps_flag=1;
  rosbag::View view_gps(*bagod, rosbag::TopicQuery(curve2_topic));
  BOOST_FOREACH(rosbag::MessageInstance const m,view_gps) {
		
    if(m.isType<sensor_msgs::NavSatFix>() ) {
      sensor_msgs::NavSatFixConstPtr fixptr = m.instantiate<sensor_msgs::NavSatFix>();
      callback_gps(fixptr,gps_flag);
     }
  }


} 
  
 
   //void callback_hector(const geometry_msgs::PoseStampedConstPtr &hectorpose);
void extract2hector_gps::aligncurve(std::vector<double> *point_curve1,std::vector<double> *point_curve2,std::vector<double> *hector_matrix,std::string dir) {

  
   std::map<double, geometry_msgs::PoseStamped>::iterator gps;
   cout<<"read gps positions......"<<endl;            
   for(gps=curve1_timetable.begin();gps!=curve1_timetable.end();gps++){
	
     
    // double stamp = gps->first;
     geometry_msgs::PoseStamped pose = gps->second;       
  
     point_curve1[0].push_back(pose.header.stamp.toSec()) ;
     point_curve1[1].push_back(pose.pose.position.x) ;
     point_curve1[2].push_back(pose.pose.position.y) ;
     point_curve1[3].push_back(pose.pose.position.z) ;
     //std::advance (gps,1);

    }

     


     string gpsFileName = dir +"gps_original"+ ".txt";
     string hectorFileName = dir +"hector_original"+ ".txt";
     
     // open file  
     FILE *fp1 = fopen(gpsFileName.c_str(),"w");
     FILE *fp2 = fopen(hectorFileName.c_str(),"w");

      for(unsigned int i = 0; i < point_curve1[0].size(); i++)

       fprintf(fp1,"%lf %lf %lf %lf\n",point_curve1[0][i],point_curve1[1][i],point_curve1[2][i],point_curve1[3][i]);
       fclose(fp1);

     cout<<"Read GPS data completed and begin to read hector pose"<<endl;

            
     std::map<double, geometry_msgs::PoseStamped>::iterator hector;
  
    for(hector=hector_timetable.begin();hector!=hector_timetable.end();hector++){
     
    //  double stamp = hector->first;
      geometry_msgs::PoseStamped pose = hector->second;
                        
      
       hector_matrix[0].push_back(pose.header.stamp.toSec());
       hector_matrix[1].push_back(pose.pose.orientation.x);
       hector_matrix[2].push_back(pose.pose.orientation.y);
       hector_matrix[3].push_back(pose.pose.orientation.z);
       hector_matrix[4].push_back(pose.pose.orientation.w);  
       hector_matrix[5].push_back(pose.pose.position.x) ;
       hector_matrix[6].push_back(pose.pose.position.y) ;
       hector_matrix[7].push_back(pose.pose.position.z) ;

       point_curve2[0].push_back(pose.header.stamp.toSec()) ;
       point_curve2[1].push_back(pose.pose.position.x) ;
       point_curve2[2].push_back(pose.pose.position.y) ;
       point_curve2[3].push_back(pose.pose.position.z) ;  
      // hector+=5;
      //hector++;
       //hector++;
      // hector++;
       //hector++;
      //std::advance (hector,12);
       
   }


    for(unsigned int i = 0; i < hector_matrix[0].size(); i++)

      fprintf(fp2,"%lf %lf %lf %lf\n",hector_matrix[0][i],hector_matrix[5][i],hector_matrix[6][i],hector_matrix[7][i]);
      fclose(fp2);
      cout<<"Read hector pose completed"<<endl; 
}



extract2curve1_curve2::extract2curve1_curve2(rosbag::Bag *_bag1,std::string &_rieglfile):extractTrajectory(_bag1,_bag1),filename(_rieglfile) {
   
     rosbag::View view_gps(*bag1, rosbag::TopicQuery(curve2_topic));
     BOOST_FOREACH(rosbag::MessageInstance const m,view_gps) {
     int gps_flag=1;
     if(m.isType<sensor_msgs::NavSatFix>() ) {
      sensor_msgs::NavSatFixConstPtr fixptr = m.instantiate<sensor_msgs::NavSatFix>();
      callback_gps(fixptr,gps_flag);
      }
     }
   

     // string filename = rieglfile ;

      cout << "Reading " << filename << "..." << endl;
      shared_ptr<basic_rconnection> rc;
      rc = basic_rconnection::create(filename);
      rc->open();
      decoder_rxpmarker dec(rc);
      buffer buf;
      GPS importer;
      for ( dec.get(buf); !dec.eoi() ; dec.get(buf) ) {
            importer.dispatch(buf.begin(), buf.end());
      }
      rc->close();

      cout << "Read " << importer._gpsInfo.size() << " gps messages." << endl;
       // cout << "Read " << importer._rollVector.size() << " inclination sensor messages." << endl;

   //   if (importer._gpsInfo.size() < 1 ) {
    //        cout << "Failed to read gps info!" << endl;   
    //        continue;
   //   }

       // GPS::GpsInfo bestGpsInfo;
       geometry_msgs::PoseStamped pose;
        startIndex=0;
       // double bestAccuracy = numeric_limits<double>::max();
     // for (GPS::GpsInfo gpsInfo : importer._gpsInfo) {
     for(unsigned int i=0;i<=(importer._gpsInfo).size()-1;i++) {     
        double timestamp=(importer._gpsInfo)[i].timestamp;
        //cout<<timestamp<<endl;
        double lat = (importer._gpsInfo)[i].lat;
        double lon = (importer._gpsInfo)[i].lon;
        double alt = (importer._gpsInfo)[i].alt;
        double Rea=6378137;
        double f=1/298.257223563;
        double Reb=Rea*(1-f);
        double e=sqrt(Rea*Rea - Reb*Reb)/Rea;
        double Ne=Rea/sqrt(1 - e*e*sin(lat)*sin(lat));

        double xe = (Ne + alt) * cos(lat) * cos(lon);
        double ye = (Ne + alt) * cos(lat) * sin(lon);
        double ze = (Ne*(1 -e*e) + alt) * sin(lat);

        Eigen::Affine3d translation(Eigen::Translation3d(Eigen::Vector3d(xe, ye, ze)));

	 
        // Transform ECEF frame to NED frame
        Eigen::Affine3d ecef2ned = Eigen::Affine3d(Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d(0, 1, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(lon, Eigen::Vector3d(1, 0, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(-lat, Eigen::Vector3d(0, 1, 0)));
        
    
//
        // Transform NED frame to ROS frame (x front, y left, z up)
        Eigen::Affine3d ned2nwu = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d(1, 0, 0)));
        // When the robot start to run,if x-axis point south direction,use the following Line of code
        Eigen::Affine3d nwu2ros = Eigen::Affine3d(Eigen::AngleAxisd(-135*M_PI/180, Eigen::Vector3d(0, 0, 1)));
        // When the robot start to run,if x-axis point east direction,use the following Line of code
       // Eigen::Affine3d nwu2ros = Eigen::Affine3d(Eigen::AngleAxisd(0.2* M_PI, Eigen::Vector3d(0, 0, 1)));
        // Local rotation from inclination sensors
        Eigen::Affine3d rotation = Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(1, 0, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1)));

       Eigen::Affine3d transform = ((translation * ecef2ned) *ned2nwu*nwu2ros) * rotation;
        //  Eigen::Affine3d transform = ((translation * ecef2ned) ); 
        // Use relative pose to ROS frame of pose at startIndex
        if (startIndex<1) {
            startTransform = ((translation * ecef2ned)*ned2nwu*nwu2ros);
            startIndex++;
        }

        Eigen::Affine3d posee = startTransform.inverse() * transform;

        // Write pose file
        if(i==0) {      
        pose.pose.position.x = posee.translation()(0);
        pose.pose.position.y = posee.translation()(1);
        pose.pose.position.z = posee.translation()(2);
        }

        else {      
        pose.pose.position.x = (posee.translation()(0)-0.20);//+5;
        pose.pose.position.y = (posee.translation()(1));//+15;
        pose.pose.position.z = (posee.translation()(2)-0.3);
        }

  

        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        curve2_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(timestamp,pose));


        } 
} 
  

void extract2curve1_curve2::aligncurve(std::vector<double> *point_curve1,std::vector<double> *point_curve2,std::string dir) {

  
    std::map<double, geometry_msgs::PoseStamped>::iterator curv1_index;  
       
    std::map<double, geometry_msgs::PoseStamped>::iterator iter_begin =  curve1_timetable.begin();
    std::map<double, geometry_msgs::PoseStamped>::iterator iter_end =  curve1_timetable.end();

    int   index =   distance(iter_begin,iter_end);
    cout<<index<<endl;

    for(curv1_index=curve1_timetable.begin();curv1_index!=curve1_timetable.end();curv1_index++){
     
     // double stamp = hector->first;
     
      geometry_msgs::PoseStamped pose = curv1_index->second;
      point_curve1[0].push_back(curv1_index->first) ;
      point_curve1[1].push_back(pose.pose.position.x) ;
      point_curve1[2].push_back(pose.pose.position.y) ;
      point_curve1[3].push_back(pose.pose.position.z) ;   
      
      std::advance (curv1_index,3);
     
   }

     string curve1FileName = dir +"curve1_original"+ ".txt";
     string curve2FileName = dir +"curve2_original"+ ".txt";
     
     
     // open file  
     FILE *fp1 = fopen(curve1FileName.c_str(),"w");
     FILE *fp2 = fopen(curve2FileName.c_str(),"w");

     for(unsigned int i = 0; i < point_curve1[0].size(); i++)
       fprintf(fp1,"%lf %lf %lf %lf\n",point_curve1[0][i],point_curve1[1][i],point_curve1[2][i],point_curve1[3][i]);
       fclose(fp1);
      cout<<"Read Curve1 data completed" <<endl;


   std::map<double, geometry_msgs::PoseStamped>::iterator curv2_index;
          
   for(curv2_index=curve2_timetable.begin();curv2_index!=curve2_timetable.end();curv2_index++){
	
     
    // double stamp = rieglgps->first;
     geometry_msgs::PoseStamped pose = curv2_index->second;       
  
     point_curve2[0].push_back(curv2_index->first) ;
     point_curve2[1].push_back(pose.pose.position.x) ;
     point_curve2[2].push_back(pose.pose.position.y) ;
     point_curve2[3].push_back(pose.pose.position.z) ;
 
    //// std::advance ( curv2_index,1);
     //curv2_index++;
     //curv2_index++;
    // curv2_index++;

    }

     

     

      for(unsigned int i = 0; i < point_curve2[0].size(); i++) //{

       fprintf(fp2,"%lf %lf %lf %lf\n",point_curve2[0][i],point_curve2[1][i],point_curve2[2][i],point_curve2[3][i]);
       //cout<<i<<endl;}
       fclose(fp2);

     cout<<"Read Curve2 data completed"<<endl;

            
     
}

extract2curve1_curve2::extract2curve1_curve2(rosbag::Bag *_bag1,std::string &_dir,int _flag):extractTrajectory(_bag1,_bag1),filename(_dir),flag(_flag) {
 
  string FileName = filename +"trajectory"+ ".data";
  cout<< FileName<<endl;
  ifstream pose_in;
  pose_in.open(FileName);
  double point_curve[7];
  unsigned int a=0;
  geometry_msgs::PoseStamped pose1;
  geometry_msgs::PoseStamped pose2;

  while (pose_in.good()){
   //cout<<5<<endl;

  // if(++a>40000)break;
 
   for (int i = 0; i < 7; ++i)
    pose_in >> point_curve[i];
  
    
   //if(k<20000)
   //continue;
   
   pose1.pose.position.x = point_curve[1];
   pose1.pose.position.y = point_curve[2];
   pose1.pose.position.z = point_curve[3]; 
   
   pose2.pose.position.x = point_curve[4];
   pose2.pose.position.y = point_curve[5];
   pose2.pose.position.z = point_curve[6];
  
   curve1_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(point_curve[0],pose1));
   curve2_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(point_curve[0],pose2));
 }

   pose_in.close();
   
}

extract2curve1_curve2::extract2curve1_curve2(rosbag::Bag *_bag1,rosbag::Bag *_bag2):extractTrajectory(_bag1,_bag2) {
    
     int gps_flag=1;
     rosbag::View view_gps(*bag1, rosbag::TopicQuery(curve2_topic));
     BOOST_FOREACH(rosbag::MessageInstance const m1,view_gps) {	
     if(m1.isType<sensor_msgs::NavSatFix>() ) {
      sensor_msgs::NavSatFixConstPtr fixptr1 = m1.instantiate<sensor_msgs::NavSatFix>();
      callback_gps(fixptr1,gps_flag);
      }
     }
     gps_flag=2;
     firstIndex=0;
     rosbag::View view_gps2(*bag2, rosbag::TopicQuery(curve2_topic));
     BOOST_FOREACH(rosbag::MessageInstance const m2,view_gps2) {
     	
     if(m2.isType<sensor_msgs::NavSatFix>() ) {
      sensor_msgs::NavSatFixConstPtr fixptr2 = m2.instantiate<sensor_msgs::NavSatFix>();
      callback_gps(fixptr2,gps_flag);
      }
     }
}
