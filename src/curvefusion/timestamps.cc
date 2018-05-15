/* Program:
 * This program is written for finding correspondence point by timestamps from different sensors.
 * History:
 * 3/05/2018	
 * Copyright (C) Shitong Du
 */

#include "curvefusion/timestamps.h"
#include <boost/foreach.hpp>

using namespace std;  
std::map<double, geometry_msgs::PoseStamped> hector_timetable;
std::map<double, geometry_msgs::PoseStamped> gps_timetable;

int startIndex=0;
Eigen::Affine3d startTransform = Eigen::Affine3d::Identity();
// Eigen::Affine3d ecef2ned;
double ecef2ned1[3],ecef2nedfirst[3],Rota[9];
double pos_ned[3];
double Rotainti[9];
double yaw=240*M_PI/180;
double pos_rota[3];
//geometry_msgs::PoseStamped pose_min0;
void timestamps::callback_gps(const sensor_msgs::NavSatFixConstPtr &fix) {

        if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
         ROS_INFO("No fix.");
        return;
         }

       if (fix->header.stamp == ros::Time(0)) {
        return;
        }
        geometry_msgs::PoseStamped pose;
        pose.header = fix->header;

        double lat, lon , alt;
        lat=(fix->latitude)*M_PI/180;
        lon=(fix->longitude)*M_PI/180;
        alt=(fix->altitude);

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
		// compute rotation only for the first GPS point, to define the coordinate system
		/*
			ecef2ned;	// global variables
			ned2ros;
			rotation;

			if startIndex<0
				ecef2ned=affine3d(lat,lon)
				ned2ros =affine3d...
				rotation = affine3d...
			
			transform=translation*ecef2ned*ned2rosi
				
				
		*/
		

        // Transform NED frame to ROS frame (x front, y left, z up)
        Eigen::Affine3d ned2ros = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d(1, 0, 0)));

        // Local rotation from inclination sensors
        //Eigen::Affine3d rotation = Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(1, 0, 0))) *
          //      Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))) *
          //      Eigen::Affine3d(Eigen::AngleAxisd(1.04, Eigen::Vector3d(0, 0, 1)));

        

        // Use relative pose to ROS frame of pose at startIndex
        if (startIndex<1) {

           // ecef2ned = Eigen::Affine3d(Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d(0, 1, 0))) *
           // Eigen::Affine3d(Eigen::AngleAxisd(lon, Eigen::Vector3d(1, 0, 0))) *
           // Eigen::Affine3d(Eigen::AngleAxisd(-lat, Eigen::Vector3d(0, 1, 0)));
           Rota[0]=(-sin(lat))*cos(lon);
           Rota[1]=-sin(lat)*sin(lon);
           Rota[2]=cos(lat);
           Rota[3]=-sin(lon);
           Rota[4]=cos(lon);
           Rota[5]=0;
           Rota[6]=-cos(lat)*cos(lon);
           Rota[7]=-cos(lat)*sin(lon);
           Rota[8]=-sin(lat);
            ecef2nedfirst[0]=Rota[0]*xe+Rota[1]*ye+Rota[2]*ze;
            ecef2nedfirst[1]=Rota[3]*xe+Rota[4]*ye;
            ecef2nedfirst[2]=Rota[6]*xe+Rota[7]*ye-Rota[8]*ze; 
            //startTransform = ((ecef2ned1));

            startIndex++;
        }

        ecef2ned1[0]=Rota[0]*xe+Rota[1]*ye+Rota[2]*ze;
        ecef2ned1[1]=Rota[3]*xe+Rota[4]*ye;
        ecef2ned1[2]=Rota[6]*xe+Rota[7]*ye-Rota[8]*ze; 
        //Eigen::Affine3d transform = ((translation * ecef2ned));
        
        pos_ned[0]=ecef2ned1[0]-ecef2nedfirst[0];
        pos_ned[1]=ecef2ned1[1]-ecef2nedfirst[1];
        pos_ned[2]=ecef2ned1[2]-ecef2nedfirst[2];

       /* Rotainti[0]=cos(yaw);
        Rotainti[1]=sin(yaw);
        Rotainti[2]=0;
        Rotainti[3]=-sin(yaw);
        Rotainti[4]=cos(yaw);
        Rotainti[5]=0;
        Rotainti[6]=0;
        Rotainti[7]=0; 
        Rotainti[7]=1.0; */
       // Eigen::Affine3d poseg = startTransform.inverse() * transform;
        // TODO: place your coordinate conversion here
       // LLtoUTM(fix->latitude, fix->longitude, northing, easting, zone);
	//Eigen::Affine3d poseg = ((translation * ecef2ned) * ned2ros);
	// pose.pose.position.x = ...
       // pos_rota[0]=Rotainti[0]*pos_ned[1]+Rotainti[1]*pos_ned[0];
        //pos_rota[1]=Rotainti[3]*pos_ned[1]+Rotainti[4]*pos_ned[0];
       // pos_rota[2]=-pos_ned[2];

        pose.pose.position.x = -pos_ned[0];
        pose.pose.position.y = pos_ned[1];
        pose.pose.position.z = -pos_ned[2];
    
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;

        gps_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(pose.header.stamp.toSec(),pose));
				
}

void timestamps::callback_hector(const geometry_msgs::PoseStampedConstPtr &hectorpose) {

	geometry_msgs::PoseStamped pose;
	pose.header	= hectorpose->header;
	pose.pose	= hectorpose->pose;

	hector_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(pose.header.stamp.toSec(),pose));
				
}


void timestamps::extractTrajectory(std::vector<double> *point_s) {
  rosbag::View view_hector(*bag, rosbag::TopicQuery(hector_topic));
  BOOST_FOREACH(rosbag::MessageInstance const m, view_hector) {
   
    if(m.isType<geometry_msgs::PoseStamped>() ) {
      geometry_msgs::PoseStampedConstPtr poseptr = m.instantiate<geometry_msgs::PoseStamped>();
      callback_hector(poseptr);
    }
  }
   
  rosbag::View view_gps(*bag, rosbag::TopicQuery(gps_topic));
  BOOST_FOREACH(rosbag::MessageInstance const m,view_gps) {
		
    if(m.isType<sensor_msgs::NavSatFix>() ) {
      sensor_msgs::NavSatFixConstPtr fixptr = m.instantiate<sensor_msgs::NavSatFix>();
      callback_gps(fixptr);
     }
  }

// now the timetables are set up and you can look for corresponding stamps.
// since the gps topic seems to have a lower frequency than the hector topic,
// look for them in the hector timetable

   std::map<double, geometry_msgs::PoseStamped>::iterator gps;
   //std::map<double, geometry_msgs::PoseStamped>::iterator gps0;
   int i=0;             
   for(gps=gps_timetable.begin();gps!=gps_timetable.end();++gps){
	//for(auto&gps : gps_timetable) {
     
     double stamp = gps->first;
     geometry_msgs::PoseStamped pose = gps->second;       
     double dt_min = 1000.0;
     geometry_msgs::PoseStamped pose_min;
                
// for each gps pose look for the closest hecotr pose wrt time
//for(auto &hector : hector_timetable) {
     std::map<double, geometry_msgs::PoseStamped>::iterator hector;
    // std::map<double, geometry_msgs::PoseStamped>::iterator hector0;
     for(hector=hector_timetable.begin();hector!=hector_timetable.end();++hector){
     double dt = fabs(hector->first - stamp);
       if(dt < dt_min) {
	 pose_min = hector->second;
	 dt_min = dt;
       }
     }
            
              
// now pose and pose_min are a corresponding pair
// and you can write the poses to a file
// stamp gps.x gps.y gps.z hector.x ...

       //pose_out << pose.header.stamp << " ";
       
       point_s[0].push_back(pose_min.header.stamp.toSec()) ;
       point_s[1].push_back(pose.pose.position.x) ;
       point_s[2].push_back(pose.pose.position.y) ;
       point_s[3].push_back(pose.pose.position.z) ;
 
       point_s[4].push_back(pose_min.pose.position.x) ;
       point_s[5].push_back(pose_min.pose.position.y) ;
       point_s[6].push_back(pose_min.pose.position.z) ;
       //point[0] << endl; 
       i++;         
   }


}
