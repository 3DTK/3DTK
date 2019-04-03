#ifndef _EXTRACTAJECTORY_H_
#define _EXTRACTAJECTORY_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <string.h>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include "riegl/scanlib.hpp"
#include <Eigen/Eigen>
#include<vector>
#include<map>
#include <math.h>
#define M_PI       3.14159265358979323846

using namespace scanlib;
extern int firstIndex;
class extractTrajectory {

  public:
  rosbag::Bag *bag1;
  rosbag::Bag *bag2;
  std::string curve1_topic;
  std::string curve2_topic;
 // unsigned int seq_h;
 // unsigned int seq_g;
  std::map<double, geometry_msgs::PoseStamped> curve1_timetable;
  std::map<double, geometry_msgs::PoseStamped> curve2_timetable;
  Eigen::Affine3d startTransform ;
  int startIndex;
  public:
  extractTrajectory(rosbag::Bag *_bag1,rosbag::Bag *_bag2) {
    startIndex=0;
    ros::NodeHandle node;
    bag1=_bag1;
    bag2=_bag2;
    curve1_topic        = "/slam_out_pose";
    curve2_topic	= "/fix";
    startTransform = Eigen::Affine3d::Identity();
   // ros::NodeHandle priv_node("~");
   // string dir_bag,dir_timestamp;
   // char *FileName;
 
    //priv_node.param<std::string>("dir_bag", dir_bag, "/home/du/Documents/datafusion/hectorgps212.bag");
    //priv_node.param<std::string>("dir_timestamp", dir_timestamp, "/home/du/Documents/datafusion/trajectory/");

  }
  

  void callback_gps(const sensor_msgs::NavSatFixConstPtr &fix,int gps_flag) {


     if (fix->status.status == sensor_msgs::NavSatStatus::STATUS_NO_FIX) {
         ROS_INFO("No fix.");
        return;
         }

       if (fix->header.stamp == ros::Time(0)) {
        return;
        }
       //if (fix->header.stamp.toSec()<1553017625)
       // return;
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
        Eigen::Affine3d ecef2ned = Eigen::Affine3d(Eigen::AngleAxisd(-0.5 * M_PI, Eigen::Vector3d(0, 1, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(lon, Eigen::Vector3d(1, 0, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(-lat, Eigen::Vector3d(0, 1, 0)));
        
    
//
        // Transform NED frame to ROS frame (x front, y left, z up)
        Eigen::Affine3d ned2nwu = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI, Eigen::Vector3d(1, 0, 0)));
        // When the robot start to run,if x-axis point south direction,use the following Line of code
//#ifndef GPS_HECTOR
        Eigen::Affine3d nwu2ros = Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1)));
//#else
       // Eigen::Affine3d nwu2ros = Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1)));
//#endif
        // When the robot start to run,if x-axis point east direction,use the following Line of code
       // Eigen::Affine3d nwu2ros = Eigen::Affine3d(Eigen::AngleAxisd(0.2* M_PI, Eigen::Vector3d(0, 0, 1)));
        // Local rotation from inclination sensors
        Eigen::Affine3d rotation = Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(1, 0, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 1, 0))) *
                Eigen::Affine3d(Eigen::AngleAxisd(0, Eigen::Vector3d(0, 0, 1)));

       Eigen::Affine3d transform = ((translation * ecef2ned) *ned2nwu*nwu2ros) * rotation;
        //  Eigen::Affine3d transform = ((translation * ecef2ned) ); 
        // Use relative pose to ROS frame of pose at startIndex
        if (firstIndex<1) {
            startTransform = ((translation * ecef2ned)*ned2nwu*nwu2ros);
            firstIndex++;
        }

        Eigen::Affine3d posee = startTransform.inverse() * transform;

        // Write pose file
               
        pose.pose.position.x = (posee.translation()(0));
        pose.pose.position.y = (posee.translation()(1));
        pose.pose.position.z = (posee.translation()(2));
  

        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        
        if(gps_flag==1){
        curve1_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(pose.header.stamp.toSec(),pose));
        }
        else if(gps_flag==2)
        curve2_timetable.insert(std::pair<double, geometry_msgs::PoseStamped>(pose.header.stamp.toSec(),pose));
  } 

 // virtual void aligncurve(std::vector<double> *point_g,std::vector<double> *hector_matrix ,unsigned int *seq_interval,std::string dir);
  //virtual void callback_gps(const sensor_msgs::NavSatFixConstPtr &fix);
 // virtual void callback_curve2(const geometry_msgs::PoseStampedConstPtr &hectorpose);
       
};

class extract2hector_gps : public extractTrajectory {
  
  public:
   extract2hector_gps(rosbag::Bag *_bag1);
   extract2hector_gps(rosbag::Bag *_bag1,rosbag::Bag *_bag2); 
  // extract2hector_gps();
   void aligncurve(std::vector<double> *point_curve1,std::vector<double> *point_curve2,std::vector<double> *hector_matrix,std::string dir);
  // void aligncurve(std::vector<double> *hector_matrix,std::vector<double> *point_corr,std::string dir);
  public:
   rosbag::Bag *bagod;
   std::map<double, geometry_msgs::PoseStamped> hector_timetable;

};


class GPS : public pointcloud
{
public:
    struct GpsInfo {
        GpsInfo() :
            timestamp(0),
            lat(0),
            lon(0),
            alt(0),
            accuracy(numeric_limits<double>::max())
        {}

        double timestamp; // gps time of week in sec
        double lat; // latitude in rad
        double lon; // longitude in rad
        double alt; // altitude above ellipsoid in m
        double accuracy; // position accuracy in m
    };

    struct InclInfo {
        InclInfo() :
            roll(0),
            pitch(0)
        {}

        double roll; // rad
        double pitch; // rad
    };

public:
    GPS()
        : pointcloud(false)
    {}
protected:
    void on_hk_gps_hr(const hk_gps_hr<iterator_type>& arg) {
        pointcloud::on_hk_gps_hr(arg);

        GpsInfo info;
        info.timestamp = (double) arg.TOWms / 1000.0;
        info.lat = (double) arg.LAT  *1e-9* M_PI / 180;
        info.lon = (double) arg.LONG  *1e-9* M_PI / 180;
        info.alt = (double) arg.HGT_ELL*1e-3 ;
        info.accuracy = (double) arg.POS_ACC * 1e-3;

        _gpsInfo.push_back(info);
    }

    void on_hk_incl(const hk_incl<iterator_type>& arg) {
        pointcloud::on_hk_incl(arg);

        double roll = (double) arg.ROLL * 1e-3 * M_PI / 180;
        double pitch = (double) arg.PITCH * 1e-3 * M_PI / 180;

        _rollVector.push_back(roll);
        _pitchVector.push_back(pitch);
    }

public:
    vector<GpsInfo> _gpsInfo;
    vector<double> _rollVector;
    vector<double> _pitchVector;
};



class extract2curve1_curve2 : public extractTrajectory {

   public:
   extract2curve1_curve2(rosbag::Bag *_bag,std::string &_rieglfile);
   extract2curve1_curve2(rosbag::Bag *_bag,std::string &_filename,int _flag); 
   extract2curve1_curve2(rosbag::Bag *_bag1,rosbag::Bag *_bag2);
   void aligncurve(std::vector<double> *point_g_ros,std::vector<double> *point_g_riegl,std::string dir);
  public:
   //std::map<double, geometry_msgs::PoseStamped> curve2_timetable;
   string filename;
   int flag;
};


#endif
