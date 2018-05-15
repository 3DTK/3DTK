#ifndef _TIMESTAMPS_H_
#define _TIMESTAMPS_H_

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
//#include <fstream>
//#include <iostream>
#include <string.h>
#include <string>
//#include "timestamps/conversions.h"
#include <geometry_msgs/PoseStamped.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/NavSatStatus.h>
#include <sensor_msgs/NavSatFix.h>
//#include <bits/stringfwd.h>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <Eigen/Eigen>
#include<vector>
#include<map>
#include <math.h>
#define M_PI       3.14159265358979323846

class timestamps {
  rosbag::Bag *bag;
  std::string hector_topic;
  std::string gps_topic;
 // std::map<double, geometry_msgs::PoseStamped> fusion_timetable;
  public:
  timestamps(rosbag::Bag *_bag) {

    ros::NodeHandle node;
    bag=_bag;
    hector_topic        = "/slam_out_pose";
    gps_topic		= "/fix";
   // ros::NodeHandle priv_node("~");
   // string dir_bag,dir_timestamp;
   // char *FileName;

    //priv_node.param<std::string>("dir_bag", dir_bag, "/home/du/Documents/datafusion/hectorgps212.bag");
    //priv_node.param<std::string>("dir_timestamp", dir_timestamp, "/home/du/Documents/datafusion/trajectory/");

  }

  void extractTrajectory(std::vector<double> *point_s);
  void callback_gps(const sensor_msgs::NavSatFixConstPtr &fix);
  void callback_hector(const geometry_msgs::PoseStampedConstPtr &hectorpose);
};

#endif
