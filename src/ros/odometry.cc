#include <ros/ros.h>
#include "stdio.h"

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include "sys/time.h"

#include <unistd.h>
#include <fcntl.h>
#include <stdexcept>

// messages
#include "volksbotRT/ticks.h"
#include "volksbotRT/pose2d.h"

#include "ros/odometry.h"

//FILE *file;

namespace volksbotRT {
// -461.817 ticks to cm
//const double Odometry::M  = -1.0/461.817;
// 44.4 cm is the wheel base
//const double Odometry::B = 44.4;

const double Odometry::covariance[36] = { 0.01, 0, 0, 0, 0, 0,
                                          0, 0.01, 0, 0, 0, 0,
                                          0, 0, 99999, 0, 0, 0,
                                          0, 0, 0, 0.174532925, 0, 0,
                                          0, 0, 0, 0, 9999, 0,
                                          0, 0, 0, 0, 0, 99999};
ros::Time old;
void Odometry::convertTicks2Odom(const ticksConstPtr& cticks)
{
  if (firstticks) {
    oldlticks = cticks->left;
    oldrticks = cticks->right;
//    oldticks = *cticks;
    firstticks = false;
    return;
  }

  ros::Time current = cticks->header.stamp;

//  int left = cticks->left - oldticks.left;
//  int right = cticks->right - oldticks.right;
//  long difftime = cticks->timestamp - oldticks.timestamp;
//  oldticks = *cticks;

  int left = cticks->left - oldlticks;
  int right = cticks->right - oldrticks;
  oldlticks = cticks->left;
  oldrticks = cticks->right;

  ros::Duration dt = current - old;
  old  = current;


  double ddt = dt.toSec();

  /////////////////
  //double vl = M*left / ddt;
  //double vr = M*right / ddt;
  //fprintf(file, "%f %f %f   %f %d %d \n", current.toSec(), vl*-0.01, vr*-0.01,  ddt, cticks->left, cticks->right );



  double vth = (M*right - M*left) / B;                  // delta theta in radian
  double vx  = ((M*right + M*left)/2.0)/100.0;                // delta x in m

  theta += vth;

  x += ((M*right + M*left)/2.0) * cos(theta);
  z -= ((M*right + M*left)/2.0) * sin(theta);

  //since all odometry is 6DOF we'll need a quaternion created from yaw
  odom_quat = tf::createQuaternionMsgFromYaw(-theta);

  //next, we'll publish the odometry message over ROS
  //odom.header.stamp = cticks->timestamp;
  odom.header.stamp= current;// no timestamp data TODO
  //set the position
  odom.pose.pose.position.x = x/100.0;
  odom.pose.pose.position.y = z/100.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;

  for (int i = 0; i < 36; i++)
    odom.pose.covariance[i] = covariance[i];

  double ax = (lastvx-(vx/ddt))/ddt;   // acceleration in cm/s^2
  //double ath = (lastvth-(vth/ddt))/ddt;  // acceleration in rad/s^2
  ax *= 0.01;   // m/s^2
  //set the velocity only if reasonably accurate

  if (ddt > 0 && fabs(ax) < 5.0) {
//  fprintf(file, "%f %f %f   %f %d %d  %f %f\n", current.toSec(), vl*-0.01, vr*-0.01,  ddt, cticks->left, cticks->right, (lastvx-(vx/ddt))/ddt, (lastvth-(vth/ddt))/ddt );
//    fprintf(file, "%f %f %f   %f %f\n", current.toSec(), cticks->vx, cticks->vth, vx/ddt, -vth/ddt );
    lastvx = vx/ddt;
    lastvth = vth/ddt;
    odom.twist.twist.linear.x = vx/ddt;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = -vth/ddt;
    for (int i = 0; i < 36; i++)
      odom.twist.covariance[i] = covariance[i];
  } else {    /// velocities cant be computed, use old values
    for (int i = 0; i < 36; i++)
      odom.twist.covariance[i] = 99999.9;
  }

  // send odometry message
  publisher.publish(odom);
}


Odometry::~Odometry() {
//fclose(file);
}

Odometry::Odometry() {
//file = fopen("/tmp/ist.txt", "w");

  x = 0;
  z = 0;
  theta = 0;
  lastvx = 0;
  lastvth = 0;

  publisher = n.advertise<nav_msgs::Odometry>("odom", 100);
  subscriber = n.subscribe("VMC", 20, &Odometry::convertTicks2Odom, this);
  firstticks = true;

  // message frames
  odom.header.frame_id = "/odom";
  odom.child_frame_id = "/base_link";

  odom_quat = tf::createQuaternionMsgFromYaw(0.0);

  odom.pose.pose.position.x = 0.0;
  odom.pose.pose.position.y = 0.0;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
  for (int i = 0; i < 36; i++)
    odom.pose.covariance[i] = covariance[i];
  //set the velocity
  odom.twist.twist.linear.x = 0;
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;
  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = 0;
  for (int i = 0; i < 36; i++)
    odom.twist.covariance[i] = covariance[i];

  n.param("/odometry/wheel_base", B, 44.4);
  n.param("/odometry/ticks_per_cm", M, -461.817);
  M = 1.0 / M;
}

void Odometry::update(int rate) {
  ros::Rate loop_rate(rate);

  while (ros::ok()) {
    ros::Time current = ros::Time::now();
    odom.header.stamp= current;// no timestamp data TODO

    publisher.publish(odom);

    ros::spinOnce();
    loop_rate.sleep();
  }

}

}
