#ifndef ODOMETRY_HH
#define ODOMETRY_HH
#include <ros/ros.h>

// messages
#include "volksbotRT/ticks.h"
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

namespace volksbotRT {

  class Odometry {
    private:

      ros::NodeHandle n;
      ros::Publisher publisher;
      tf::TransformBroadcaster odom_broadcaster;
      ros::Subscriber subscriber;
      bool firstticks;

      double x,z,theta;

      double lastvx, lastvth;

      int oldlticks, oldrticks;


      // -461.817 ticks to cm
      double M;
      // 44.4 cm is the wheel base
      double B;

      static const double covariance[36];


      nav_msgs::Odometry odom;

      geometry_msgs::Quaternion odom_quat; //quaternion rotation

    public:
      Odometry();
      ~Odometry();

      void setTicks(double m) {
        M = 1.0 / m;
      }

      void setWheelBase(double b) {
        B = b;
      }

      void convertTicks2Odom(const ticksConstPtr& cticks);

      const nav_msgs::Odometry& getCurrentOdom() {
        return odom;
      }

      void update(int ms);


  };
}

#endif
