#ifndef __EKF_FILTER_H__
#define __EKF_FILTER_H__

#include "my_robot_pose_ekf/odom_estimation.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

#include <tf/tf.h>    
#include <tf/transform_listener.h>

class ekffilter {
  public:

  ekffilter(tf::TransformListener *l):
     odom_covariance_(6),
      imu_covariance_(3)
  {
    listener = l;
  }

  /// callback function for odo data
  void addOdom(const nav_msgs::Odometry& odom);
  
  /// callback function for imu data
  void addImu(const sensor_msgs::Imu& imu);


  private:

  void updateHistory();

  // ekf filter
  estimation::OdomEstimation my_filter_;

  // estimated robot pose message to send
//  geometry_msgs::PoseWithCovarianceStamped  output_;

  tf::TransformListener *listener;

  ros::Time odom_stamp_, imu_stamp_, filter_stamp_;
  tf::Transform odom_meas_, imu_meas_;
  tf::StampedTransform base_imu_offset;
  MatrixWrapper::SymmetricMatrix odom_covariance_, imu_covariance_;

  tf::Quaternion q;
  tf::Quaternion orientation;

};

#endif
