#include "ros/ekf_filter.h"
#include <iomanip>
#include <iostream>
#include <fstream>
#include <queue>
using std::queue;
#include <vector>
using std::vector;
#include <sstream>
using std::stringstream;
using std::string;
using std::cout;
using std::endl;
using std::ifstream;

void ekffilter::addOdom(const nav_msgs::Odometry& odom) {
  odom_stamp_ = odom.header.stamp;
  tf::quaternionMsgToTF(odom.pose.pose.orientation, q);
  odom_meas_  = tf::Transform(q, tf::Vector3(odom.pose.pose.position.x, odom.pose.pose.position.y, 0));
  for (unsigned int i=0; i<6; i++)
    for (unsigned int j=0; j<6; j++)
      odom_covariance_(i+1, j+1) = odom.pose.covariance[6*i+j];

  my_filter_.addMeasurement(tf::StampedTransform(odom_meas_.inverse(), odom_stamp_, "base_link", "wheelodom"), odom_covariance_);


  // initialize filer with odometry frame
  if ( !my_filter_.isInitialized()){
    my_filter_.initialize(odom_meas_, odom_stamp_);
    ROS_INFO("Kalman filter initialized with odom measurement");
  }
  updateHistory();
}


void ekffilter::addImu(const sensor_msgs::Imu& imu) {
  imu_stamp_ = imu.header.stamp;
  tf::quaternionMsgToTF(imu.orientation, orientation);
  imu_meas_ = tf::Transform(orientation, tf::Vector3(0,0,0));
  for (unsigned int i=0; i<3; i++)
    for (unsigned int j=0; j<3; j++)
      imu_covariance_(i+1, j+1) = imu.orientation_covariance[3*i+j];

  // Transforms imu data to base_link frame
  listener->lookupTransform("base_link", imu.header.frame_id, imu_stamp_, base_imu_offset);
  imu_meas_ = imu_meas_ * base_imu_offset;

  // manually set covariance untile imu sends covariance
  if (imu_covariance_(1,1) == 0.0){
    MatrixWrapper::SymmetricMatrix measNoiseImu_Cov(3);  measNoiseImu_Cov = 0;
    measNoiseImu_Cov(1,1) = pow(0.00017,2);  // = 0.01 degrees / sec
    measNoiseImu_Cov(2,2) = pow(0.00017,2);  // = 0.01 degrees / sec
    measNoiseImu_Cov(3,3) = pow(0.00017,2);  // = 0.01 degrees / sec
    imu_covariance_ = measNoiseImu_Cov;
  }

  my_filter_.addMeasurement(tf::StampedTransform(imu_meas_.inverse(), imu_stamp_, "base_link", "imu"), imu_covariance_);

  updateHistory();
}




void ekffilter::updateHistory() {
  // initial value for filter stamp; keep this stamp when no sensors are active
  filter_stamp_ = ros::Time(INT_MAX, INT_MAX);
  filter_stamp_ = std::min(filter_stamp_, odom_stamp_);
  filter_stamp_ = std::min(filter_stamp_, imu_stamp_);  // TODO check wether this will fail at the beginning

  // update filter
  if ( my_filter_.isInitialized() )  {
    bool diagnostics = true;
    if (my_filter_.update(true, true, false,  filter_stamp_, diagnostics)){

      // output most recent estimate and relative covariance
      //        my_filter_.getEstimate(output_);
      //          pose_pub_.publish(output_);
      //          ekf_sent_counter_++;

      // broadcast most recent estimate to TransformArray
      tf::StampedTransform tmp;
      my_filter_.getEstimate(ros::Time(), tmp);
      //         odom_broadcaster_.sendTransform(StampedTransform(tmp, tmp.stamp_, publish_name, "base_link"));
      // TODO
      listener->setTransform(tf::StampedTransform(tmp, tmp.stamp_, "odom_combined", "base_link"));
      //std::cout << tmp.stamp_ << std::endl;

    }
    if (!diagnostics)
      ROS_WARN("Robot pose ekf diagnostics discovered a potential problem");
  }


}
