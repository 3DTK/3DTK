#ifndef _TF_BROADCASTER_H_
#define _TF_BROADCASTER_H_

#include <ros/ros.h>
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <boost/foreach.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <vector>
#include <math.h>
#include "sensor_msgs/LaserScan.h"

using namespace std;
using namespace Eigen;

extern double time_s[60000];
extern std::vector<MatrixXd> matrix_new;
//extern unsigned int num;

using namespace std;
using namespace Eigen;
#define foreach BOOST_FOREACH

class tf_broadcaster {
  rosbag::Bag bag;
  tf::TransformBroadcaster br;
  geometry_msgs::TransformStamped trans;
  
  
  public:
 // tf_broadcaster(std::vector<MatrixXd> &poses_result);
  int broadcaster_tf();
  //void readtf(std::vector<MatrixXd> &poses_result);
  
};

#endif

