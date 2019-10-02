#include "ros/icp6Dwrapper.h"

#include <slam6d/icp6D.h>
#include <slam6d/icp6Dsvd.h>
#include <slam6d/icp6Dquat.h>
#include <slam6d/icp6Dapx.h>
#include <slam6d/icp6Dortho.h>
#include <slam6d/icp6Dhelix.h>
#include <slam6d/icp6Ddual.h>
#include <slam6d/icp6Dlumeuler.h>
#include <slam6d/icp6Dlumquat.h>
#include <slam6d/icp6Dquatscale.h>
#include <slam6d/icp6Dnapx.h>
#include <slam6d/scan.h>
#include <slam6d/basicScan.h>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <geometry_msgs/TransformStamped.h>


Icp6DWrapper::~Icp6DWrapper() {
  delete l;
  delete my_icp6Dminimizer;
  my_icp6Dminimizer = 0;
  delete icp;
  icp = 0;
}

Icp6DWrapper::Icp6DWrapper() {
  l = new tf::TransformListener(nh, ros::Duration(100.0));

  ros::NodeHandle nh_("~");

  // TODO: make them available from command line
  nh_.param<double>("maximal_distance_match", mdm, 75);
  ROS_INFO("maximal_distance_match: %f",mdm);
  nh_.param<int>("max_iterations_icp", iters, 100);
  ROS_INFO("maximal iterations: %d",iters);
  nh_.param<double>("min_epsilon_ICP", epsICP, 0.00001);
  nh_.param<int>("random_subset_ptpairs", rnd, 1);
  nh_.param<double>("maximal_sensor_range",maxDist,-1);
  nh_.param<double>("minimal_sensor_range",minDist,-1);

  nh_.param<std::string>("sensor_topic",sensor_topic,"/pointcloud");
  nh_.param<std::string>("sensor_frame",sensor_frame,"base_link");	// TODO
  nh_.param<std::string>("base_frame",base_frame,"base_link");
  nh_.param<std::string>("odom_frame",odom_frame,"odom");
  nh_.param<std::string>("map_frame",map_frame,"map");
  nns_method = BOCTree;

  nh_.param<double>("reduction_voxelsize", red, 10.0);
  nh_.param<int>("reduction_points_per_voxel", ppvoxel, 1);
  int algo = 1;
  nh_.param<int>("algorithm_icp", algo, 1);
  my_icp6Dminimizer = 0;
  bool quiet = true;
  switch (algo) {
  case 1 :
    my_icp6Dminimizer = new icp6D_QUAT(quiet);
    break;
  case 2 :
    my_icp6Dminimizer = new icp6D_SVD(quiet);
    break;
  case 3 :
    my_icp6Dminimizer = new icp6D_ORTHO(quiet);
    break;
  case 4 :
    my_icp6Dminimizer = new icp6D_DUAL(quiet);
    break;
  case 5 :
    my_icp6Dminimizer = new icp6D_HELIX(quiet);
    break;
  case 6 :
    my_icp6Dminimizer = new icp6D_APX(quiet);
    break;
  case 7 :
    my_icp6Dminimizer = new icp6D_LUMEULER(quiet);
    break;
  case 8 :
    my_icp6Dminimizer = new icp6D_LUMQUAT(quiet);
    break;
  case 9 :
    my_icp6Dminimizer = new icp6D_QUAT_SCALE(quiet);
    break;
  case 10 :
    my_icp6Dminimizer = new icp6D_NAPX(quiet);
    break;
  }
  icp = new icp6D(my_icp6Dminimizer, mdm, iters, true, false, rnd, true, 0, epsICP, false, false);

  map_odom.setIdentity();
  map_odom.stamp_ = ros::Time::now();
  map_odom.child_frame_id_ = odom_frame;
  map_odom.frame_id_ = map_frame;

  subPCL2 = nh.subscribe(sensor_topic, 100, &Icp6DWrapper::callbackPointCloud, this);
//  pubMapOdom = nh.advertise<nav_msgs::Odometry>("map_to_odom",1000);

}

void Icp6DWrapper::callbackPointCloud(const sensor_msgs::PointCloud2ConstPtr& msg) {
  ROS_INFO("====================================");
  ROS_INFO("RECEIVING scan %d",msg->header.seq);
  tf::StampedTransform transform;
  ros::Time rostime = msg->header.stamp;
  try {
    //l->waitForTransform(GLOBAL_FRAME, KINECT_FRAME, rostime, ros::Duration(30.0)); // FIXME
    l->lookupTransform (odom_frame, sensor_frame, rostime, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Transforming Pointcloud failed!  %s", ex.what());
    return;
  }

  double x,y,z;
  transform.getBasis().getEulerYPR(x,y,z);
  ROS_INFO("SCAN IN POSE: %lf %lf %lf, %lf %lf %lf",
  		  transform.getOrigin().getX(), transform.getOrigin().getY(),
		  transform.getOrigin().getZ(),x*180/M_PI,y*180/M_PI,z*180/M_PI);

  double transMat[16];
  ros2frames(transform, transMat);
  double rPos[3], rPosTheta[4];
  Matrix4ToEuler(transMat, rPosTheta, rPos);

  vector<double*> data;
  pointcloud2scan(msg,data);
  Scan *scan = new BasicScan(rPos, rPosTheta, data);

  scan->setRangeFilter(maxDist, minDist);
  scan->setSearchTreeParameter(nns_method);
  scan->createSearchTree();

  // FIXME throw old data away, nicer
  if (scans.size() > 1) {
    delete scans[0];
    scans.erase(scans.begin());
  }

  // add new scan
  scans.push_back(scan);

  // do ICP
  if (scans.size() > 1) {
    icp->doICP(scans);
    tf::StampedTransform transformnew;
    frames2ros(scan->get_transMat(),transformnew);

    transformnew.getBasis().getEulerYPR(x,y,z);
//	ROS_INFO_STREAM("Scan out Mat: " << scan->get_transMat());
	ROS_INFO("SCAN OUT POSE: %lf %lf %lf %lf %lf %lf",
			transformnew.getOrigin().getX(),
			transformnew.getOrigin().getY(),
			transformnew.getOrigin().getZ() ,deg(x),deg(y),deg(z));

    tf::Transform scan_to_map = transform.inverse();
    //tf::Transform scannew_to_map = transformnew.inverse();

    tf::StampedTransform map_odom = tf::StampedTransform(
		transformnew * scan_to_map,
		transform.stamp_,
		map_frame, odom_frame);

//	ROS_INFO_STREAM("T_maptoodom: "  << transformToString(map_odom));

/*	nav_msgs::Odometry odom_corr;
	odom_corr.header.stamp = msg->header.stamp;
	odom_corr.header.seq   = msg->header.seq;
	odom_corr.header.frame_id = "map";
	odom_corr.child_frame_id  = "odom";
	tf::poseTFToMsg(map_odom,odom_corr.pose.pose);*/

    b.sendTransform(map_odom);
/* 	pubMapOdom.publish(odom_corr); */

  } else {
	map_odom.setIdentity();
	map_odom.stamp_ = msg->header.stamp;
	map_odom.frame_id_ = map_frame;
	map_odom.child_frame_id_ = odom_frame;
	b.sendTransform(map_odom);
  }


}

void Icp6DWrapper::spin() {

  ros::Rate loop_rate(10);
  while (ros::ok()) {

//    map_odom.stamp_ = ros::Time::now();
//  	b.sendTransform(map_odom);
    loop_rate.sleep();
    ros::spinOnce();
  }

}


