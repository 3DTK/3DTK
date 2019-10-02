#include "ros/calibration.h"
#include "ros/importer.h"
#include "ros/ekf_filter.h"
#include "ros/powell.h"

#include <boost/foreach.hpp>
#include <nav_msgs/Odometry.h>
#include <volksbotRT/ticks.h>
#include <ros/odometry.h>

const unsigned int calibration::USE_NONE = 1;

const unsigned int calibration::USE_RIEGL_X  = 1;
const unsigned int calibration::USE_RIEGL_Y  = 2;
const unsigned int calibration::USE_RIEGL_Z  = 4;
const unsigned int calibration::USE_RIEGL_RX = 8;
const unsigned int calibration::USE_RIEGL_RY = 16;
const unsigned int calibration::USE_RIEGL_RZ = 32;

const unsigned int calibration::USE_SICK_X   = 64;
const unsigned int calibration::USE_SICK_Y   = 128;
const unsigned int calibration::USE_SICK_Z   = 256;
const unsigned int calibration::USE_SICK_RX  = 512;
const unsigned int calibration::USE_SICK_RY  = 1024;
const unsigned int calibration::USE_SICK_RZ  = 2048;

const unsigned int calibration::USE_XSENS_X  = 4096;
const unsigned int calibration::USE_XSENS_Y  = 8192;
const unsigned int calibration::USE_XSENS_Z  = 16384;
const unsigned int calibration::USE_XSENS_RX = 32768;
const unsigned int calibration::USE_XSENS_RY = 65536;
const unsigned int calibration::USE_XSENS_RZ = 131072;

const unsigned int calibration::USE_ODOM_WHEEL_BASE = 262144;
const unsigned int calibration::USE_ODOM_TICKS      = 524288;
const unsigned int calibration::USE_TIME_OFFSET     = 1048576;

void calibration::count(unsigned int flag, unsigned int &n) {
  n = 1; // start with 1 due to numerical recipies

  if (flag & USE_ODOM_WHEEL_BASE) n++;
  if (flag & USE_ODOM_TICKS) n++;
  if (flag & USE_TIME_OFFSET) n++;

  if (flag & USE_SICK_X   ) n++;
  if (flag & USE_SICK_Y   ) n++;
  if (flag & USE_SICK_Z   ) n++;
  if (flag & USE_SICK_RX  ) n++;
  if (flag & USE_SICK_RY  ) n++;
  if (flag & USE_SICK_RZ  ) n++;
  if (flag & USE_XSENS_X  ) n++;
  if (flag & USE_XSENS_Y  ) n++;
  if (flag & USE_XSENS_Z  ) n++;
  if (flag & USE_XSENS_RX ) n++;
  if (flag & USE_XSENS_RY ) n++;
  if (flag & USE_XSENS_RZ ) n++;

  if (flag & USE_RIEGL_X  ) n++;
  if (flag & USE_RIEGL_Y  ) n++;
  if (flag & USE_RIEGL_Z  ) n++;
  if (flag & USE_RIEGL_RX ) n++;
  if (flag & USE_RIEGL_RY ) n++;
  if (flag & USE_RIEGL_RZ ) n++;
}

void calibration::toX(float *X) {
  unsigned int n = 1;
  if (params & USE_ODOM_WHEEL_BASE) X[n++] = wheel_base;
  if (params & USE_ODOM_TICKS) X[n++] = ticks_per_cm;
  if (params & USE_TIME_OFFSET) X[n++] = time_offset;

  if (params & USE_SICK_X   ) X[n++] = sick_pos[0];
  if (params & USE_SICK_Y   ) X[n++] = sick_pos[1];
  if (params & USE_SICK_Z   ) X[n++] = sick_pos[2];
  if (params & USE_SICK_RX  ) X[n++] = sick_euler[0];
  if (params & USE_SICK_RY  ) X[n++] = sick_euler[1];
  if (params & USE_SICK_RZ  ) X[n++] = sick_euler[2];
  if (params & USE_XSENS_X  ) X[n++] = xsens_pos[0];
  if (params & USE_XSENS_Y  ) X[n++] = xsens_pos[1];
  if (params & USE_XSENS_Z  ) X[n++] = xsens_pos[2];
  if (params & USE_XSENS_RX ) X[n++] = xsens_euler[0];
  if (params & USE_XSENS_RY ) X[n++] = xsens_euler[1];
  if (params & USE_XSENS_RZ ) X[n++] = xsens_euler[2];

  if (params & USE_RIEGL_X  ) X[n++] = riegl_pos[0];
  if (params & USE_RIEGL_Y  ) X[n++] = riegl_pos[1];
  if (params & USE_RIEGL_Z  ) X[n++] = riegl_pos[2];
  if (params & USE_RIEGL_RX ) X[n++] = riegl_euler[0];
  if (params & USE_RIEGL_RY ) X[n++] = riegl_euler[1];
  if (params & USE_RIEGL_RZ ) X[n++] = riegl_euler[2];
}

void calibration::printHeaderToFile(const char *filename) {
  ofstream of(filename, ios_base::app);
  of << "# Using:";
  if (params & USE_ODOM_WHEEL_BASE) of << " ODOM_WHEEL_BASE";
  if (params & USE_ODOM_TICKS)      of << " ODOM_TICKS";
  if (params & USE_TIME_OFFSET)     of << " TIME_OFFSET";

  if (params & USE_SICK_X   ) of << " SICK_X";
  if (params & USE_SICK_Y   ) of << " SICK_Y";
  if (params & USE_SICK_Z   ) of << " SICK_Z";
  if (params & USE_SICK_RX  ) of << " SICK_RX";
  if (params & USE_SICK_RY  ) of << " SICK_RY";
  if (params & USE_SICK_RZ  ) of << " SICK_RZ";
  if (params & USE_XSENS_X  ) of << " XSENS_X";
  if (params & USE_XSENS_Y  ) of << " XSENS_Y";
  if (params & USE_XSENS_Z  ) of << " XSENS_Z";
  if (params & USE_XSENS_RX ) of << " XSENS_RX";
  if (params & USE_XSENS_RY ) of << " XSENS_RY";
  if (params & USE_XSENS_RZ ) of << " XSENS_RZ";

  if (params & USE_RIEGL_X  ) of << " RIEGL_X";
  if (params & USE_RIEGL_Y  ) of << " RIEGL_Y";
  if (params & USE_RIEGL_Z  ) of << " RIEGL_Z";
  if (params & USE_RIEGL_RX ) of << " RIEGL_RX";
  if (params & USE_RIEGL_RY ) of << " RIEGL_RY";
  if (params & USE_RIEGL_RZ ) of << " RIEGL_RZ";


  of << endl;

  of.flush();
  of.close();
}

void calibration::fromX(float *X) {
  unsigned int n = 1;

  if (params & USE_ODOM_WHEEL_BASE) wheel_base = X[n++] ;
  if (params & USE_ODOM_TICKS) ticks_per_cm = X[n++] ;
  if (params & USE_TIME_OFFSET) time_offset = X[n++] ;

  if (params & USE_SICK_X   ) sick_pos[0] = X[n++] ;
  if (params & USE_SICK_Y   ) sick_pos[1] = X[n++] ;
  if (params & USE_SICK_Z   ) sick_pos[2] = X[n++] ;
  if (params & USE_SICK_RX  ) sick_euler[0] = X[n++] ;
  if (params & USE_SICK_RY  ) sick_euler[1] = X[n++] ;
  if (params & USE_SICK_RZ  ) sick_euler[2] = X[n++] ;
  if (params & USE_XSENS_X  ) xsens_pos[0] = X[n++] ;
  if (params & USE_XSENS_Y  ) xsens_pos[1] = X[n++] ;
  if (params & USE_XSENS_Z  ) xsens_pos[2] = X[n++] ;
  if (params & USE_XSENS_RX ) xsens_euler[0] = X[n++] ;
  if (params & USE_XSENS_RY ) xsens_euler[1] = X[n++] ;
  if (params & USE_XSENS_RZ ) xsens_euler[2] = X[n++] ;

  if (params & USE_RIEGL_X  ) riegl_pos[0] = X[n++] ;
  if (params & USE_RIEGL_Y  ) riegl_pos[1] = X[n++] ;
  if (params & USE_RIEGL_Z  ) riegl_pos[2] = X[n++] ;
  if (params & USE_RIEGL_RX ) riegl_euler[0] = X[n++] ;
  if (params & USE_RIEGL_RY ) riegl_euler[1] = X[n++] ;
  if (params & USE_RIEGL_RZ ) riegl_euler[2] = X[n++] ;
}

void calibration::extractTrajectory() {
  rosbag::View tfview(*bag, rosbag::TopicQuery("/tf"));

  tf::TransformListener *l = new tf::TransformListener( tfview.getEndTime() - tfview.getBeginTime(), false);

  BOOST_FOREACH(rosbag::MessageInstance const m, tfview)
  {
    if ( m.isType<tf::tfMessage>() ) {
      tf::tfMessageConstPtr tfm = m.instantiate<tf::tfMessage>();
      for (unsigned int i = 0; i < tfm->transforms.size(); i++) {
        tf::StampedTransform trans;
        transformStampedMsgToTF(tfm->transforms[i], trans);
        l->setTransform(trans);
       // cout<<tfm->transforms[i].header.frame_id<<endl;
      }
    }
  }

  setTrajectory(l);
}

/**
 * Calculates Trajectory from the ground up, new odometry, new filtering with IMU etc.
 */
void calibration::calculateTrajectory() {
  rosbag::View tfview(*bag, rosbag::TopicQuery("/VMC"));
  volksbotRT::Odometry odo;
  odo.setTicks(ticks_per_cm);
  odo.setWheelBase(wheel_base);


  ros::Duration d = tfview.getEndTime() - tfview.getBeginTime();
  d += ros::Duration(1.0); // add a small buffer

  tf::TransformListener *l = new tf::TransformListener( d, false);
  BOOST_FOREACH(rosbag::MessageInstance const m, tfview)
  {
    if ( m.isType<volksbotRT::ticks>() ) {
      volksbotRT::ticksConstPtr tptr = m.instantiate<volksbotRT::ticks>();

      odo.convertTicks2Odom(tptr);
      const nav_msgs::Odometry& odom = odo.getCurrentOdom();

      geometry_msgs::TransformStamped odom_trans;
      // copy contents of odometry message to tfstamped
      odom_trans.header.stamp = tptr->header.stamp;
      odom_trans.header.frame_id = "/odom_combined"; // frame the scan is transformed to
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = odom.pose.pose.position.x;
      odom_trans.transform.translation.y = odom.pose.pose.position.y;
      odom_trans.transform.translation.z = odom.pose.pose.position.z;
      odom_trans.transform.rotation = odom.pose.pose.orientation;


      tf::StampedTransform trans;
      transformStampedMsgToTF(odom_trans, trans);
      l->setTransform(trans);

      addStaticTransforms(l, odom.header.stamp);

    }
  }

  setTrajectory(l);
}

/**
 * Calculates Trajectory from the Odometry only, disregarding the runtime tf history
 */
void calibration::calculateTrajectoryFromOdom() {
  rosbag::View tfview(*bag, rosbag::TopicQuery("/odom"));

  tf::TransformListener *l = new tf::TransformListener( tfview.getEndTime() - tfview.getBeginTime(), false);
  BOOST_FOREACH(rosbag::MessageInstance const m, tfview)
  {
    if ( m.isType<nav_msgs::Odometry>() ) {
      nav_msgs::OdometryConstPtr optr = m.instantiate<nav_msgs::Odometry>();
      geometry_msgs::TransformStamped odom_trans;
      // copy contents of odometry message to tfstamped
      odom_trans.header.stamp = optr->header.stamp;
      odom_trans.header.frame_id = "/odom_combined"; // frame the scan is transformed to
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = optr->pose.pose.position.x;
      odom_trans.transform.translation.y = optr->pose.pose.position.y;
      odom_trans.transform.translation.z = optr->pose.pose.position.z;
      odom_trans.transform.rotation = optr->pose.pose.orientation;


      tf::StampedTransform trans;
      transformStampedMsgToTF(odom_trans, trans);
      l->setTransform(trans);

      addStaticTransforms(l, optr->header.stamp);

    }
  }

  setTrajectory(l);
}

void calibration::calculateTrajectory(bool recreateOdom, bool redoFiltering) {
  ROS_INFO("CalculateTrajectory");
  rosbag::View view(*bag, rosbag::TopicQuery("/VMC"));
  view.addQuery(*bag, rosbag::TopicQuery("/odom") );

  if (redoFiltering) {
    view.addQuery(*bag, rosbag::TopicQuery("/imu_data") );
  }

  ros::Duration d = view.getEndTime() - view.getBeginTime();
  d += ros::Duration(3.0); // add a small buffer

  tf::TransformListener *l = new tf::TransformListener( d, false);

  ekffilter ekf(l);
  volksbotRT::Odometry odo;
  odo.setTicks(ticks_per_cm);
  odo.setWheelBase(wheel_base);

//  nav_msgs::OdometryConstPtr optr;
  nav_msgs::Odometry o;
  sensor_msgs::Imu i;
  bool firstticks = true;

  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
    if (recreateOdom && m.isType<volksbotRT::ticks>() ) {
      volksbotRT::ticksConstPtr tptr = m.instantiate<volksbotRT::ticks>();
      odo.convertTicks2Odom(tptr); // recompute odometry
      if (firstticks) {
        firstticks = false;
        continue;
      }

      o =  odo.getCurrentOdom();
      if (redoFiltering)
        ekf.addOdom(o);
      addStaticTransforms(l,o.header.stamp);
    } else if (!recreateOdom && m.isType<nav_msgs::Odometry>() ) {
      o = *(m.instantiate<nav_msgs::Odometry>());
      addStaticTransforms(l,o.header.stamp);
      if (redoFiltering)
        ekf.addOdom(o);
    } else if (redoFiltering && m.isType<sensor_msgs::Imu>()) {
      i = *(m.instantiate<sensor_msgs::Imu>());
      addStaticTransforms(l,i.header.stamp);
      ekf.addImu(i);
    }

    if (redoFiltering) {

    } else { // in this case imu values are disregarded, instead compute odom_combined from odometry only
      // copy contents of odometry message to tfstamped
      geometry_msgs::TransformStamped odom_trans;
      odom_trans.header.stamp = o.header.stamp;
      odom_trans.header.frame_id = "/odom_combined"; // frame the scan is transformed to
      odom_trans.child_frame_id = "base_link";
      odom_trans.transform.translation.x = o.pose.pose.position.x;
      odom_trans.transform.translation.y = o.pose.pose.position.y;
      odom_trans.transform.translation.z = o.pose.pose.position.z;
      odom_trans.transform.rotation = o.pose.pose.orientation;

      tf::StampedTransform trans;
      transformStampedMsgToTF(odom_trans, trans);
      l->setTransform(trans);

    }
  }

  setTrajectory(l);
}



void calibration::addStaticTransforms(tf::TransformListener *l, ros::Time t) {
  tf::Quaternion q;

  // SICK
  q.setRPY(sick_euler[2], sick_euler[1], sick_euler[0]);
  l->setTransform( tf::StampedTransform(tf::Transform(q, tf::Vector3( sick_pos[0], sick_pos[1], sick_pos[2])), t, "/base_link", "/front_laser" ) );

  // RIEGL
  // yaw, pitch, roll in configuration files
  q.setRPY(riegl_euler[2], riegl_euler[1], riegl_euler[0]);
  l->setTransform( tf::StampedTransform(tf::Transform(q, tf::Vector3( riegl_pos[0], riegl_pos[1], riegl_pos[2])), t, "/base_link", "/riegl" ) );

  // XSENS
  q.setRPY(xsens_euler[2], xsens_euler[1], xsens_euler[0]);
  l->setTransform( tf::StampedTransform(tf::Transform(q, tf::Vector3( xsens_pos[0], xsens_pos[1], xsens_pos[2])), t, "/base_link", "/xsens" ) );
}


void calibration::getParamList(double *list, int nr, const char*name, const double *def) {
    ros::NodeHandle n;
    XmlRpc::XmlRpcValue my_list;
    n.getParam(name, my_list);

    for (int32_t i = 0; i < nr; ++i)
    {
      if (my_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        list[i] = my_list[i];
      } else if (my_list[i].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        int tmp = my_list[i];
        list[i] = tmp;
      } else if (def != 0) {
        list[i] = def[i];
      }
    }
}


void calibration::calibrate(timeMap &tm, evaluator &ev, ScanImporter &si) {
  tmap = &tm;
  eval = &ev;
  imp = &si;

  // nurmerical recipies uses arrays oddly, first value is never used
  float startestimate[nr_params];
  float **directions = new float*[nr_params];

  for (unsigned int i = 0; i < nr_params; i++)
    directions[i] = new float[nr_params];

  for (unsigned int i = 0; i < nr_params; i++)
    for (unsigned int j = 0; j < nr_params; j++)
      directions[i][j] = 0.0;

  for (unsigned int i = 0; i < nr_params; i++) {
    directions[i][i] = 0.01;  // 0.01 instead of 1 to minimize overshooting
//    directions[i][i] = 0.1;
  }

  // set starting estimate
  toX(startestimate);

  float ftol = 0.001;

  float best_val;
  int iterations = 0;

  ROS_INFO("Starting powell");
  printHeaderToFile("calibration.hist");
  powell(startestimate, directions, nr_params-1, ftol, &iterations, &best_val, this);

  outputCalibration("calibration_new.yaml");

  cout << "Took " << iterations << " ( " << counter << " ) iterations with result: ";
  for (unsigned int i = 1; i < nr_params; i++)
    cout <<  startestimate[i] << " ";
  cout << "with val = " << best_val << endl;

}

float calibration::function(float *X) {
  counter++;

  int index = contains(X);

  if (index != -1) {
    printToFile(X, rescache[index], "calibration.hist");
    return rescache[index];
  }

  // set calibration parameters according to X
  fromX(X);


  // alternatively change odom parameters as well (recomputing the odometry guess)

  // set up the tf history (the path of the robot)
  // 4 possibilities exist:
  //  1. only use odometry as is, discarding the imu (false, false)
  //  2. recompute odometry, discard imu (true, false)
  //  3. use odometry as is, refilter with imu (false, true)
  //  4. recompute odometry and refilter (true, true)

  bool redoOdo = (params & (USE_ODOM_WHEEL_BASE | USE_ODOM_TICKS ));
  bool redoFilter = (params & (USE_XSENS_RX | USE_XSENS_RY | USE_XSENS_RZ ));

  calculateTrajectory(redoOdo, redoFilter);


  // try different time differences
  tmap->setOffset(time_offset);

  vector<Scan *> scans;
  // we need to create a new importer, since the riegl-baseclass pointcloud is not able to reset timestamps
  ScanImporter simp(*imp);
  simp.setScans(&scans);
  simp.import();

  double value = eval->evaluate(scans);

  // cache values for speedup
  float *copy = new float[nr_params];
  for (unsigned int i = 1; i < nr_params; i++)
    copy[i] = X[i];

  cache.push_back(copy);
  rescache.push_back(value);

  printToFile(X, value, "calibration.hist");

  for (int i = 0; i < (int)scans.size(); i++) {
    delete scans[i];
  }
  scans.clear();

  return value;
}
