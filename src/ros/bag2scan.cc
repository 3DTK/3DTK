#include "ros/importer.h"
#include "ros/sickimporter.h"
#include "ros/evaluator.h"
#include "ros/timemap.h"
#include "ros/calibration.h"

#include "std_msgs/Empty.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/Point32.h"

#include "riegl/RieglStatus.h"
#include "riegl/RieglTime.h"

#include "slam6d/globals.icc"
#include "slam6d/scan.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6D.h"
#include "slam6d/point.h"
#include "slam6d/point_type.h"
#include "slam6d/icp6Dquat.h"

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>

#include <iomanip>
#include <iostream>
#include <fstream>
#include <queue>
#include <vector>
#include <sstream>

#include <pthread.h>
#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <boost/foreach.hpp>
#include <boost/filesystem/operations.hpp>

#include <getopt.h>

/**
 * Explains the usage of this program's command line parameters
 * @param prog name of the program
 */
void usage(char* prog)
{
  const std::string bold("\033[1m");
  const std::string normal("\033[m");
  std::cout << std::endl
    << bold << "USAGE " << normal << std::endl
    << "   " << prog << " [options] directory" << std::endl << std::endl;
  std::cout << bold << "OPTIONS" << normal << std::endl
  << bold << "  -L" << normal << ", " << bold << "--linescans" << normal << "" << std::endl
  << "         transform scan line by line AND writes a scan per line" << std::endl
  << bold << "  -l" << normal << ", " << bold << "--lines" << normal << "" << std::endl
  << "         transform scan line by line" << std::endl
  << bold << "  -t" << normal << ", " << bold << "--trajectory" << normal << "" << std::endl
  << "         writes computed trajectory to file trajectory.txt" << std::endl
  << bold << "  -T" << normal << ", " << bold << "--trustTF" << normal << "" << std::endl
  << "         do not recompute trajectory, but trust the logged tf history" << std::endl
  << bold << "  -d NR" << normal << ", " << bold << "--diff NR" << normal << "" << std::endl
  << "         adds <NR> seconds to riegl timestamps (default = -0.662)" << std::endl
  << bold << "  -p" << normal << ", " << bold << "--rieglcoord" << normal << "" << std::endl
  << "         write scans in the riegl coordinate system." << std::endl
  << bold << "  -m" << normal << ", " << bold << "--mapping" << normal << "" << std::endl
  << "         write pose with respect to the /map coordinate system." << std::endl
  << bold << "  -n" << normal << ", " << bold << "--norobot" << normal << "" << std::endl
  << "         if the scanner was used without the robot running" << std::endl
  << bold << "  -c" << normal << ", " << bold << "--calibrate" << normal << "" << std::endl
  << "         calibrates the system using the given log." << std::endl

  << bold << "  -R, --reflectance, --reflectivity" << normal << std::endl
  << "         write reflectivity values " << std::endl
  << std::endl
  << bold << "  -a, --amplitude" << std::endl << normal
  << "         write amplitude values " << std::endl
  << std::endl
  << bold << "  -d, --deviation" << std::endl << normal
  << "         write deviation values " << std::endl
  << std::endl

  << std::endl << std::endl;

  exit(1);
}


void parseArgs(int argc,char **argv, std::string &dir, bool &lines, bool &trajectory,
bool &trustTF, double &tdiff, bool &calibrate, int &start, int &end, bool
&redoOdometry, bool &redoFilter, int &sickscans, bool &stopandgo, bool
&rieglcoord, bool &norobot, bool &linescans, unsigned int &types, std::string
&mapstring) {

  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  std::cout << std::endl;
  static struct option longopts[] = {
    { "linescans",       no_argument,         0,  'L' },
    { "lines",           no_argument,         0,  'l' },
    { "trajectory",      no_argument,         0,  't' },
    { "trustTF",         no_argument,         0,  'T' },
    { "diff",            required_argument,   0,  'd' },
    { "calibrate",       no_argument,         0,  'c' },
    { "start",           required_argument,   0,  's' },
    { "end",             required_argument,   0,  'e' },
    { "redoOdo",         no_argument,         0,  'o' },
    { "redoFilter",      no_argument,         0,  'f' },
    { "writeSick",       required_argument,   0,  'w' },
    { "stopandgo",       no_argument,         0,  'g' },
    { "rieglcoord",      no_argument,         0,  'p' },
    { "norobot",         no_argument,         0,  'n' },
    { "reflectance",     no_argument,         0,  'R' },
    { "reflectivity",    no_argument,         0,  'R' },
    { "amplitude",       no_argument,         0,  'A' },
    { "deviation",       no_argument,         0,  'D' },
    { "mapping",         no_argument,         0,  'm' },

    /*
    { "octree",          optional_argument,   0,  'O' },
    */
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  while ((c = getopt_long(argc, argv,"tTcADgw:ofLld:s:e:pnRm", longopts, NULL)) != -1)
    switch (c)
   {
   case 'L':
     linescans = true;
     break;
   case 'l':
     lines = true;
     break;
   case 't':
     trajectory = true;
     break;
   case 'T':
     trustTF = true;
     break;
   case 'd':
     tdiff = atof(optarg);
     break;
   case 'c':
     calibrate = true;
     break;
   case 's':
     start = atoi(optarg);
     if (start < 0) { std::cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
     break;
   case 'e':
     end = atoi(optarg);
     if (end < 0)     { std::cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
     if (end < start) { std::cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
     break;
   case 'o':
     redoOdometry = true;
     break;
   case 'w':
     sickscans = atoi(optarg);
     break;
   case 'f':
     redoFilter = true;
     break;
   case 'g':
     stopandgo = true;
     break;
   case 'p':
     rieglcoord = true;
     break;
   case 'n':
     norobot = true;
     stopandgo = true;
     break;
   case 'm':
     mapstring = "/map";
     break;
   case 'R':
     types |= PointType::USE_REFLECTANCE;
     break;
   case 'A':
     types |= PointType::USE_AMPLITUDE;
     break;
   case 'D':
     types |= PointType::USE_DEVIATION;
     break;
   default:
     usage(argv[0]);
     break;
   }

  if (linescans) {
    lines = true;
    calibrate = false;
  }

  if (optind != argc-1) {
    std::cerr << "\n*** Directory missing ***" << std::endl;
    usage(argv[0]);
  }
  dir = argv[optind];

  if (dir[dir.length()-1] != '/') dir = dir + "/";
}

void writeTrajectory(rosbag::Bag &bag, tf::TransformListener *l) {
  rosbag::View tfview(bag, rosbag::TopicQuery("/odom"));
  std::ofstream o;
  o.open("trajectory.txt");
  double rx,ry,rz;


  BOOST_FOREACH(rosbag::MessageInstance const m, tfview)
  {
    if ( m.isType<nav_msgs::Odometry>() ) {
      nav_msgs::OdometryConstPtr optr = m.instantiate<nav_msgs::Odometry>();

      try {
        tf::StampedTransform transform;
        l->lookupTransform("/odom_combined", "base_link", optr->header.stamp, transform);
        transform.getBasis().getRPY(rx,ry,rz);
        o << transform.getOrigin().getX() << " " << transform.getOrigin().getY() << " " << rz << std::endl;
      }
      catch (tf::TransformException ex){
        ROS_ERROR("Transform error in writeTrajectory: %s",ex.what());
      }

    }
  }

  o.close();
}

/**
 * This function adds static transforms for all sensors on the robot
 */
void addStaticTransforms(tf::TransformListener *l, ros::Time t) {
  tf::Quaternion q;

  // SICK
  q.setRPY(0.0, 0, 0.0);
  l->setTransform( tf::StampedTransform(tf::Transform(q, tf::Vector3(0.12,0,0.24)), t, "/base_link", "/front_laser" ) );

  // RIEGL
  // yaw, pitch, roll in configuration files
  q.setRPY(0, 0, 2.0943951);
  l->setTransform( tf::StampedTransform(tf::Transform(q, tf::Vector3(-0.135, 0, 0.4325)), t, "/base_link", "/riegl" ) );

  // XSENS
  q.setRPY(3.14159265, 0, 0);
  l->setTransform( tf::StampedTransform(tf::Transform(q, tf::Vector3(-0.17, 0, 0.18)), t, "/base_link", "/xsens" ) );
}

/**
 * Calculates Trajectory from the Odometry only, disregarding the tf history
 */
tf::TransformListener *calculateTrajectoryFromOdom(rosbag::Bag &bag) {
  rosbag::View tfview(bag, rosbag::TopicQuery("/odom"));

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

  ROS_INFO("TF History succesfully set up!");

  return l;
}

/**
 * Calculates Trajectory from the saved TF data
 *
 */
tf::TransformListener *calculateTrajectory(rosbag::Bag &bag) {
  ROS_INFO("before tfview");
  rosbag::View tfview(bag, rosbag::TopicQuery("/tf"));

  ROS_INFO("before TransformListener");
  tf::TransformListener *l = new tf::TransformListener( tfview.getEndTime() - tfview.getBeginTime(), false);
  ROS_INFO("TransformListener");
  BOOST_FOREACH(rosbag::MessageInstance const m, tfview)
  {
    if ( m.isType<tf::tfMessage>() ) {
      tf::tfMessageConstPtr tfm = m.instantiate<tf::tfMessage>();
      for (unsigned int i = 0; i < tfm->transforms.size(); i++) {
        tf::StampedTransform trans;
        transformStampedMsgToTF(tfm->transforms[i], trans);

        l->setTransform(trans);
      }
    }
  }

  ROS_INFO("TF History succesfully set up!");

  return l;
}



void readTSMapfromBag(rosbag::Bag &bag, std::vector<double> *timestamps) {
  rosbag::View tfview(bag, rosbag::TopicQuery("/riegltime"));

  bool first = true;
  double friegltime;
  ros::Time rostime, frostime;
  ros::Duration d;

  BOOST_FOREACH(rosbag::MessageInstance const m, tfview)
  {
    if ( m.isType<riegl::RieglTime>() ) {
      riegl::RieglTimeConstPtr rtptr = m.instantiate<riegl::RieglTime>();
      if (first) {
        first = false;
        friegltime = rtptr->time;
        frostime = rtptr->header.stamp;
      }

      timestamps[0].push_back( rtptr->time );
      timestamps[1].push_back( rtptr->header.stamp.toSec());
    }
  }


  return ;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "assembler");
  ros::NodeHandle n;
  ROS_INFO("Node created!\n");


  // get configuration
  std::string basedir, tsfile, bagfile, rxpfile, outputpath, sickoutputpath;
  bool lines = true;        // TODO: add option to switch this off
  bool trajectory = false;
  int sickscans = 0;
  bool trustTF = false;
  //double tdiff = -0.6620;    // default value is 662ms in the past
  double tdiff = 0.0;    // default value is 662ms in the past
  bool calibrate = false;
  int start = -1;
  int end = -1;
  bool redoOdometry = false;
  bool redoFilter = false;
  bool stopandgo = false;
  bool rieglcoord = false;
  bool norobot = false;
  bool linescans = false;
  std::string mapstring = "/odom_combined";
  unsigned int types = PointType::USE_NONE;

  parseArgs(argc, argv, basedir, lines, trajectory, trustTF, tdiff, calibrate,
  start, end, redoOdometry, redoFilter, sickscans, stopandgo, rieglcoord,
  norobot, linescans, types, mapstring);

  ROS_INFO("mapstring %s", mapstring.c_str());
  n.param<std::string>("/log/timestamp", tsfile, "timestamps.txt");
  n.param<std::string>("/log/bagfile", bagfile, "log.bag");
  n.param<std::string>("/log/scandir", outputpath, "scan3d/");
  n.param<std::string>("/log/lms100", sickoutputpath, "lms/");
  n.param<std::string>("/log/rxp", rxpfile, "raw.rxp");
  outputpath = basedir + outputpath;
  sickoutputpath = basedir + sickoutputpath;
  bagfile =  basedir + bagfile;
  tsfile =   basedir + tsfile;
  rxpfile =  boost::filesystem::canonical(basedir + rxpfile).string();
  // create directory if it does not yet exist
  mkdir(outputpath.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
  mkdir(sickoutputpath.c_str(), S_IRWXU|S_IRWXG|S_IRWXO);
  // done with configuring

  // process bag file
  rosbag::Bag bag(bagfile);
  if(rieglcoord)
    ROS_INFO("rieglcoord = true");
  else
    ROS_INFO("rieglcoord = false");
  if(trustTF)
    ROS_INFO("trustTF = true");
  if(linescans)
    ROS_INFO("linescans = true");
  if(norobot)
    ROS_INFO("norobot = true");
  else
    ROS_INFO("norobot = false");

  std::vector<double> tm[2];
  readTSMapfromBag(bag, tm );
//  timeMap *tmap = new timeMapSimple(tm);
//  timeMap *tmap = new timeMapLinear(tm);
//  timeMap *tmap = new timeMapLinearOlsen(tm);
  timeMap *tmap = new timeMapOlsen(tm);
  calibration cal = calibration(&bag,
      calibration::USE_ODOM_WHEEL_BASE | calibration::USE_ODOM_TICKS |
      calibration::USE_RIEGL_X | calibration::USE_RIEGL_Y
      | calibration::USE_RIEGL_RX | calibration::USE_RIEGL_RY | calibration::USE_RIEGL_RZ
//      | calibration::USE_XSENS_RX | calibration::USE_XSENS_RY | calibration::USE_XSENS_RZ
//       |  calibration::USE_TIME_OFFSET
      );

  if (!calibrate) {
    // set up the tf history (the path of the robot)
    if (trustTF) cal.extractTrajectory();
    else  cal.calculateTrajectory(redoOdometry, redoFilter);
    // if the trajectory should be written to file
    if (trajectory) writeTrajectory(bag, cal.getTrajectory());
    if (sickscans) {
      sickImporter si(sickoutputpath, &cal, &bag, sickscans);
      si.import();
    }


    tmap->setOffset(tdiff);
    if (linescans) {
      LineScanImporter lsi(rxpfile, tmap, outputpath, &cal, lines, stopandgo, types, start, end, mapstring);
      lsi.import();
    } else {
      FileImporter fi(rxpfile, tmap, outputpath, &cal, lines, stopandgo, rieglcoord, norobot, types, start, end, mapstring);
      fi.import();
    }
  } else {
    /////////////////////////
    cal.calculateTrajectory(redoOdometry, redoFilter);
    ScanReducer sr(rxpfile, tmap, &cal,
        5, 1, // reduction parameters
        lines, stopandgo, PointType::USE_NONE, start, end);
    sr.import();


    //////////////////////
   PPevaluator eval(5.0, true);  // mdist parameter TODO
//    PPevaluatorICP eval(10.0);  // mdist parameter TODO
std::cout << "start eval with " << (end - start + 1) << " scans" << std::endl;
//   Planeevaluator eval(basedir, end-start + 1,  0,5);
    ScanImporter si(rxpfile, tmap, &cal, lines, stopandgo, PointType::USE_NONE, start, end);
    si.setReducer(&sr);
    cal.calibrate(*tmap, eval, si);
  }
}

