#ifndef __SICKIMPORTER_H__
#define __SICKIMPORTER_H__


#include "laser_geometry/laser_geometry.h"
#include "calibration.h"

#include "rosbag/bag.h"

#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"

#include "slam6d/globals.icc"
#include "slam6d/scan.h"
#include "slam6d/icp6Dquat.h"
#include "slam6d/icp6D.h"

#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <tf/transform_listener.h>

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

class sickImporter {
  std::string outputpath;
  calibration *cal;
  rosbag::Bag *bag;
  int skip;

public:

  sickImporter(std::string _outputpath, calibration *_cal, rosbag::Bag *_bag, int _skip ) : outputpath(_outputpath), cal(_cal), bag(_bag), skip(_skip) {

  }

  void getTransform(double *t, double *ti, double *rP, double *rPT, ros::Time time) {
    tf::StampedTransform transform;
    cal->getTrajectory()->lookupTransform ("/odom_combined", "/base_link", time, transform);

    double mat[9];
    double x = transform.getOrigin().getX()*100;
    double y = transform.getOrigin().getY()*100;
    double z = transform.getOrigin().getZ()*100;
    mat[0] = transform.getBasis().getRow(0).getX();
    mat[1] = transform.getBasis().getRow(0).getY();
    mat[2] = transform.getBasis().getRow(0).getZ();

    mat[3] = transform.getBasis().getRow(1).getX();
    mat[4] = transform.getBasis().getRow(1).getY();
    mat[5] = transform.getBasis().getRow(1).getZ();

    mat[6] = transform.getBasis().getRow(2).getX();
    mat[7] = transform.getBasis().getRow(2).getY();
    mat[8] = transform.getBasis().getRow(2).getZ();

    t[0] = mat[4];
    t[1] = -mat[7];
    t[2] = -mat[1];
    t[3] = 0.0;

    t[4] = -mat[5];
    t[5] = mat[8];
    t[6] = mat[2];
    t[7] = 0.0;

    t[8] = -mat[3];
    t[9] = mat[6];
    t[10] = mat[0];
    t[11] = 0.0;

    // translation
    t[12] =  -y;
    t[13] =  z;
    t[14] =  x;
    t[15] = 1;
    M4inv(t, ti);
    Matrix4ToEuler(t, rPT, rP);
  }

  void import() {
    rosbag::View tfview(*bag, rosbag::TopicQuery("/LMS")); // laser scans
    laser_geometry::LaserProjection projector;
    int counter = 0;
    int skipcounter = 0;
    std::ofstream o;     // file stream for scan
    double transform[16], tinv[16], rP[3], rPT[3];

    BOOST_FOREACH(rosbag::MessageInstance const m, tfview)
    {
      if ( m.isType<sensor_msgs::LaserScan>()  ) {
        skipcounter++;
        if (skipcounter % skip != 0) {
          continue;
        }


        sensor_msgs::LaserScanConstPtr ls = m.instantiate<sensor_msgs::LaserScan>();
        sensor_msgs::PointCloud cloud_in;
        sensor_msgs::PointCloud cloud_out;

        projector.projectLaser(*ls, cloud_in); // convert to point cloud
        try {
        //  cout << ls->header.frame_id << " " << cloud_in.header.frame_id << endl;
          cal->getTrajectory()->transformPointCloud("/odom_combined", cloud_in, cloud_out);  // transform
        }
        catch (tf::TransformException ex){
          ROS_ERROR("Transform error in sickImporter::import() : %s",ex.what());
          continue;
        }

        getTransform(transform, tinv, rP, rPT, ls->header.stamp);


        // write to file
        string scanFileName = outputpath + "scan"+ to_string(counter,3) + ".3d";
        string poseFileName = outputpath + "scan"+ to_string(counter,3) + ".pose";
        string framesFileName = outputpath + "scan"+ to_string(counter,3) + ".frames";
        counter++;

        // scan
        o.open(scanFileName.c_str());
        o << std::setprecision(10);
        for (unsigned int j = 0; j < cloud_out.points.size(); j++) {
          geometry_msgs::Point32 p = cloud_out.points[j];

          double x_neu, y_neu, z_neu, x,y,z;
          x = -100.0 * p.y;
          y = 100.0 * p.z;
          z = 100.0 * p.x;
        //  o << x << " " << y << " " << z << endl;
          x_neu = x * tinv[0] + y * tinv[4] + z * tinv[8];
          y_neu = x * tinv[1] + y * tinv[5] + z * tinv[9];
          z_neu = x * tinv[2] + y * tinv[6] + z * tinv[10];
          x = x_neu + tinv[12];
          y = y_neu + tinv[13];
          z = z_neu + tinv[14];

          o << x << " " << y << " " << z << endl;
        }
        o.close();
        // pose
        o.open(poseFileName.c_str());
        o << rP[0] << " " << rP[1] << " " << rP[2] << endl;
        o << deg(rPT[0]) << " " << deg(rPT[1]) << " " << deg(rPT[2]) << endl;
        o.flush();
        o.close();

        // frames
        o.open(framesFileName.c_str());
        for (int k = 0; k < 3; k++) {
          for (unsigned int i = 0; i < 16; i++) {
            o << transform[i] << " ";
          }
          o << "2" << endl;
        }
        o.flush();
        o.close();
      }
    }
  }



  };

#endif
