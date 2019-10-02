#ifndef __CALIBRATION_H__
#define __CALIBRATION_H__

#include "importer.h"
#include "timemap.h"
#include "evaluator.h"
#include "ekf_filter.h"

#include "rosbag/bag.h"
#include "rosbag/view.h"

#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <sstream>



class ScanImporter;

class calibration {
  rosbag::Bag *bag;
  tf::TransformListener *listener;

  double wheel_base;
  double ticks_per_cm;

  double sick_pos[3];
  double sick_euler[3];

  double riegl_pos[3];
  double riegl_euler[3];

  double xsens_pos[3];
  double xsens_euler[3];

  double time_offset;

  public:

  static const unsigned int USE_NONE;

  static const unsigned int USE_RIEGL_X ;
  static const unsigned int USE_RIEGL_Y ;
  static const unsigned int USE_RIEGL_Z ;
  static const unsigned int USE_RIEGL_RX;
  static const unsigned int USE_RIEGL_RY;
  static const unsigned int USE_RIEGL_RZ;

  static const unsigned int USE_SICK_X  ;
  static const unsigned int USE_SICK_Y  ;
  static const unsigned int USE_SICK_Z  ;
  static const unsigned int USE_SICK_RX ;
  static const unsigned int USE_SICK_RY ;
  static const unsigned int USE_SICK_RZ ;

  static const unsigned int USE_XSENS_X ;
  static const unsigned int USE_XSENS_Y ;
  static const unsigned int USE_XSENS_Z ;
  static const unsigned int USE_XSENS_RX;
  static const unsigned int USE_XSENS_RY;
  static const unsigned int USE_XSENS_RZ;

  static const unsigned int USE_ODOM_WHEEL_BASE;
  static const unsigned int USE_ODOM_TICKS;

  static const unsigned int USE_TIME_OFFSET;


  calibration(rosbag::Bag *_bag, unsigned int parameters = USE_NONE) {
    ROS_INFO("Calibration ctr");
    ros::NodeHandle n;

    n.param("/odometry/wheel_base", wheel_base, 44.4);
    n.param("/odometry/ticks_per_cm", ticks_per_cm, -461.817);
    n.param("/timing/offset", time_offset, 0.0);

    const double sick_position[] = {0.12, 0, 0.24};
    const double sick_orientation[] = {0,0,0};
    const double riegl_position[] = {-0.135, 0, 0.4325};
    const double riegl_orientation[] = {2.0943951, 0.0, 0.0};
    const double xsens_position[] = {-0.44, -0.14, 0.16};
    const double xsens_orientation[] = {3.14159265, 0, 0};
    getParamList(sick_pos, 3, "sick/position",        (const double *)sick_position );
    getParamList(sick_euler, 3, "sick/orientation",   (const double *)sick_orientation );
    getParamList(riegl_pos, 3, "riegl/position",      (const double *)riegl_position );
    getParamList(riegl_euler, 3, "riegl/orientation", (const double *)riegl_orientation );
    getParamList(xsens_pos, 3, "xsens/position",      (const double *)xsens_position );
    getParamList(xsens_euler, 3, "xsens/orientation", (const double *)xsens_orientation );

    bag = _bag;
    listener = 0;

    count(parameters, nr_params);
    params = parameters;

    counter = 0;
  }

  void getParamList(double *list, int n, const char*name, const double *def=0);

  tf::TransformListener *getTrajectory() { return listener; }
  void setTrajectory(tf::TransformListener *l) {
    if (listener) {
      delete listener;
    }
    listener = l;
  }

  /**
   * Calculates Trajectory from the saved TF data
   */
  void extractTrajectory();

  /**
   * Calculates Trajectory from the Odometry only, disregarding the runtime tf history
   */
  void calculateTrajectoryFromOdom();

  /**
   * Calculates Trajectory from the ground up, new odometry, new filtering with IMU etc.
   */
  void calculateTrajectory();
  void calculateTrajectory(bool, bool);

  void calibrate(timeMap &tm, evaluator &eval, ScanImporter &si);

  float function(float *X);

  private:

  int counter;
  void toX(float *X);
  void fromX(float *X);
  void count(unsigned int flag, unsigned int &n);

  void printHeaderToFile(const char *filename);

  void printToFile(float *X, double res, const char *filename) {
    ofstream of(filename, ios_base::app);
    of << std::setprecision(15);
    for (unsigned int i = 1; i < nr_params; i++)
      of <<  X[i] << " ";
    of << " " << res << endl;
    of.flush();
    of.close();
  }

  void outputCalibration(const char *filename) {
    printHeaderToFile(filename);

    ofstream of(filename, ios_base::app);
    of << std::setprecision(15);
    of << "odometry/wheel_base:      " << wheel_base << endl;
    of << "odometry/ticks_per_cm:    " << ticks_per_cm << endl;
    of << endl;
    of << "sick/position:           [" << sick_pos[0] << ", "
                                       << sick_pos[1] << ", "
                                       << sick_pos[2] << "]" << endl;
    of << "sick/orientation:        [" << sick_euler[0] << ", "
                                       << sick_euler[1] << ", "
                                       << sick_euler[2] << "]" << endl;
    of << endl;
    of << "riegl/position:          [" << riegl_pos[0] << ", "
                                       << riegl_pos[1] << ", "
                                       << riegl_pos[2] << "]" << endl;
    of << "riegl/orientation:       [" << riegl_euler[0] << ", "
                                       << riegl_euler[1] << ", "
                                       << riegl_euler[2] << "]" << endl;
    of << endl;
    of << "xsens/position:          [" << xsens_pos[0] << ", "
                                       << xsens_pos[1] << ", "
                                       << xsens_pos[2] << "]" << endl;
    of << "xsens/orientation:       [" << xsens_euler[0] << ", "
                                       << xsens_euler[1] << ", "
                                       << xsens_euler[2] << "]" << endl;
    of << endl;
    of << "timing/offset:            " << time_offset << endl;
    of << endl;

    of.flush();
    of.close();

  }

  /**
   * This function adds static transforms for all sensors on the robot
   */
  void addStaticTransforms(tf::TransformListener *l, ros::Time t);

  unsigned int nr_params;
  timeMap *tmap;
  evaluator *eval;
  ScanImporter *imp;

  unsigned int params;

  vector<float *> cache;
  vector<double> rescache;

#define NUM_LIMIT 0.0000000001
  int contains(float *X) {
    for (unsigned int i = 0; i < cache.size(); i++) {
      for (unsigned int j = 1; j < nr_params; j++) {  // start at 1 due to num rec
        if ( fabs(X[j] - cache[i][j]) > NUM_LIMIT)
          goto too_large;
      }
      // all checks failed, numbers are close enough
      return i;

      too_large:;  // numbers are different

    }
    return -1; // value is not cached
  }


};

#endif
