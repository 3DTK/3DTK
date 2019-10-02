#ifndef __IMPORTER_H__
#define __IMPORTER_H__

#include "riegl/scanlib.hpp"

#include "timemap.h"
#include "calibration.h"

#include "rosbag/bag.h"
#include "std_msgs/Empty.h"
#include "sensor_msgs/PointCloud.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Point32.h"

#include "riegl/RieglStatus.h"
#include "riegl/RieglTime.h"

#include "slam6d/point_type.h"
#include "slam6d/scan.h"

#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>

#include <boost/asio/io_service.hpp>
#include <boost/bind.hpp>
#include <boost/thread/thread.hpp>

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

#include <ros/ros.h>

#if defined(_MSC_VER)
#   include <memory>
#else
#   include <tr1/memory>
#endif
using namespace scanlib;
//using namespace std;
using namespace std::tr1;

class calibration;

class  scanstruct {
public:
  scanstruct(int _index, bool _invert = true) : invert(_invert) {
    // TODO make reserving dependant on scan parameters
    points.reserve(721*500);
    clouds.reserve(721);
    index = _index;
  }

  ~scanstruct() {
    for (unsigned int i = 0; i < points.size(); i++)
      delete points[i];
    points.clear();
    for (unsigned int i = 0; i < clouds.size(); i++)
      delete clouds[i];
    clouds.clear();
  }

  // sets this scans pose
  void setTransform(double *mat, double x, double y, double z);
  void setPath(std::string _outputpath) { outputpath = _outputpath; };

  // these vectors contain the points, or if a buffer is used the pointclouds
  std::vector<geometry_msgs::PointStamped*> points;
  std::vector<sensor_msgs::PointCloud *> clouds;

  unsigned int index;
  std::string outputpath;

  double transmat[16];

  bool hasPoints() {
    return points.size() > 0 || clouds.size() > 0;
  }

  bool invert;

};
// forward declaration
class ScanReducer;

class timedImporter
    : public pointcloud
{
public:
    inline timedImporter(std::string _rxpfile, timeMap *_tmap, calibration *_cal,
        bool _lines=false, bool _stopandgo=false, bool _rieglcoord=false,
        bool _norobot = false, unsigned int _pointtype = PointType::USE_NONE,
        int _start=-1, int _end=-1, std::string
     _mapstring = "/odom_combined", bool _linescans=false)
      : pointcloud(false), linescans(_linescans), start(_start), end(_end),
      rxpfile(_rxpfile), tmap(_tmap), lines(_lines), stopandgo(_stopandgo),
      rieglcoord(_rieglcoord), norobot(_norobot), mapstring(_mapstring)
    {
      remembertime = true;
      if (lines) {
        pbuffer.reserve(500);
        use_buffer = true;
      }
      counter = 0;
      pcounter = 0;
      current = new scanstruct(counter++, !stopandgo );
      cal = _cal;

      rxpfile = "file://" + rxpfile;

      pointtype = PointType(PointType::USE_TIME | _pointtype);

      reducer = 0;
    }

    void import() {
#ifdef WITH_OLD_RIVLIB
      std::tr1::shared_ptr<basic_rconnection> rc;
#else
      std::shared_ptr<basic_rconnection> rc;
#endif
//      ROS_INFO("open file %s for reading scans...", rxpfile.c_str());
      rc = basic_rconnection::create( rxpfile );
      rc->open();
      decoder_rxpmarker dec(rc);
      buffer buf;

      for ( dec.get(buf); !dec.eoi() && ros::ok() && ((end >= 0 && end >=  (int)(counter - 1) ) || end < 0) ; dec.get(buf) ) {
        this->dispatch(buf.begin(), buf.end());
      }

      rc->close();

      if (lines ) {
        transform_buffer();
      }

      newScan();
      // make sure no residual stuff in the memory
      clearMem();

      // start counting from 0
      counter = 0;
      pcounter = 0;
      current = new scanstruct(counter++, !stopandgo);
      if (lines) {
        pbuffer.reserve(500);
      }
      remembertime = true;
    }

    ~timedImporter() {
      clearMem();
    }


    void setReducer(ScanReducer *red) {
      reducer = red;
    }


protected:
    ScanReducer *reducer;

    bool linescans;
    bool haspoints;
    int start;
    int end;

    void newScan();

    void clearMem() {
      delete current;
      if (!pbuffer.empty()) {
        for (unsigned int i = 0; i < pbuffer.size(); i++) {
          delete pbuffer[i];
        }
        pbuffer.clear();
      }
    }

    std::string rxpfile;
    // for using next pose as scans pose
    bool remembertime ;

    // current scan
    scanstruct *current;

    // a buffer for faster transforming the scans
//    vector<geometry_msgs::PointStamped*> pbuffer;
    std::vector<double*> pbuffer;

    void on_echo_transformed(echo_type echo);
    void on_frame_stop(const frame_stop<iterator_type>& arg);
    void on_frame_start_dn(const frame_start_dn<iterator_type>& arg) {
      basic_packets::on_frame_start_dn(arg);
      remembertime = true; // remember time again to make sure the pose is from the very first point
      newScan();
    }
    void on_frame_start_up(const frame_start_up<iterator_type>& arg) {
      basic_packets::on_frame_start_up(arg);
      remembertime = true; // remember time again to make sure the pose is from the very first point
      newScan();
    }
    void on_line_start_up(const line_start_up<iterator_type>& arg) {
      basic_packets::on_line_start_up(arg);
      if (lines) {
        this->transform_buffer();
      }
    }
    // transforms the entire buffer in a rigid fashion
    void transform_buffer();

    virtual void frameStop(scanstruct *scan) = 0;

//private:
    timeMap *tmap;

    // whether to use a buffer or transform each point individually TODO: check if 2 bols are still necessary
    bool lines, use_buffer;

    bool stopandgo;
    bool rieglcoord;
    bool norobot;
    std::string mapstring;

    PointType pointtype;
    // contains the trajectory of the robot
    calibration *cal;

    // counts the current number of scans we are at
    unsigned int counter;

    // counts the current number of points we are at
    int pcounter;

};


class FileImporter
    : public timedImporter
{
  public:
    FileImporter(std::string _rxpfile, timeMap *_tmap, std::string _outputpath, calibration *_cal,
        bool _lines=false, bool _stopandgo=false, bool _rieglcoord=false,
        bool _norobot=false,unsigned int _pointtype = PointType::USE_NONE,
        int start=-1, int end=-1, std::string _mapstring="/odom_combined")
      : timedImporter(_rxpfile, _tmap, _cal, _lines, _stopandgo, _rieglcoord, _norobot, _pointtype, start, end, _mapstring) {
        outputpath = _outputpath;

        ioservice = boost::make_shared<boost::asio::io_service>();
        // create a work object to keep ioservice running
        work = boost::make_shared<boost::asio::io_service::work>(*ioservice);
        threads = boost::make_shared<boost::thread_group>();
        for (std::size_t i = 0; i < boost::thread::hardware_concurrency(); i++) {
          threads->create_thread(boost::bind(&boost::asio::io_service::run, ioservice));
        }
      }

    ~FileImporter() {
      // destroy work object to allow ioservice to shutdown
      work.reset();
      // wait for all threads to finish
      threads->join_all();
    }

  protected:
    void frameStop(scanstruct *scan);
    static void writeScan(scanstruct *scan);
  private:
    // the path to write scans to
    std::string outputpath;

    // threadpool used for writing scans in parallel
    boost::shared_ptr<boost::asio::io_service> ioservice;
    boost::shared_ptr<boost::asio::io_service::work> work;
    boost::shared_ptr<boost::thread_group> threads;
};

class LineScanImporter
    : public timedImporter
{
  public:
    LineScanImporter(std::string _rxpfile, timeMap *_tmap, std::string _outputpath, calibration *_cal,
        bool _lines=false, bool _stopandgo=false, unsigned int _pointtype = PointType::USE_NONE,
        int start=-1, int end=-1, std::string _mapstring="/odom_combined")
      : timedImporter(_rxpfile, _tmap, _cal, _lines, _stopandgo, false, false,
      _pointtype, start, end, _mapstring, true) {
        outputpath = _outputpath;

        ioservice = boost::make_shared<boost::asio::io_service>();
        // create a work object to keep ioservice running
        work = boost::make_shared<boost::asio::io_service::work>(*ioservice);
        threads = boost::make_shared<boost::thread_group>();
        for (std::size_t i = 0; i < boost::thread::hardware_concurrency(); i++) {
          threads->create_thread(boost::bind(&boost::asio::io_service::run, ioservice));
        }
    }

    ~LineScanImporter() {
      // destroy work object to allow ioservice to shutdown
      work.reset();
      // wait for all threads to finish
      threads->join_all();
    }

  protected:
    void frameStop(scanstruct *scan);
    static void writeLineScan(scanstruct *scan);
  private:
    // the path to write scans to
    std::string outputpath;

    // threadpool used for writing scans in parallel
    boost::shared_ptr<boost::asio::io_service> ioservice;
    boost::shared_ptr<boost::asio::io_service::work> work;
    boost::shared_ptr<boost::thread_group> threads;
};


class ScanImporter
    : public timedImporter
{
  public:
    ScanImporter(std::string _rxpfile, timeMap *_tmap, calibration *_cal,
        bool _lines=false, bool _stopandgo=false,
        unsigned int _pointtype = PointType::USE_NONE,
        int start=-1, int end=-1)
      : timedImporter(_rxpfile, _tmap, _cal, _lines, _stopandgo, false, false, _pointtype, start, end) {
        scans = 0;
      }

    ScanImporter(const ScanImporter &si)
      : timedImporter(si.rxpfile, si.tmap, si.cal, si.lines, si.stopandgo, false, false, si.pointtype.toFlags(), si.start, si.end) {
      rxpfile = si.rxpfile;
      scans = si.scans;
      reducer = si.reducer;
    }

    void setScans(std::vector<Scan *> *_scans) {
      scans = _scans;
    }

  protected:
    void frameStop(scanstruct *scan);

    std::vector<Scan *> *scans;
};

class ScanReducer
    : public ScanImporter
{
  public:
    ScanReducer(std::string _rxpfile, timeMap *_tmap, calibration *_cal, double vs, int nrp,
        bool _lines=false, bool _stopandgo=false,
        unsigned int _pointtype = PointType::USE_NONE,
        int start=-1, int end=-1)
      : ScanImporter(_rxpfile, _tmap, _cal, _lines, _stopandgo,
          _pointtype, start, end) {
        scans = 0;
        currentID = 0;
        nrpts = nrp;
        voxelsize = vs;

      }

    bool accept(int i) {
      return (reducedIDs.find(i) != reducedIDs.end());
    }

    int nrpoints() {
      return reducedIDs.size();
    }

  protected:

    double voxelsize;
    int nrpts;

    int currentID;
    void frameStop(scanstruct *scan);

    std::set<int> reducedIDs;

};

#endif
