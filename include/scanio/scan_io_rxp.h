/**
 * @file
 * @brief IO of a 3D scan in rxp file format
 * @author Thomas Escher
 */

#ifndef __SCAN_IO_RXP_H__
#define __SCAN_IO_RXP_H__

#include "scan_io.h"
#include "slam6d/point.h"

#include "riegl/scanlib.hpp"
using namespace scanlib;
using namespace std;
using namespace std::tr1;

class importer;



/**
 * @brief 3D scan loader for RXP scans
 *
 * The compiled class is available as shared object file
 */
class ScanIO_rxp : public ScanIO {
public:
  virtual std::list<std::string> readDirectory(const char* dir_path, unsigned int start, unsigned int end);
  virtual void readPose(const char* dir_path, const char* identifier, double* pose);
  virtual void readScan(const char* dir_path, const char* identifier, PointFilter& filter, std::vector<double>* xyz, std::vector<unsigned char>* rgb, std::vector<float>* reflectance, std::vector<float>* temperature, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation);
  virtual bool supports(IODataType type);

  ScanIO_rxp() : dec(0), imp(0) {}
private:
  std::tr1::shared_ptr<basic_rconnection> rc;
  decoder_rxpmarker *dec;
  importer *imp;
  std::string old_path;
};

/**
 * The importer class is the interface to riegl's pointcloud class, and will convert their point struct to slam6d's point class.
 *
 * Code adapted from rivlib/example/pointcloudcpp.cpp available from http://www.riegl.com .
 */
class importer : public scanlib::pointcloud
{
public:
  importer(PointFilter& filter, int start, std::vector<double>* xyz, std::vector<float>* reflectance, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation) :
    pointcloud(false), // set this to true if you need gps aligned timing
    filter(&filter), xyz(xyz), reflectance(reflectance), amplitude(amplitude), type(type), deviation(deviation),
    start(start), currentscan(0)
  {}
  inline int getCurrentScan() { return currentscan; }
  inline void set(PointFilter& filter, std::vector<double>* xyz, std::vector<float>* reflectance, std::vector<float>* amplitude, std::vector<int>* type, std::vector<float>* deviation)
  {
    this->filter = &filter;
    this->xyz = xyz;
    this->reflectance = reflectance;
    this->amplitude = amplitude;
    this->type = type;
    this->deviation = deviation;
  }
protected:
  PointFilter* filter;
  std::vector<double> *xyz;
  std::vector<float> *reflectance;
  std::vector<float> *amplitude;
  std::vector<int> *type;
  std::vector<float> *deviation;
  int start;
  int currentscan;
  // overridden from pointcloud class
  void on_echo_transformed(echo_type echo);

  void on_frame_stop(const scanlib::frame_stop<iterator_type>& arg) {
    scanlib::basic_packets::on_frame_stop(arg);
    currentscan++;
  }
};

#endif
