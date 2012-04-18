/**
 * @file
 * @brief IO of a 3D scan for leica ASCII file format with relectance values.
 * @author Sven Albrecht. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SCAN_IO_CAD_H__
#define __SCAN_IO_CAD_H__

#include <string>
using std::string;
#include <vector>
using std::vector;
#include <fstream>
#include <limits>
#include <algorithm>
#include <locale>

#include "scan_io.h"

#include <boost/program_options.hpp>
#include <boost/filesystem.hpp>

namespace po = boost::program_options;  // to reduce typing effort
namespace fs = boost::filesystem;       // to reduce typing effort

/**
 * @brief 3D scan loader for 1 UOS Scan and multiple .stl files
 *
 * The compiled class is available as shared object file
 */
class ScanIO_CAD : public ScanIO {

  // default values for parameters / flags / etc
  // sampling parameters
  static constexpr bool STD_STORE_CAD = false;    // don't save sampling
  static constexpr float STD_SAMPLE_DIST = 0.5f;  // point distance of 0.5 (cm)
  static constexpr bool STD_RND_SAMP = true;      // use random sampling
  // coordinate order
  static constexpr unsigned int STD_X_INDEX = 0;  // cad x coord = uos x coord
  static constexpr unsigned int STD_Y_INDEX = 2;  // cad y coord = uos z coord
  static constexpr unsigned int STD_Z_INDEX = 1;  // cad z coord = uos y coord
  // scale factors
  static constexpr float STD_SCALE_FACTOR = 0.1f; // general scale factor
  static constexpr float STD_X_SCALE = 0.1f;      // scale for x axis
  static constexpr float STD_Y_SCALE = 0.1f;      // scale for y axis
  static constexpr float STD_Z_SCALE = 0.1f;      // scale for z axis
  static constexpr bool STD_COMBINE_SCALES = false; // combine scale factors?
  static constexpr bool STD_INVERT_AXIS = false;  // default inversion for axes


public:

  /**
   * @brief Default constructor to initialize the member varibales with
   * their default values.
   */
  ScanIO_CAD(void);

  virtual int readScans(int start, int end, string &dir, int maxDist, 
      int minDist, double *euler, vector<Point> &ptss);

private:


  /**
   * @brief method to read a scan in regular UOS .3d file format
   *
   * @param scan_in the (opened) filestream for the .3d file
   * @param maxDist the maximal distance threshold from the origin for points
   * @param minDist the minimal distance threshold from the origin for points
   * @param points the vector containing the read scan points
   * 
   */
  void read3DScan (std::ifstream &scan_in, int maxDist, int minDist, vector<Point> &points);

  /**
   * @brief function to create a surface sampling from a given .stl file
   *
   * @param scan_in the (opened) filestream for the .stl file
   * @param points the vector containing the resultant surface sample
   * @return true if the surface sample could be created, false otherwise
   */
  bool createSamplingFromSTL (std::ifstream &scan_in, vector<Point> &points);
  /**
   * @brief Function to check if a opened .stl file contains binary or 
    * ASCII data.
    *
    * @param scan_in reference to the (opened) filestream for the .stl file
    *
    * @return true if the filestream contains an ASCII encoded .stl file,
    * false otherwise
    */
  bool hasASCIIHeader (std::ifstream &scan_in);
 
  /**
    * @brief Function to parse an ASCII encoded .stl file and create a 
    * 3D surface sampling of it.
    *
    * @param scan_in the (opened) filestream containint the .stl information
    * @param points used to store the 3D points of the surface sampling
    * @return true if the file could be parsed successfully, false otherwise
    */
  bool parseASCII (std::ifstream &scan_in, vector<Point> &points);
 
  /**
    * @brief Function to parse an binary encoded .stl file and create a 
    * 3D surface sampling of it.
    *
    * @param scan_in the (opened) filestream containint the .stl information
    * @param points used to store the 3D points of the surface sampling
    * @return true if the file could be parsed successfully, false otherwise
    */
  bool parseBinary (std::ifstream &scan_in, vector<Point> &points);

  /**
   * @brief Method to create a point sampling of a face (i.e. triangle).
   *
   * @param vertex1 the first corner of the triangle
   * @param vertex2 the second corner of the triangle
   * @param vertex3 the third corner of the triangle
   * @param points reference to the Point vector, where the sample points
   * are stored
   */
  void createSampingForFace (float* vertex1, float* vertex2, 
      float* vertex3, vector<Point> &points);

  /**
   * @brief Method to store the point sampling to the UOS .3d file format.
   * The resultant .3d file will have the same name as the original .stl 
   * file apart from the file extension.
   *
   * @param filename the name of the .stl file
   * @param points a reference to the vector containing the point 
   * information to be stored
   */
  void storeSamplingTo3D (std::string filename, vector<Point> &points);

  /**
   * @brief Computes the dot product of two 3D vectors given as float*.
   *
   * @param vec1 the first 3D vector as a float array
   * @param vec2 the second 3D vector as a float array
   *
   * @return the dot product of the two vectors
   */
  static float dotProd (const float* vec1, const float* vec2);

  /**
   * @brief Method to parse a config file.
   * The config file needs to have INI file similar syntax. If a parsing
   * error occurs a help with all available options will be printed to the
   * console and the program will terminate without matching
   *
   * @param config A filestream to the config file
   */
  void parseConfig (std::ifstream &config);

  /**
   * @brief Utility method to convert the data of a point in CAD coordinates
   * to the slam coordinate system.
   * This can involve the change of coordinate order, axis direction and
   * scaling (of all or single coordinates), depending on the configuration
   * passed via config file.
   * @param raw_data A 3D point read from the CAD file as a float array
   * @param vertex The resulting 3D point after all specified
   * operations have taken place
   */
  void copyRawData2Vertex (float *raw_data, float *vertex);

  // member variables to hold the indices used in data conversion
  unsigned int x_index;
  unsigned int y_index;
  unsigned int z_index;

  // bools to determine if an axis should be inverted during data conversion
  bool invert_x;
  bool invert_y;
  bool invert_z;

  // scale factors (global and individual) for data conversion
  float scale_factor;
  float x_scale;
  float y_scale;
  float z_scale;

  // determines if individual scale factors should be used
  bool individual_scale;
  // determines if individual scaling should be combined with
  // the global scale factor
  bool combine_scales;

  // determines if random sampling should be used for surface sampling
  bool random_sampling;

  // determines if the sampled CAD model is stored as a .3d file
  bool store_cad;
  // sets the distance between sample points (determines sample density)
  float sample_point_dist;

  // option description for easier parsing of config
  po::options_description _op_descr;
  po::variables_map _vmap;

  std::vector<fs::path> _stl_files;
  size_t _stl_files_pos;

  /**
   * @brief Method to set up the option description.
   * All allowed parameters with their default values should be
   * initialized here.
   */
  void setUpOptionDescription (void);

  void parseDirForAllSTL (std::string &directory_name);

  bool getNextSTLFromDir (fs::path &stl_file);
};

// Since this shared object file is  loaded on the fly, we
// need class factories

// the types of the class factories
typedef ScanIO* create_sio();
typedef void destroy_sio(ScanIO*);

#endif
