/**
 * @file
 * @brief Implementation of reading a 3D CAD model and creating a 3D 
 * surface sampling of it. This sampling is then used as an artificial 
 * 3D scan in regular scan matching
 * @author Sven Albrecht. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "slam6d/scan_io_cad.h"
#include "slam6d/globals.icc"
#include <fstream>
using std::ifstream;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
#include <cmath>
#include <limits>

// TODO: make eventually compatible to windoze
#include <stdint.h>   // can be changed to cstdint if -std=c++0x compiler flag is used


#ifdef _MSC_VER
#include <windows.h>
#endif

#define BOOST_NEW_FS_API 104600

ScanIO_CAD::ScanIO_CAD (void) : _op_descr ("CAD Config Options")
{
  setUpOptionDescription ();
}

/**
 * Reads specified 'scans' from CAD models in a given directory.
 *
 * Scan poses will NOT be initialized after a call
 * to this function.
 *
 * Each CAD model will be represented by a sampling of 3D points on the
 * the surface of the CAD model. Currently the implementation is able to
 * handle CAD models in the .stl file format (binary or ASCII is fine).
 * In constrast to other ScanIOs only one 3D scan will be read (specified
 * by parameter start). Other data is generated from the (arbitrary) named
 * .stl files in the directory specified by argument dir. Parameter end is
 * soley kept to satisfy the general interface.
 *
 * !!!!!!
 * Config parameters are read from an (optional) config.txt in the scan directory.
 * A config.txt with content "help = 1" will print out all possible parameters.
 * !!!!!!
 *
 * @param start Starts to read with this scan
 * @param end Stops with this scan
 * @param dir The directory from which to read
 * @param maxDist Reads only Points up to this Distance
 * @param minDist Reads only Points from this Distance
 * @param euler Initital pose estimates (will not be applied to the points
 * @param ptss Vector containing the read points
 */
int ScanIO_CAD::readScans(int start, int end, string &dir, int maxDist, int minDist,
			  double *euler, vector<Point> &ptss)
{
  /* 
   * we need to read 1 normal scan (the first one) and afterwards 
   * create point samplings of the CAD models  
   */
  static int fileCounter = start;  // this is a really ugly hack...

  // check before the first scan is read for a config file
  if (fileCounter == start)
  {
    // is there a config file ?
    std::string config_file_name = dir + "config.txt";
    std::ifstream config;
    config.open (config_file_name.c_str ());
    if (config.good ())
    {
      // parse the config to eventually overload default parameters
      parseConfig (config);
    }
    // else use the default parameters

    // retrieve all stl files
    parseDirForAllSTL (dir);
  }

  string scanFileName;
  string poseFileName;

  ifstream scan_in, pose_in;

  // clear the point vector (to make sure no old garbage is contained)
  ptss.clear ();

  int my_fileNr = fileCounter;
  
  if (end > -1 && fileCounter > end) return -1; // 'nuf read

  fs::path stl_file;

  // the first scan file name should be a .3d file
  if (fileCounter == start)
  {
    scanFileName = dir + "scan" + to_string(fileCounter, 3) + ".3d";
    poseFileName = dir + "scan" + to_string(fileCounter, 3) + ".pose";
  }
  // otherwise it should be a .stl file
  else
  {
    // get the next stl file if the directory contains one more
    if (getNextSTLFromDir (stl_file))
    {
      fs::path pose_file (stl_file);

      // retrieve the the filenames according to the installed boost version
#if BOOST_VERSION < BOOST_NEW_FS_API
      // change extension
      pose_file.replace_extension (".pose");
      scanFileName = stl_file.directory_string ();
      poseFileName = pose_file.directory_string ();
#else
      // change extension
      pose_file.replace_extension (fs::path (".pose"));
      scanFileName = stl_file.string ();
      poseFileName = pose_file.string ();
#endif
    }
    else
    {
      // aparently no stl file available anymore
      return -1;
    }
  }
  
  scan_in.open(scanFileName.c_str());
  pose_in.open(poseFileName.c_str());

  // read pose information independent from file type
  // read 3D scan
  if (!pose_in.good() && !scan_in.good()) return -1; // no more files in the directory
  if (!pose_in.good()) { cerr << "ERROR: Missing file " << poseFileName << endl; exit(1); }
  if (!scan_in.good()) { cerr << "ERROR: Missing file " << scanFileName << endl; exit(1); }
  cout << "Processing Scan " << scanFileName;

  // read a 'normal' Slam6D Pose
  for (unsigned int i = 0; i < 6; pose_in >> euler[i++]);

  cout << " @ pose (" << euler[0] << "," << euler[1] << "," << euler[2]
	  << "," << euler[3] << "," << euler[4] << ","  << euler[5] << ")" << endl;
  
  // convert angles from deg to rad
  for (unsigned int i = 3; i <= 5; i++) euler[i] = rad(euler[i]);
  
  if (fileCounter == start)
  {
    read3DScan (scan_in, maxDist, minDist, ptss);
  }
  else
  {
    bool success = createSamplingFromSTL (scan_in, ptss);
    if (!success)
    {
      // terminate similar to missing file
      exit (1);
    }
    if (store_cad)
    {
      std::string stem;
      // get the string of the file-stem according to installed boost API
#if BOOST_VERSION < BOOST_NEW_FS_API
      stem = stl_file.stem ();
#else
      stem = stl_file.stem ().string ();
#endif
      fs::path sample_file (stl_file);
      // is filename scanXXX.3d ?
      if (stem.substr (0, 4).compare ("scan") == 0 &&
          std::atoi (stem.substr (4).c_str ()) == fileCounter)
      {
        // replace the extension according to the installed boost API
#if BOOST_VERSION < BOOST_NEW_FS_API
        sample_file.replace_extension (".3d");
#else
        sample_file.replace_extension (fs::path (".3d"));
#endif
        // the appropriate .pose file already exists
      }
      else
      {
        // generate appropriate filename
        std::string filename = "scan" + to_string (fileCounter, 3) + ".3d";
        sample_file.remove_filename ();
        sample_file /= filename;

        // copy .pose file
        fs::path orig_pose (stl_file);
        fs::path copied_pose (sample_file);
#if BOOST_VERSION < BOOST_NEW_FS_API
        orig_pose.replace_extension (".pose");
        copied_pose.replace_extension (".pose");
#else
        orig_pose.replace_extension (fs::path (".pose"));
        copied_pose.replace_extension (fs::path (".pose"));
#endif
        fs::copy_file (orig_pose, copied_pose, fs::copy_option::overwrite_if_exists);
      }
      // retrieve the filename according to the installed boost API
#if BOOST_VERSION < BOOST_NEW_FS_API
      storeSamplingTo3D (sample_file.directory_string (), ptss);
#else
      storeSamplingTo3D (sample_file.string (), ptss);
#endif
    }
  }

  

  // close the filestreams
  scan_in.close();
  scan_in.clear();
  pose_in.close();
  pose_in.clear();
  fileCounter++;
  
  return  my_fileNr;
}

/**
  * Helper function to read a regular UOS scan 
  */
void
ScanIO_CAD::read3DScan (std::ifstream &scan_in, int maxDist, int minDist, 
    vector<Point> &points)
{
  // reading regular uos scan (copy & paste from scan_io_uos.cc)
  // overread the first line
  char dummy[255];
  scan_in.getline(dummy, 255);
  double maxDist2 = sqr (maxDist);
  double minDist2 = sqr (minDist);

  while (scan_in.good()) {
    Point p;
    try {
      scan_in >> p;
    } catch (...) {
      break;
    }
    // load points up to a certain distance only
    // maxDist2 = -1 indicates no limitation
    if (maxDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) < maxDist2)
    {
      if (minDist == -1 || sqr(p.x) + sqr(p.y) + sqr(p.z) > minDist2)
      {
        points.push_back(p);
      }
    }
  }
}

/**
  * Helper function to create a 3D point sampling for a given .stl file
  */
bool
ScanIO_CAD::createSamplingFromSTL (std::ifstream &scan_in, 
    vector<Point> &points)
{
  bool success;
  if (hasASCIIHeader (scan_in))
  {
    std::cout << "detected ASCII header, parsing file" << std::endl;
    success = parseASCII (scan_in, points);
  }
  else
  {
    std::cout << "couldn't detect ASCII header, trying to parse binary" 
      << std::endl;
    success = parseBinary (scan_in, points);
  }
  return success;
}

bool
ScanIO_CAD::hasASCIIHeader (std::ifstream &scan_in)
{
  // store current get pointer position
  std::streampos current_pos = scan_in. tellg ();

  bool is_ascii = true;

  /* idea: check for the keywords with have to occur at the beginning 
   * and the end of an  ASCII .stl files 
   * (see http://www.ennex.com/~fabbers/StL.asp for further specs) 
   */
  std::string solid ("solid");
  std::string last_line;
  std::string name;
  std::string last_read_string;

  // go to the beginning of the ifstream
  scan_in.seekg (0, std::ios::beg);
  // read the first string
  scan_in >> last_read_string;
  if (solid.compare (last_read_string) != 0)
  {
    is_ascii = false;
  }
  if (is_ascii)
  {
    // read the next string (i.e. name of the stl)
    scan_in >> name;
    // generate expected last line
    last_line = "endsolid " + name;
    // set the get pointer to the correct position
    scan_in.seekg (-last_line.length () - 2, std::ios::end);
    scan_in >> last_read_string; 
    if (last_read_string.compare ("endsolid") != 0)
    {
      is_ascii = false;
    }
    if (is_ascii)
    {
      scan_in >> last_read_string;
      if (last_read_string.compare (name) != 0)
      {
        is_ascii = false;
      }
    }
  }

  // reset the position of the get pointer to its original position
  scan_in.seekg (current_pos, std::ios::beg);
  return is_ascii;
}

bool
ScanIO_CAD::parseASCII (std::ifstream &scan_in, vector<Point> &points)
{
  float vertex1[3];
  float vertex2[3];
  float vertex3[3];
  float raw_data[3];

  unsigned int line_nr = 0;
  unsigned int face_counter = 0;

  float* vertices[3];
  vertices[0] = vertex1;
  vertices[1] = vertex2;
  vertices[2] = vertex3;

  bool success = true;
  bool reached_end = false;
  
  // retrieve current pos of get pointer
  std::ios::streampos current_pos = scan_in.tellg ();

  // set get pointer to beginning of file
  scan_in.seekg (0, std::ios::beg);

  std::string read_key;
  std::string tmp;

  // ignore first line, since in hasASCIIHeader ()
  std::getline (scan_in, read_key);
  line_nr++;

  while (scan_in.good ())
  {
    // read first string at current position
    scan_in >> read_key;
    // determine if end of file reached
    if (read_key.compare ("endsolid") == 0)
    {
      reached_end = true;
      break;
    }
    // otherwise check if a new face begins
    if (read_key.compare ("facet") != 0 ||
        !scan_in.good ())
    {
      // no new face -> misformated .stl
      std::cout << "expected 'facet', contains '" << read_key.substr (0,5)
        << "'" << std::endl;
      success = false;
      break;
    }
    // read the rest of the line (containing the normal)
    getline (scan_in, read_key);
    line_nr++;

    // get the next 2 strings
    scan_in >> read_key;
    scan_in >> tmp;
    read_key += " " + tmp;
    if (read_key.compare ("outer loop") != 0 ||
        !scan_in.good ())
    {
      std::cout << "expected 'outer loop', read '" << read_key 
        << "' instead" << std::endl;
      success = false;
      break;
    }
    // read the rest of the line
    getline (scan_in, read_key);
    line_nr++;

    // read the 3 vertices
    for (unsigned int i = 0; i < 3; ++i)
    {
      scan_in >> read_key;
      line_nr++;
      if (read_key.compare ("vertex") == 0)
      {
        // read x, y and z coordinate in this order
        scan_in >> raw_data[0] >> raw_data[1] >> raw_data[2];
        // scale and order according to the specified settings
        copyRawData2Vertex (raw_data, vertices[i]);
        if (!scan_in.good ())
        {
          success = false;
          break;
        }
      }
      else
      {
        success = false;
        break;
      }
    }
    // retrieve the closing tag for the vertices
    scan_in >> read_key;
    if (read_key.compare ("endloop") != 0 ||
        !scan_in.good ())
    {
      success = false;
      break;
    }
    // retrieve the rest of the current line
    getline (scan_in, read_key);
    line_nr++;
       
    // retrieve the closing tag for the face
    scan_in >> read_key;
    if (read_key.compare ("endfacet") != 0 ||
        !scan_in.good ())
    {
      std::cout << "Expected 'endfacet', read '" << read_key
        << "' instead" << std::endl;
      success = false;
      break;
    }
    // retrieve the rest of the line
    getline (scan_in, read_key);
    line_nr++;
    // increment nr of read faces
    face_counter++;
    // create the sampling for the face
    createSampingForFace (vertex1, vertex2, vertex3, points);
  }

  if (!reached_end)
  {
    success = false;
  }

  // reset position of get pointer
  scan_in.seekg (current_pos, std::ios::beg);

  if (!success)
  {
    std::cerr << "Misformated ASCII .stl file. Parsing error in line "
      << line_nr << std::endl;
  }
  else
  {
    std::cout << "Created " << points.size () << " sample points for " 
      << face_counter << " faces." << std::endl;
  }
  return success;
}

bool
ScanIO_CAD::parseBinary (std::ifstream &scan_in, vector<Point> &points)
{
  // TODO: make eventually compatible to windoze
  uint32_t nr_faces;
  uint32_t face_counter = 0;
  uint16_t attribute;

  float normal[3];
  float vertex1[3];
  float vertex2[3];
  float vertex3[3];
  float raw_data[3];

  bool success;

  size_t header_size = 80; // defined in the binary .stl standard
  char* header = new char[header_size];

  // retrieve the current get pointer position
  std::streampos current_pos = scan_in.tellg ();

  // jump to begin of file
  scan_in.seekg (0, std::ios::beg);

  if (scan_in.is_open ())
  {
    // read the header (unimportant for the sampling)
    scan_in.read (header, header_size);

    // read nr of faces contained in stl file
    scan_in.read ((char*) &nr_faces, 4);

    // try to read all faces
    for (face_counter = 0; face_counter < nr_faces; ++face_counter)
    {
      if (!scan_in.good ())
      {
        break;
      }

      // read face normal (which we don't need)
      scan_in.read ((char*) &normal, 12);

      // read the first vertex 
      scan_in.read ((char*) &raw_data, 12);
      copyRawData2Vertex (raw_data, vertex1);

      // read the second vertex 
      scan_in.read ((char*) &raw_data, 12);
      copyRawData2Vertex (raw_data, vertex2);

      // read the first vertex 
      scan_in.read ((char*) &raw_data, 12);
      copyRawData2Vertex (raw_data, vertex3);

      // read the attribute (which we don't need)
      scan_in.read ((char*) &attribute, 2);

      // create the sampling for the current face
      createSampingForFace (vertex1, vertex2, vertex3, points);
    }
  }

  // check if indeed the specified number of faces could be read
  if (face_counter == nr_faces)
  {
    success = true;
    std::cout << "Created " << points.size () << " sample points for "
      << nr_faces << " faces." << std::endl;
  }
  else
  {
    std::cerr << "Error parsing binary .stl file. Expected " << nr_faces
      << " faces but could read only " << face_counter << std::endl;
    success = false;
  }

  // reset the get pointer position;
  scan_in.seekg (current_pos, std::ios::beg);

  delete [] header;
  return success;
}

void
ScanIO_CAD::createSampingForFace (float* vertex1, float* vertex2,
    float* vertex3, vector<Point> &points)
{
  /*
   * Random sampling approach to sample a triangle:
   * First compute the area of the triangle and decide how many sample
   * points are needed; afterwards randomly create a number of points
   * inside the triangle
   */

  // get the area of the triangle
  float vector12[3];
  float vector13[3];
  float vector23[3];

  for (unsigned int i = 0; i < 3; ++i)
  {
    vector12[i] = vertex2[i] - vertex1[i];
    vector13[i] = vertex3[i] - vertex1[i];
    vector23[i] = vertex3[i] - vertex2[i];
  }

  float a, b, c;
  a = b = c = 0.0f;

  a = dotProd (vector12, vector12);
  b = dotProd (vector13, vector13);
  c = dotProd (vector23, vector23);
  
  a = sqrt (a);
  b = sqrt (b);
  c = sqrt (c);

  float s = a + b + c;
  s /= 2.0f;

  float area = sqrt (s * (s - a) * (s - b) * (s - c));

  unsigned int nr_sample_points, nr_added_points;

  nr_sample_points = area / (sample_point_dist * sample_point_dist);
  nr_added_points = 0;

  if (nr_sample_points > 3)
  {
    points.reserve (points.size () + nr_sample_points);

    float u, v;
    float point_in_triangle[3];

    while (nr_added_points < nr_sample_points)
    {
      // randomly generate scalar u and v in [0,1]
      u = (float) std::rand () / (float) RAND_MAX;
      v = (float) std::rand () / (float) RAND_MAX;

      // check if u and v are smaller than 1.0 (i.e. point is in triangle)
      if (u + v <= 1.0f)
      {
        for (unsigned int i = 0; i < 3; ++i)
        {
          point_in_triangle[i] = vertex1[i] + u * vector12[i] + v * vector13[i];
        }
        points.push_back (Point (point_in_triangle[0], 
              point_in_triangle[1], point_in_triangle[2]));
        nr_added_points++;
      }
    }
  }
  // for a small face just add the corners to the sample points
  else
  {
    points.reserve (points.size () + 3);
    points.push_back (Point (vertex1[0], vertex1[1], vertex1[2]));
    points.push_back (Point (vertex2[0], vertex2[1], vertex2[2]));
    points.push_back (Point (vertex3[0], vertex3[1], vertex3[2]));
  }
}

void
ScanIO_CAD::storeSamplingTo3D (std::string filename, vector<Point> &points)
{
  std::ofstream sample_file (filename.c_str ());

  // write header
  sample_file << "# artificial scan from cad model '" << filename 
    << "', containing " << points.size () << " sample points" << std::endl;
  // write all points
  vector<Point>::const_iterator it = points.begin ();
  while (it != points.end ())
  {
    sample_file << *it++ << std::endl;
  }

  sample_file.flush ();
  sample_file.close ();
}

float
ScanIO_CAD::dotProd (const float* vec1, const float* vec2)
{
  return vec1[0] * vec2[0] + vec1[1] * vec2[1] + vec1[2] * vec2[2];
}

void
ScanIO_CAD::parseConfig (std::ifstream &config)
{
  try
  {
    po::store (po::parse_config_file (config, _op_descr), _vmap);
  }
  catch (std::exception &e)
  {
    std::cout << "illegal parameter in config file:" << std::endl
      << e.what () << std::endl << std::endl;
    std::cout << "The following options are available:" << std::endl;
    std::cout << _op_descr << std::endl;
    config.close ();
    exit (-1);
  }
  catch (...)
  {
    std::cerr << "this line shouldn't have been reached ever" << std::endl;
    config.close ();
    exit (-2);
  }

  po::notify (_vmap);

  if (_vmap.count ("help"))
  {
    std::cout << _op_descr << std::endl;
    // when help is enabled we'll just exit
    exit (1);
  }
  std::cout << "'store_cad' ";
  if (store_cad)
    std::cout << "was set to true";
  else
    std::cout << "was set to false";
  std::cout << endl;

  if (x_index == y_index)
  {
    std::cout << "Warning: x_index = y_index = " << x_index << std::endl;
  }
  if (x_index == z_index)
  {
    std::cout << "Warning: x_index = z_index = " << x_index << std::endl;
  }
  if (y_index == z_index)
  {
    std::cout << "Warning: y_index = z_index = " << y_index << std::endl;
  }

  if (fabs (x_scale - STD_X_SCALE) > std::numeric_limits<float>::epsilon () ||
      fabs (y_scale - STD_Y_SCALE) > std::numeric_limits<float>::epsilon () ||
      fabs (z_scale - STD_Z_SCALE) > std::numeric_limits<float>::epsilon ())
  {
    this->individual_scale = true;
  }
  else
  {
    this->individual_scale = false;
  }

  config.close ();
}


void
ScanIO_CAD::copyRawData2Vertex (float *raw_data, float *vertex)
{
  // copy the raw data to the desired positions
  vertex[0] = raw_data[x_index];
  vertex[1] = raw_data[y_index];
  vertex[2] = raw_data[z_index];

  // any axis to be inverted in the data converted to scan coords?
  if (invert_x)
    vertex[0] *= -1.0f;
  if (invert_y)
    vertex[1] *= -1.0f;
  if (invert_z)
    vertex[2] *= -1.0f;

  // scale with individual scale_factor
  if (individual_scale)
  {
    vertex[0] *= x_scale;
    vertex[1] *= y_scale;
    vertex[2] *= z_scale;
    // scale additionally with general scale factor
    if (combine_scales)
    {
      vertex[0] *= scale_factor;
      vertex[1] *= scale_factor;
      vertex[2] *= scale_factor;
    }
  }
  // scale with the general scale factor
  else
  {
    vertex[0] *= scale_factor;
    vertex[1] *= scale_factor;
    vertex[2] *= scale_factor;
  }
}

void
ScanIO_CAD::setUpOptionDescription (void)
{
  // just something for a little nicer output formating
  std::stringstream x_ss;
  x_ss << STD_X_SCALE;
  std::stringstream y_ss;
  y_ss << STD_Y_SCALE;
  std::stringstream z_ss;
  z_ss << STD_Z_SCALE;
  std::stringstream scale_ss;
  scale_ss << STD_SCALE_FACTOR;

  // set up the options descriptions
  _op_descr.add_options ()
    ("help", "produce this help message")
    ("store_cad",
     po::value<bool> (&store_cad)->default_value (STD_STORE_CAD),
     "determine storage of sampled cad")
    ("sample_dist",
     po::value<float> (&sample_point_dist)->default_value (STD_SAMPLE_DIST),
     "approximate distance of sample points")
    ("random_sampling",
     po::value<bool> (&random_sampling)->default_value (STD_RND_SAMP),
     "enables/diables random surface sampling")
    ("x_index",
     po::value<unsigned int> (&x_index)->default_value (STD_X_INDEX),
     "array index for the x coordinate (default should work for UOS scans)")
    ("y_index",
     po::value<unsigned int> (&y_index)->default_value (STD_Y_INDEX),
     "array index for the y coordinate (default should work for UOS scans)")
    ("z_index",
     po::value<unsigned int> (&z_index)->default_value (STD_Z_INDEX),
     "array index for the z coordinate (default should work for UOS scans)")
    ("scale_factor",
     po::value<float> (&scale_factor)->default_value (STD_SCALE_FACTOR,
       scale_ss.str ()),
     "set general scale factor for x-, y- and z-axis")
    ("x_scale",
     po::value<float> (&x_scale)->default_value (STD_X_SCALE, x_ss.str ()),
     "scale factor to convert the x coordinate of the CAD models to scan scale")
    ("y_scale",
     po::value<float> (&y_scale)->default_value (STD_Y_SCALE, y_ss.str ()),
     "scale factor to convert the y coordinate of the CAD models to scan scale")
    ("z_scale",
     po::value<float> (&z_scale)->default_value (STD_Z_SCALE, z_ss.str ()),
     "scale factor to convert the z coordinate of the CAD models to scan scale")
    ("combine_scales",
     po::value<bool> (&combine_scales)->default_value (STD_COMBINE_SCALES),
     "enables combining of general scale factor and individual scale factors")
    ("invert_x",
     po::value<bool> (&invert_x)->default_value (STD_INVERT_AXIS),
     "inverts the direction of the CAD models x-axis")
    ("invert_y",
     po::value<bool> (&invert_y)->default_value (STD_INVERT_AXIS),
     "inverts the direction of the CAD models y-axis")
    ("invert_z",
     po::value<bool> (&invert_z)->default_value (STD_INVERT_AXIS),
     "inverts the direction of the CAD models z-axis")
    ;
}

void
ScanIO_CAD::parseDirForAllSTL (std::string &directory_name)
{
  // make sure nothing is stored in the stl file vector
  _stl_files.clear ();

  // get a directory iterator to mark the end of the directory
  fs::path dir (directory_name);
  fs::directory_iterator end;

  // iterate over the complete directory
  for (fs::directory_iterator pos (dir); pos != end; ++pos)
  {
    if (fs::is_regular_file (*pos))
    {
      std::string extension;
      // retrieve the extension string according to the installed boost API
#if BOOST_VERSION < BOOST_NEW_FS_API
      extension = pos->path ().extension ();
#else
      extension = pos->path ().extension ().string ();
#endif
      std::for_each (extension.begin (), extension.end (), ::tolower);

      // add all stl files into the vector
      if (extension.compare (".stl") == 0)
      {
        _stl_files.push_back ((*pos));
      }
    }
  }
  _stl_files_pos = 0;
}


bool
ScanIO_CAD::getNextSTLFromDir (fs::path &stl_file)
{
  if (_stl_files_pos < _stl_files.size ())
  {
    stl_file = _stl_files[_stl_files_pos++];
    return true;
  }
  // return an empty path
  stl_file = fs::path ();
  return false;
}


/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) ScanIO* create()
#else
extern "C" ScanIO* create()
#endif
{
  return new ScanIO_CAD;
}


/**
 * class factory for object construction
 *
 * @return Pointer to new object
 */
#ifdef _MSC_VER
extern "C" __declspec(dllexport) void destroy(ScanIO *sio)
#else
extern "C" void destroy(ScanIO *sio)
#endif
{
  delete sio;
}

#ifdef _MSC_VER
BOOL APIENTRY DllMain(HANDLE hModule, DWORD dwReason, LPVOID lpReserved)
{
	return TRUE;
}
#endif
