#ifndef __PROGRAM_OPTIONS_H__
#define __PROGRAM_OPTIONS_H__

#include <cmath>
#include <string>

#include <boost/program_options.hpp>

#ifdef _MSC_VER
#include "windows.h"
#endif

#ifdef WITH_OPENGL
#ifdef __APPLE__
  #include <OpenGL/gl.h>
#else
  #include <GL/gl.h>
#endif
#else
#include "show/dummygl.h"
#endif

#include "slam6d/io_types.h"
#include "slam6d/point_type.h"

struct WindowDimensions {
  int w, h;
  WindowDimensions() : w(0), h(0) {};
  WindowDimensions(int w, int h) : w(w), h(h) {};
};

struct Color {
  float r, g, b;
  Color() : r(0), g(0), b(0) {};
  Color(float r, float g, float b) : r(r), g(g), b(b) {};
};

struct Position {
  double x, y, z;
  Position() : x(0), y(0), z(0) {};
  Position(double x, double y, double z) : x(x), y(y), z(z) {};
};

struct Quaternion {
  double x, y, z, w;
  Quaternion() : x(0), y(0), z(0), w(1) {};
  Quaternion(double x, double y, double z, double w) : x(x), y(y), z(z), w(w) {};
};

enum class ShowColormap : int {
  solid = 0,
  grey = 1,
  hsv = 2,
  jet = 3,
  hot = 4,
  rand = 5,
  shsv = 6,
  temp = 7
};

struct Camera {
  Position position;
  Quaternion rotation;
  GLfloat fov;
};

struct fog_settings {
  int type;
  GLfloat density;
};

template<class T>
struct range {
  T min, max;
};

// The members initialized with {} can not have a sensible default in parse_args

struct color_settings {
  PointType ptype;
  int colorval = -1;
  bool explicit_coloring;
  ShowColormap colormap;
  range<float> colormap_values {NAN, NAN};
  Color bgcolor;
};

struct dataset_settings {
  std::string input_directory;
  IOType format;
  bool use_scanserver;
  range<int> scan_numbers;

  Camera camera;
  double scale;
  bool init_with_topview;
  int pointsize;

  fog_settings fog;

  color_settings coloring;

  range<int> distance_filter;
  double octree_reduction_voxel;
  int octree_reduction_randomized_bucket {};
  int skip_files;

  // TODO make this an std::optional (C++17)
  int origin_type {};
  bool origin_type_set {};
  bool sphere_mode {};

  bool save_octree;
  bool load_octree;
  bool cache_octree;

  std::string objects_file_name {};
  std::string custom_filter {};
  std::string trajectory_file_name {};
  bool identity;

  bool draw_points;
  bool draw_cameras;
  bool draw_path;

  bool color_animation;
  bool anim_convert_jpg;
};

struct window_settings {
  bool nogui;
  float max_fps;
  WindowDimensions dimensions;
  bool advanced_controls;
  bool take_screenshot;
};

/**
 * Parses arguments to `show`. The arguments come from these sources:
 *  - user config file in ~/.config/3dtk/show.ini
 *  - a file named "config.ini" in the input directory
 *  - command line arguments
 *
 * Config files have an "option=value" pair on each line with option names just
 * like the command line arguments.
 *
 * @param argc the number of arguments
 * @param argv the arguments
 * @param ds the dataset_settings to fill
 * @param ws the window_settings to fill
 * @param directory_present if this pointer is not null, allow input-dir to not be present and write that presence in the target bool
 * @return the parsed options
 */
void parse_args(int argc, char **argv, dataset_settings& ds, window_settings& ws, bool *directory_present = nullptr);

#endif
