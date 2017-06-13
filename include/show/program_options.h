#ifndef __PROGRAM_OPTIONS_H__
#define __PROGRAM_OPTIONS_H__

#include <string>

#include <boost/program_options.hpp>
#include <GL/gl.h>

#include "slam6d/io_types.h"
#include "slam6d/point_type.h"

struct WindowDimensions {
  int w, h;
  WindowDimensions() : w(0), h(0) {};
  WindowDimensions(int w, int h) : w(w), h(h) {};
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

enum class Colormap : int {
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
  Colormap colormap;
  range<float> colormap_values {};
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
  int skip_points;

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

void parse_args(int argc, char **argv, dataset_settings& ds, window_settings& ws);

#endif
