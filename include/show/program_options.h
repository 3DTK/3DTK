#ifndef __PROGRAM_OPTIONS_H__
#define __PROGRAM_OPTIONS_H__

#include <cmath>
#include <string>

#include <boost/program_options.hpp>

#ifdef _WIN32
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
#include "slam6d/scan_settings.h"

class GenericProgramOptions
{
public:
  GenericProgramOptions();
  virtual boost::program_options::parsed_options parse(int argc, char **argv, boost::program_options::variables_map &vm);
  boost::program_options::parsed_options parse(int argc, char **argv) {
    boost::program_options::variables_map vm;
    return parse(argc, argv, vm);
  }
  virtual boost::program_options::parsed_options parseConfig(std::string filename, const boost::program_options::options_description &desc, boost::program_options::variables_map &vm);
  inline boost::program_options::parsed_options parseConfig(std::string filename, boost::program_options::variables_map &vm) {
    boost::program_options::options_description visopts = visibleOptions();
    return parseConfig(filename, visopts, vm);
  }
  virtual boost::program_options::options_description visibleOptions();
  virtual void help(std::string program, boost::program_options::options_description opt_desc) {
    std::cout << "Usage: " << program << " [options] <input-dir>" << std::endl;
    std::cout << opt_desc << std::endl;
    exit(0);
  }
  virtual inline void help(std::string program) {
    boost::program_options::options_description visopts = visibleOptions();
    help(program, visopts);
  }

  boost::program_options::options_description other_options;
protected:
  bool m_no_config;
  boost::program_options::variables_map *m_vm;
};

class ScanProgramOptions : public GenericProgramOptions
{
public:
  ScanProgramOptions(dataset_settings& dss, bool* directory_present = nullptr);
  virtual boost::program_options::parsed_options parse(int argc, char **argv, boost::program_options::variables_map &vm);
  virtual boost::program_options::options_description visibleOptions();
  virtual void process();

  boost::program_options::options_description datasource_options;
  boost::program_options::options_description color_options;
  boost::program_options::options_description reduction_options;
  boost::program_options::options_description transform_options;
  boost::program_options::options_description octtree_caching_options;
protected:
  dataset_settings *m_dss;
  bool* m_directory_present;
  std::vector<std::string> m_data_sources;
};

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

struct Camera {
  Position position;
  Quaternion rotation;
  GLfloat fov;
};

struct fog_settings {
  int type;
  GLfloat density;
};

struct display_settings {
  Camera camera;
  int init_with_viewmode;
  int pointsize;

  fog_settings fog;

  bool draw_points;
  bool draw_cameras;
  bool draw_path;
  bool draw_poses;

  bool color_animation;
  bool anim_convert_jpg;

  bool hide_label;
};

struct window_settings {
  bool nogui;
  float max_fps;
  WindowDimensions dimensions;
  bool advanced_controls;
  bool invert_mouse_x, invert_mouse_y;
  bool take_screenshot;
  std::string screenshot_filename;
  bool capture_mouse;
  bool hide_widgets;
};

std::string getConfigHome();

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
class ShowProgramOptions : public ScanProgramOptions
{
public:
  ShowProgramOptions(dataset_settings& dss, window_settings& ws, display_settings& ds, bool* directory_present = nullptr);
  virtual boost::program_options::parsed_options parse(int argc, char **argv, boost::program_options::variables_map &vm);
  virtual boost::program_options::options_description visibleOptions();
  virtual void process();

  boost::program_options::options_description gui_options;
  boost::program_options::options_description display_options;

private:
  bool m_no_points, m_no_cameras, m_no_path, m_no_poses, m_no_fog, m_no_anim_color, m_no_anim_convert_jpg;
  window_settings *m_ws;
  display_settings *m_ds;
};

void parse_show_args(int argc, char **argv, dataset_settings& dss, window_settings& ws, display_settings& ds, bool *directory_present = nullptr);
void setGUIOptions(bool& nogui, float& fps,
		   WindowDimensions& dimensions, bool& advanced,
		   bool& invertMouseX, bool& invertMouseY,
		   bool& capturedMouse, bool& hideWidgetsInFullscreen,
		   boost::program_options::options_description& gui_options);
void setDisplayOptions(double& scale, GLfloat& fov, int& viewmode,
		       bool& noPoints, bool& noCameras, bool& noPath, bool& noPoses,
		       bool& noFog, int& fogType, GLfloat& fogDensity,
		       Position& position, Quaternion& rotation,
		       int& pointsize, boost::program_options::options_description& display_options);
void setColorOptions(Color& bgcolor, bool& color, ShowColormap& colormap,
		     float& colormin, float& colormax,
		     int& scansColored, bool& noAnimColor,
		     boost::program_options::options_description& color_options);
void setScanOptions(bool& scanserver, int& start, int& end,
		    IOType& format, boost::program_options::options_description& scan_options);
void setReductionOptions(double& distMin, double& distMax, double& reduce,
			 int& octree, int& stepsize,
			 boost::program_options::options_description& reduction_options);
void setPointOptions(int& originType, double& sphereRadius, boost::program_options::options_description& point_options);
void setFileOptions(bool& saveOct, bool& loadOct, bool& autoOct, boost::program_options::options_description& file_options);
void setOtherOptions(bool& screenshot, std::string& screenshot_filename, std::string& objFileName,
		     std::string& customFilter,	bool& noAnimConvertJPG,
		     std::string& trajectoryFileName, bool& identity, bool& no_config,
		     boost::program_options::options_description& other_options);

#endif
