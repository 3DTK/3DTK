#include <map>

#include <boost/regex.hpp>

#include "show/program_options.h"

using namespace boost::program_options;

void parse_args(int argc, char **argv, dataset_settings& ds, window_settings& ws, bool *directory_present) {
  using namespace std;

  // Temporary parsing variables
  bool no_points, no_cameras, no_path, no_poses, no_fog, no_animcolor, no_anim_convert_jpg;

  // TODO make all defaults declared here the initial values for the settings structs, then use that initial value as the default here
  options_description gui_options("GUI options");
  setGUIOptions(ws.nogui, ws.max_fps, ws.dimensions, ws.advanced_controls,
		ws.invert_mouse_x, ws.invert_mouse_y,
		ws.capture_mouse, ws.hide_widgets, gui_options);

  options_description display_options("Display options");
  setDisplayOptions(ds.scale, ds.camera.fov, ds.init_with_viewmode,
		    no_points, no_cameras, no_path, no_poses,
		    no_fog, ds.fog.type, ds.fog.density,
		    ds.camera.position, ds.camera.rotation, ds.pointsize,
		    display_options);

  options_description color_options("Point coloring");
  setColorOptions(ds.coloring.bgcolor, ds.coloring.explicit_coloring,
  		  ds.coloring.colormap, ds.coloring.colormap_values.min,
  		  ds.coloring.colormap_values.max,
		  ds.coloring.scans_colored, no_animcolor,
  		  color_options);

  options_description scan_options("Scan selection");
  setScanOptions(ds.use_scanserver, ds.scan_numbers.min,
		 ds.scan_numbers.max, ds.format, scan_options);

  options_description reduction_options("Point reduction");
  setReductionOptions(ds.distance_filter.min, ds.distance_filter.max,
		      ds.octree_reduction_voxel,
		      ds.octree_reduction_randomized_bucket,
		      ds.skip_files, reduction_options);

  options_description point_options("Point transformation");
  setPointOptions(ds.origin_type, ds.sphere_radius, point_options);

  options_description file_options("Octree caching");
  setFileOptions(ds.save_octree, ds.load_octree, ds.cache_octree,
		 file_options);

  options_description other_options("Other options");
  setOtherOptions(ws.take_screenshot, ds.objects_file_name,
		  ds.custom_filter, no_anim_convert_jpg,
		  ds.trajectory_file_name, ds.identity,
		  other_options);
  
  // These options will be displayed in the help text
  options_description visible_options("");
  visible_options
    .add(gui_options)
    .add(display_options)
    .add(color_options)
    .add(scan_options)
    .add(reduction_options)
    .add(point_options)
    .add(file_options)
    .add(other_options)
    ;

  options_description cmdline_options("");
  cmdline_options.add(visible_options);
  cmdline_options.add_options()
    ("input-dir", value(&ds.input_directory), "Scan directory")
    ;

  positional_options_description pd;
  pd.add("input-dir", 1);

  // Parse the options into this map
  variables_map vm;

  // First parse, but we are only interested in the input directory
  store(
    command_line_parser(argc, argv)
      .positional(pd)
      .options(cmdline_options)
      .run()
    , vm);

  // Parse user config file

  string config_home = getConfigHome();

  ifstream user_config_file((config_home + "/3dtk/show.ini").c_str());
  if (user_config_file) {
    store(parse_config_file(user_config_file, visible_options), vm);
  }

  // Now we need may need the passed directory
  notify(vm);

  // Parse ./config.ini file in the input directory

  if (vm.count("input-dir")) {
    ifstream local_config_file((ds.input_directory + "/config.ini").c_str());
    if (local_config_file) {
      store(parse_config_file(local_config_file, visible_options), vm);

      // Command line options now overwrite ./config.ini file
      store(
        command_line_parser(argc, argv)
          .options(cmdline_options)
          .run()
        , vm);
      notify(vm);
    }
  }

  // Help text
  if (vm.count("help")) {
    cout << "Usage: " << argv[0] << " [options] <input-dir>" << endl;
    cout << visible_options << endl;
    exit(0);
  }

  if (directory_present == nullptr && vm.count("input-dir") != 1) {
    cerr << "Error: Please specify a directory. See --help for options." << endl;
    exit(1);
  }

  // Scan number range
  if (ds.scan_numbers.min < 0) {
    throw logic_error("Cannot start at a negative scan number.");
  }
  if (ds.scan_numbers.max < -1) {
    throw logic_error("Cannot end at a negative scan number.");
  }
  if (0 < ds.scan_numbers.max && ds.scan_numbers.max < ds.scan_numbers.min) {
    throw logic_error("<end> (" + to_string(ds.scan_numbers.max) + ") cannot be smaller than <start> (" + to_string(ds.scan_numbers.min) + ").");
  }

  // cache_octree implies load_octree and save_octree
  if (ds.cache_octree) {
    ds.load_octree = true;
    ds.save_octree = true;
  }

  // Set drawing options from flags
  ds.draw_points      = !no_points;
  ds.draw_cameras     = !no_cameras;
  ds.draw_path        = !no_path;
  ds.draw_poses       = !no_poses;
  ds.color_animation  = !no_animcolor;
  ds.anim_convert_jpg = !no_anim_convert_jpg;
  if (no_fog) ds.fog.type = 0;

  // Bitset for initializing PointTypes
  unsigned int types = PointType::USE_NONE;

  // RGB formats imply colored points
  switch (ds.format) {
    case UOS_RGB:
    case UOS_RRGBT:
    case RIEGL_RGB:
    case XYZ_RGB:
    case KS_RGB:
      types |= PointType::USE_COLOR;
      break;
    default:
      break;
  }

  // Coloring bool_switch names mapped to their PointType bit,
  // in the order in which they should be the default coloring
  std::vector<std::pair<string, unsigned int>> point_type_flags {
    {"time",        PointType::USE_TIME},
    {"amplitude",   PointType::USE_AMPLITUDE},
    {"deviation",   PointType::USE_DEVIATION},
    {"height",      PointType::USE_HEIGHT},
    {"reflectance", PointType::USE_REFLECTANCE},
    {"temperature", PointType::USE_TEMPERATURE},
    {"type",        PointType::USE_TYPE},
    {"color",       PointType::USE_COLOR}
  };

  // These are the names for values of listboxColorVal
  std::array<string, 6> colorval_names {"height", "reflectance", "temperature", "amplitude", "deviation", "type"};

  for (auto const &kv_pair : point_type_flags) {
    if (vm[kv_pair.first].as<bool>()) {
      // Translate color bool_switches to a bitset
      types |= kv_pair.second;

      // remember the most important one as default for listboxColorVal
      auto colorval_it = std::find(colorval_names.begin(), colorval_names.end(), kv_pair.first);
      if (colorval_it != colorval_names.end()) {
        ds.coloring.colorval = colorval_it - colorval_names.begin();
      }
    }
  }

  ds.coloring.ptype = PointType(types);

  if (vm.count("origin")) {
    ds.origin_type_set = true;
  }

  const char separator =
#ifdef _MSC_VER
  '\\';
#else
  '/';
#endif

  if (vm.count("input-dir")) {
    if (ds.input_directory.back() != separator) {
      ds.input_directory += separator;
    }
  }

  if (directory_present != nullptr) {
    *directory_present = vm.count("input-dir") != 0;
  }
}

std::string getConfigHome()
{
  std::string config_home;
#ifndef _MSC_VER
  // in $XDG_CONFIG_HOME/3dtk/show.ini
  char *home_c = getenv("HOME");
  char *config_home_c = getenv("XDG_CONFIG_HOME");
  if (config_home_c && *config_home_c != '\0') {
    config_home = config_home_c;
  } else {
    config_home = std::string(home_c) + "/.config";
  }
#else
  // in %APPDATA%/3dtk/show.ini
  config_home = std::string(getenv("APPDATA"));
#endif

  return config_home;
}

void setGUIOptions(bool& nogui, float& fps,
		   WindowDimensions& dimensions, bool& advanced,
		   bool& invertMouseX, bool& invertMouseY,
		   bool& captureMouse, bool &hideWidgets,
		   options_description& gui_options)
{
  gui_options.add_options()
    ("nogui,G", bool_switch(&nogui), "Turn off GUI")
    ("fps,F", value(&fps)->default_value(60), "Maximum framerate")
    ("dimensions,x",
     value(&dimensions)->default_value(WindowDimensions(960, 540), "960x540"), "Window dimensions in WxH format.")
    ("advanced,2", bool_switch(&advanced),
     "Switch on advanced controls")
    ("invertmousex", bool_switch(&invertMouseX),
     "Invert camera rotation for mouse x movement.")
    ("invertmousey", bool_switch(&invertMouseY),
     "Invert camera rotation for mouse y movement.")
    ("capturemouse", bool_switch(&captureMouse),
     "Capture mouse cursor")
    ("hidewidgets", bool_switch(&hideWidgets)->default_value(true),
     "Hide widgets in fullscreen")
    ;
}

void setDisplayOptions(double& scale, GLfloat& fov, int& viewmode,
		       bool& noPoints, bool& noCameras, bool& noPath, bool& noPoses,
		       bool& noFog, int& fogType, GLfloat& fogDensity,
		       Position& position, Quaternion& rotation,
		       int& pointsize,
		       options_description& display_options)
{
  display_options.add_options()
    ("scale,C", value(&scale)->default_value(0.01, "0.01"),
     "Scale factor to use. Influences movement speed etc. "
     "Use 1 when point coordinates are in meters, 0.01 when in centimeters "
     "and so forth.")
    ("fov", value(&fov)->default_value(60),
     "Horizontal field of view angle in degrees. "
     "The vertical angle depends on window size.")
    ("viewmode", value(&viewmode)->default_value(0), // FIXME starts out with a view from the front, need to call topView()
     "Initialize the camera above the point cloud with parallel projection.")
    ("no-points", bool_switch(&noPoints),
     "Initially, do not draw points.")
    ("no-cameras", bool_switch(&noCameras),
     "Initially, do not draw cameras.")
    ("no-path", bool_switch(&noPath),
     "Initially, do not draw camera path.")
    ("no-poses", bool_switch(&noPoses),
     "Initially, do not draw poses.")
    ("no-fog", bool_switch(&noFog),
     "Initially turn off fog.")
    ("fog-type", value(&fogType)->default_value(1),
     "How fog dims points with distance:\n"
     "0   = no fog\n"
     "1   = exponential\n"
     "2   = exponential squared\n"
     "3   = linear\n"
     "4-6 = inverted options 1-3 (further is brighter)") // FIXME? during testing, these did not do anything
    ("fog-density", value(&fogDensity)->default_value(0.1),
     "Fog density. Useful values are between 0 and 1.")
    ("position",
     value(&position)->default_value(Position(0,0,0), "0,0,0"),
     "Camera starting position, given as \"%lf,%lf,%lf\" for x, y and z.")
    ("rotation",
     value(&rotation)->default_value(Quaternion(1,0,0,0), "1,0,0,0"),
     "Camera starting rotation, given as a quaternion \"%lf,%lf,%lf,%lf\" for x, y, z, and w.")
    ("pointsize", value(&pointsize)->default_value(1),
     "Size of each point in pixels.")
    ;
}

void setColorOptions(Color& bgcolor, bool& color, ShowColormap& colormap,
		     float& colormin, float& colormax,
		     int& scansColored, bool& noAnimColor,
		     options_description& color_options)
{
  color_options.add_options()
    ("bgcolor", value(&bgcolor)->default_value(Color(0,0,0), "0,0,0"),
     "Drawing area background color, given as \"%f,%f,%f\" for red, green and blue.")
    ("color,c", bool_switch(&color),
     "Use included RGB values for coloring points.")
    ("reflectance,R", bool_switch(),
     "Use reflectance values for coloring point clouds.")
    ("temperature,D", bool_switch(),
     "Use temperature values for coloring point clouds.")
    ("amplitude,a", bool_switch(),
     "Use amplitude values for coloring point clouds.")
    ("deviation,d", bool_switch(),
     "Use deviation values for coloring point clouds.")
    ("height,h", bool_switch(),
     "Use y-height values for coloring point clouds.")
    ("type,T", bool_switch(),
     "Use type values for coloring point clouds.")
    ("time,t", bool_switch()) // TODO description
    ("colormap",
     value(&colormap)->default_value(ShowColormap::solid, "solid"),
     "With which colors to color the points, according to their color value "
     "in a spectrum. Available color maps are: solid, grey, hsv, jet, hot, "
     "rand, shsv, temp.")
    ("colormin", value(&colormin),
     "Minimum value for mapping the color spectrum.")
    ("colormax", value(&colormax),
     "Maximum value for mapping the color spectrum.")
    ("scanscolored", value(&scansColored),
     "Scans colored")
    ("noanimcolor,A", bool_switch(&noAnimColor),
     "Do not switch to different color settings when displaying animation")
    ;
}

void setScanOptions(bool& scanserver, int& start, int& end,
		    IOType& format, options_description& scan_options)
{
  scan_options.add_options()
    ("scanserver,S", bool_switch(&scanserver),
     "Use the scanserver as an input method and for handling scan data.")
    ("start,s", value(&start)->default_value(0), "Start at this scan number (0-based)")
    ("end,e", value(&end)->default_value(-1), "Stop at this scan number (0-based, with -1 meaning don't stop)")
    ("format,f", value(&format)->default_value(UOS, "uos"),
     "The input files are read with this shared library.\n"
     "Available values: uos, uos_map, uos_rgb, uos_frames, uos_map_frames, "
     "old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply, "
     "wrl, xyz, zuf, iais, front, x3d, rxp, ais.")
    ;
}

void setReductionOptions(int& distMin, int& distMax, double& reduce,
			 int& octree, int& stepsize,
			 options_description& reduction_options)
{
  reduction_options.add_options()
    ("min,M", value(&distMin)->default_value(0),
     "Neglect all points closer than this to the origin")
    ("max,m", value(&distMax)->default_value(-1),
     "Neglect all points further than this from the origin (with -1 meaning not to)")
    ("reduce,r", value(&reduce)->default_value(0),
     "Turn on octree based point reduction with voxels  of size arg^3.")
    ("octree,O", value(&octree)->implicit_value(1),
     "Enable randomized octree based point reduction with arg points per voxel. "
     "Requires --reduce (-r).") // TODO where is this enforced?
    ("stepsize", value(&stepsize)->default_value(1),
     "Reduce point cloud by including only every arg-th scan file.")
    ;
}

void setPointOptions(int& originType, double& sphereRadius,
		     options_description& point_options)
{
  point_options.add_options()
    ("origin,o", value(&originType),
      "Set the starting and reset position according to n. By default, the "
      "starting and reset position are at the origin of the "
      "coordinate system. Also try --position.\n"
      "arg = 0             : the center of mass of all scans\n"
      "arg = [1,2,3,...]   : the center of scan 0,1,2,... (arg-1)\n"
      "arg = [-1,-2,-3,...]: the position of scan 0,1,2,... (-arg-1)\n")
    ("sphere,b", value(&sphereRadius)->default_value(0),
      "Map all measurements on a sphere of this radius.")
    ;
}

void setFileOptions(bool& saveOct, bool& loadOct, bool& autoOct,
		    options_description& file_options)
{
  file_options.add_options()
    ("saveOct", bool_switch(&saveOct),
      "Store all used scans as octrees in the input directory. "
      "All reflectivity/amplitude/deviation/type settings are stored as well. "
      "Only works when using octree display.")
    ("loadOct", bool_switch(&loadOct),
      "Only reads octrees from the given directory. "
      "All reflectivity/amplitude/deviation/type settings are read from file. "
      "--reflectance/--amplitude and similar parameters are therefore ignored. "
      "Only works when using octree display.")
    ("autoOct", bool_switch(&autoOct),
      "Like --loadOct and --saveOct used together except that it only loads "
      "the octrees if they are newer than the underlying data and only stores "
      "octrees if they either didn't exist yet or are older than the "
      "underlying data.")
    ;
}

void setOtherOptions(bool& screenshot, std::string& objFileName,
		     std::string& customFilter,	bool& noAnimConvertJPG,
		     std::string& trajectoryFileName, bool& identity,
		     options_description& other_options)
{
  other_options.add_options()
    ("help,?", "Display this help text")
    ("screenshot", bool_switch(&screenshot), "Take screenshot and exit")
    ("loadObj,l", value(&objFileName),
      "Load objects specified in this file")
    ("customFilter,u", value(&customFilter),
      "Apply a custom filter. Filter mode and data are specified as a "
      "semicolon-seperated string:\n"
      "\"{filterMode};{nrOfParams}[;param1][;param2][...]\"\n"
      "Multiple filters can be specified in a file (syntax in file is same as "
      "direct specification)\n"
      "\"FILE;{fileName}\"\n"
      "See filter implementation in src/slam6d/pointfilter.cc for more detail.")
    ("no-anim-convert-jpg,J", bool_switch(&noAnimConvertJPG)) // TODO description
    ("trajectory-file", value(&trajectoryFileName)) // TODO description
    ("identity,i", bool_switch(&identity)) //TODO description
    ;
}

void validate(boost::any& v, const std::vector<std::string>& values,
  WindowDimensions* target_type, int)
{
  validators::check_first_occurrence(v);
  const char *s = validators::get_single_string(values).c_str();

  int w, h;
  if (sscanf(s, "%dx%d", &w, &h) == 2) {
    v = boost::any(WindowDimensions(w, h));
  } else {
    throw validation_error(validation_error::invalid_option_value, "Window dimensions must be given in format WxH");
  }
}

void validate(boost::any& v, const std::vector<std::string>& values,
  Position* target_type, int)
{
  validators::check_first_occurrence(v);
  const char *s = validators::get_single_string(values).c_str();

  double x, y, z;
  if (sscanf(s, "%lf,%lf,%lf", &x, &y, &z) == 3) {
    v = boost::any(Position(x, y, z));
  } else {
    throw validation_error(validation_error::invalid_option_value, "Camera coordinates must be given as \"X,Y,Z\".");
  }
}

void validate(boost::any& v, const std::vector<std::string>& values,
  Color* target_type, int)
{
  validators::check_first_occurrence(v);
  const char *s = validators::get_single_string(values).c_str();

  float r, g, b;
  if (sscanf(s, "%f,%f,%f", &r, &g, &b) == 3) {
    v = boost::any(Color(r, g, b));
  } else {
    throw validation_error(validation_error::invalid_option_value, "Colors must be given as \"R,G,B\".");
  }
}

void validate(boost::any& v, const std::vector<std::string>& values,
  Quaternion* target_type, int)
{
  validators::check_first_occurrence(v);
  const char *s = validators::get_single_string(values).c_str();

  double x, y, z, w;
  if (sscanf(s, "%lf,%lf,%lf,%lf", &x, &y, &z, &w) == 4) {
    v = boost::any(Quaternion(x, y, z, w));
  } else {
    throw validation_error(validation_error::invalid_option_value, "Camera rotation must be given as \"X,Y,Z,W\"");
  }
}

void validate(boost::any& v, const std::vector<std::string>& values,
  ShowColormap* target_type, int)
{
  validators::check_first_occurrence(v);
  const std::string& s = validators::get_single_string(values);

  std::map<std::string, ShowColormap> colormap_values({
    {"solid", ShowColormap::solid},
    {"grey", ShowColormap::grey},
    {"hsv", ShowColormap::hsv},
    {"jet", ShowColormap::jet},
    {"hot", ShowColormap::hot},
    {"rand", ShowColormap::rand},
    {"shsv", ShowColormap::shsv},
    {"temp", ShowColormap::temp}
  });

  try {
    v = boost::any(colormap_values.at(s));
  } catch (std::out_of_range) {
    throw validation_error(validation_error::invalid_option_value, "Unknown colormap: " + s);
  }
  // FIXME also need to call changeColorMap after some unknown thing is initialized
}

void validate(boost::any& v, const std::vector<std::string>& values,
  IOType* target_type, int)
{
  validators::check_first_occurrence(v);
  const std::string& s = validators::get_single_string(values);

  try {
    v = boost::any(formatname_to_io_type(s.c_str()));
  } catch (const std::runtime_error &e) {
    throw validation_error(validation_error::invalid_option_value, "Error due to --format (" + s + "): " + e.what());
  }
}
