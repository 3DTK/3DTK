#include <map>

#include <boost/regex.hpp>

#include "show/program_options.h"

void parse_args(int argc, char **argv, dataset_settings& ds, window_settings& ws, bool *directory_present) {
  using namespace boost::program_options;
  using namespace std;

  // Temporary parsing variables
  bool no_points, no_cameras, no_path, no_fog, no_animcolor, no_anim_convert_jpg;

  // TODO make all defaults declared here the initial values for the settings structs, then use that initial value as the default here
  options_description gui_options("GUI options");
  gui_options.add_options()
    ("nogui,G", bool_switch(&ws.nogui),
      "Turn off GUI")
    ("fps,F", value(&ws.max_fps)->default_value(60), "Maximum framerate")
    ("dimensions,x",
       value(&ws.dimensions)->default_value(WindowDimensions(960, 540), "960x540"),
      "Window dimensions in WxH format.")
    ("advanced,2", bool_switch(&ws.advanced_controls),
      "Switch on advanced controls")
    ;

  options_description display_options("Display options");
  display_options.add_options()
    ("scale,C", value(&ds.scale)->default_value(0.01, "0.01"),
      "Scale factor to use. Influences movement speed etc. "
      "Use 1 when point coordinates are in meters, 0.01 when in centimeters "
      "and so forth.")
    ("fov", value(&ds.camera.fov)->default_value(60),
      "Horizontal field of view angle in degrees. "
      "The vertical angle depends on window size.")
    ("topview", bool_switch(&ds.init_with_topview), // FIXME starts out with a view from the front, need to call topView()
      "Initialize the camera above the point cloud with parallel projection.")
    ("no-points", bool_switch(&no_points),
      "Initially, do not draw points.")
    ("no-cameras", bool_switch(&no_cameras),
      "Initially, do not draw cameras.")
    ("no-path", bool_switch(&no_path),
      "Initially, do not draw camera path.")
    ("no-fog", bool_switch(&no_fog),
      "Initially turn off fog.")
    ("fog-type", value(&ds.fog.type)->default_value(1),
      "How fog dims points with distance:\n"
      "0   = no fog\n"
      "1   = exponential\n"
      "2   = exponential squared\n"
      "3   = linear\n"
      "4-6 = inverted options 1-3 (further is brighter)") // FIXME? during testing, these did not do anything
    ("fog-density", value(&ds.fog.density)->default_value(0.1),
      "Fog density. Useful values are between 0 and 1.")
    ("position",
      value(&ds.camera.position)->default_value(Position(0,0,0), "0,0,0"),
      "Camera starting position, given as \"%lf,%lf,%lf\" for x, y, z.")
    ("rotation",
      value(&ds.camera.rotation)->default_value(Quaternion(0,0,0,1), "0,0,0,1"),
      "Camera starting rotation, given as a quaternion \"%lf,%lf,%lf,%lf\".")
    ("pointsize", value(&ds.pointsize)->default_value(1),
      "Size of each point in pixels.")
    ;

  options_description color_options("Point coloring");
  color_options.add_options()
    ("color,c", bool_switch(),
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
      value(&ds.coloring.colormap)->default_value(Colormap::solid, "solid"),
      "With which colors to color the points, according to their color value "
      "in a spectrum. Available color maps are: solid, grey, hsv, jet, hot, "
      "rand, shsv, temp.")
    ("colormin", value(&ds.coloring.colormap_values.min),
      "Minimum value for mapping the color spectrum.")
    ("colormax", value(&ds.coloring.colormap_values.max),
      "Maximum value for mapping the color spectrum.")
    ("noanimcolor,A", bool_switch(&no_animcolor),
      "Do not switch to different color settings when displaying animation")
    ;

  options_description scan_options("Scan selection");
  scan_options.add_options()
    ("scanserver,S", bool_switch(&ds.use_scanserver),
      "Use the scanserver as an input method and for handling scan data.")
    ("start,s", value(&ds.scan_numbers.min)->default_value(0), "Start at this scan number (0-based)")
    ("end,e", value(&ds.scan_numbers.max)->default_value(-1), "Stop at this scan number (0-based, with -1 meaning don't stop)")
    ("format,f", value(&ds.format)->default_value(UOS, "uos"),
      "The input files are read with this shared library.\n"
      "Available values: uos, uos_map, uos_rgb, uos_frames, uos_map_frames, "
      "old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply, "
      "wrl, xyz, zuf, iais, front, x3d, rxp, ais.")
    ;

  options_description reduction_options("Point reduction");
  reduction_options.add_options()
    ("min,M", value(&ds.distance_filter.min)->default_value(0),
      "Neglect all points closer than this to the origin")
    ("max,m", value(&ds.distance_filter.max)->default_value(-1),
      "Neglect all points further than this from the origin (with -1 meaning not to)")
    ("reduce,r", value(&ds.octree_reduction_voxel)->default_value(0),
      "Turn on octree based point reduction with voxels  of size arg^3.")
    ("octree,O", value(&ds.octree_reduction_randomized_bucket)->implicit_value(1),
      "Enable randomized octree based point reduction with arg points per voxel. "
      "Requires --reduce (-r).") // TODO where is this enforced?
    ("stepsize", value(&ds.skip_points)->default_value(1),
      "Reduce point cloud by including only every arg-th point.")
    ;

  options_description point_options("Point transformation");
  point_options.add_options()
    ("origin,o", value(&ds.origin_type),
      "Set the starting and reset position according to n. By default, the "
      "starting and reset position are at the origin of the "
      "coordinate system. Also try --position.\n"
      "arg = 0             : the center of mass of all scans\n"
      "arg = [1,2,3,...]   : the center of scan 0,1,2,... (arg-1)\n"
      "arg = [-1,-2,-3,...]: the position of scan 0,1,2,... (-arg-1)\n")
    ("sphere,b", value(&ds.sphere_mode),
      "Map all measurements on a sphere of this radius.")
    ;

  options_description file_options("Octree caching");
  file_options.add_options()
    ("saveOct", bool_switch(&ds.save_octree),
      "Store all used scans as octrees in the input directory. "
      "All reflectivity/amplitude/deviation/type settings are stored as well. "
      "Only works when using octree display.")
    ("loadOct", bool_switch(&ds.load_octree),
      "Only reads octrees from the given directory. "
      "All reflectivity/amplitude/deviation/type settings are read from file. "
      "--reflectance/--amplitude and similar parameters are therefore ignored. "
      "Only works when using octree display.")
    ("autoOct", bool_switch(&ds.cache_octree),
      "Like --loadOct and --saveOct used together except that it only loads "
      "the octrees if they are newer than the underlying data and only stores "
      "octrees if they either didn't exist yet or are older than the "
      "underlying data.")
    ;

  options_description other_options("Other options");
  other_options.add_options()
    ("help,?", "Display this help text")
    ("screenshot", bool_switch(&ws.take_screenshot), "Take screenshot and exit")
    ("loadObj,l", value(&ds.objects_file_name),
      "Load objects specified in this file")
    ("customFilter,u", value(&ds.custom_filter),
      "Apply a custom filter. Filter mode and data are specified as a "
      "semicolon-seperated string:\n"
      "\"{filterMode};{nrOfParams}[;param1][;param2][...]\"\n"
      "Multiple filters can be specified in a file (syntax in file is same as "
      "direct specification)\n"
      "\"FILE;{fileName}\"\n"
      "See filter implementation in src/slam6d/pointfilter.cc for more detail.")
    ("no-anim-convert-jpg,J", bool_switch(&no_anim_convert_jpg)) // TODO description
    ("trajectory-file", value(&ds.trajectory_file_name)) // TODO description
    ("identity,i", bool_switch(&ds.identity)) //TODO description
    ;

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

  string config_home;
#ifndef _MSC_VER
  // in $XDG_CONFIG_HOME/3dtk/show.ini
  char *home_c = getenv("HOME");
  char *config_home_c = getenv("XDG_CONFIG_HOME");
  if (config_home_c && *config_home_c != '\0') {
    config_home = config_home_c;
  } else {
    config_home = string(home_c) + "/.config";
  }
#else
  // in %APPDATA%/3dtk/show.ini
  config_home = string(getenv("APPDATA"));
#endif

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
  if (vm.count("help") ||
      (directory_present == nullptr && vm.count("input-dir") == 0)) {
    cout << "Usage: " << argv[0] << " [options] <input-dir>" << endl;
    cout << visible_options << endl;
    exit(0);
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

  // Translate color bool_switches to a bitset
  std::map<string, unsigned int> point_type_flags({
    {"reflectance", PointType::USE_REFLECTANCE},
    {"temperature", PointType::USE_TEMPERATURE},
    {"amplitude",   PointType::USE_AMPLITUDE},
    {"deviation",   PointType::USE_DEVIATION},
    {"height",      PointType::USE_HEIGHT},
    {"type",        PointType::USE_TYPE},
    {"color",       PointType::USE_COLOR},
    {"time",        PointType::USE_TIME}
  });

  for (auto const &kv_pair : point_type_flags) {
    if (vm[kv_pair.first].as<bool>()) {
      types |= kv_pair.second;
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

void validate(boost::any& v, const std::vector<std::string>& values,
  WindowDimensions* target_type, int)
{
  using namespace boost;
  static regex r("\\dx\\d");

  using namespace boost::program_options;

  validators::check_first_occurrence(v);
  const std::string& s = validators::get_single_string(values);

  smatch match;
  if (regex_match(s, match, r)) {
    v = boost::any(WindowDimensions(lexical_cast<int>(match[1]),
                                    lexical_cast<int>(match[2])));
  } else {
    throw validation_error(validation_error::invalid_option_value, "Window dimensions must be given in format WxH");
  }
}

void validate(boost::any& v, const std::vector<std::string>& values,
  Position* target_type, int)
{
  using namespace boost;
  static regex r("\\d,\\d,\\d");

  using namespace boost::program_options;

  validators::check_first_occurrence(v);
  const std::string& s = validators::get_single_string(values);

  smatch match;
  if (regex_match(s, match, r)) {
    v = boost::any(Position(lexical_cast<double>(match[1]),
                            lexical_cast<double>(match[2]),
                            lexical_cast<double>(match[3])));
  } else {
    throw validation_error(validation_error::invalid_option_value, "Camera coordinates must be given as \"X,Y,Z\".");
  }
}

void validate(boost::any& v, const std::vector<std::string>& values,
  Quaternion* target_type, int)
{
  using namespace boost;
  static regex r("\\d,\\d,\\d,\\d");

  using namespace boost::program_options;

  validators::check_first_occurrence(v);
  const std::string& s = validators::get_single_string(values);

  smatch match;
  if (regex_match(s, match, r)) {
    v = boost::any(Quaternion(lexical_cast<double>(match[1]),
                              lexical_cast<double>(match[2]),
                              lexical_cast<double>(match[3]),
                              lexical_cast<double>(match[4])));
  } else {
    throw validation_error(validation_error::invalid_option_value, "Camera rotation must be given as \"X,Y,Z,W\"");
  }
}

void validate(boost::any& v, const std::vector<std::string>& values,
  Colormap* target_type, int)
{
  using namespace boost::program_options;

  validators::check_first_occurrence(v);
  const std::string& s = validators::get_single_string(values);

  std::map<std::string, Colormap> colormap_values({
    {"solid", Colormap::solid},
    {"grey", Colormap::grey},
    {"hsv", Colormap::hsv},
    {"jet", Colormap::jet},
    {"hot", Colormap::hot},
    {"rand", Colormap::rand},
    {"shsv", Colormap::shsv},
    {"temp", Colormap::temp}
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
  using namespace boost::program_options;

  validators::check_first_occurrence(v);
  const std::string& s = validators::get_single_string(values);

  try {
    v = boost::any(formatname_to_io_type(s.c_str()));
  } catch (const std::runtime_error &e) {
    throw validation_error(validation_error::invalid_option_value, "Error due to --format (" + s + "): " + e.what());
  }
}
