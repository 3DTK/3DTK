#include "show/show_common.h"

/**
 * Parses arguments to `show`. The arguments come from these sources:
 *  - user config file in ~/.config/3dtk/show.ini
 *  - a file named "config" in the input directory
 *  - command line arguments
 *
 * Config files have an "option=value" pair on each line with option names just
 * like the command line arguments.
 *
 * @param argc the number of arguments
 * @param argv the arguments
 * @param dir parsing result - the directory
 * @param start parsing result - starting at scan number 'start'
 * @param end parsing result - stopping at scan number 'end'
 * @param maxDist parsing result - maximal distance
 * @param minDist parsing result - minimal distance
 * @param red parsing result - reduce points with octtree
 * @param readInitial ignored
 * @param octtree parsing result - reduce points with randomized octtree
 * @param ptype parsing result - PointType of the input
 * @param fps parsing result - max. fps
 * @param loadObj parsing result
 * @param loadOct parsing result - load cached octtrees
 * @param saveOct parsing result - save octtree caches
 * @param autoOct parsing result - load, save, invalidate cached octtrees
 * @param origin parsing result - mode for setting the origin
 * @param originset parsing result - if origin was modified
 * @param scale parsing result - measurement scale
 * @param type parsing result - file format to be read
 * @param scanserver parsing result - use scanserver program
 * @param sphereMode parsing result - map input points onto sphere
 * @param customFilter parsing result
 * @param trajectoryFile parsing result
 * @param stepsize parsing result - skip input lines
 * @param identity parsing result
 * @return 0, if the parsing was successful, 1 otherwise
 */
int parseArgs(int argc,char **argv,
              string &dir, int& start, int& end, int& maxDist, int& minDist, 
              double &red, bool &readInitial, unsigned int &octree,
              PointType &ptype, float &fps, string &loadObj,
              bool &loadOct, bool &saveOct, bool &autoOct, int &origin, bool &originset,
              double &scale, IOType &type, bool& scanserver, 
              double& sphereMode, string& customFilter, string& trajectoryFile,
              int &stepsize, bool &identity)
{
  using namespace boost::program_options;

  // Temporary parsing variables
  unsigned int types = PointType::USE_NONE;
  string format;
  bool no_points, no_cameras, no_path, no_fog;

  options_description gui_options("GUI options");
  gui_options.add_options()
    ("nogui,G", bool_switch(&nogui),
      "Turn off GUI")
    ("fps,F", value(&fps)->default_value(20), "Maximum framerate")
    ("dimensions,x", value<string>(),
      "Window dimensions in WxH format.")
    ("advanced,2", bool_switch(&advanced_controls),
      "Switch on advanced controls")
    ;
  options_description display_options("Display options");
  display_options.add_options()
    ("scale,C", value(&scale)->default_value(0.01),
      "Scale factor to use. Influences movement speed etc. "
      "Use 1 when point coordinates are in meters, 0.01 when in centimeters "
      "and so forth.")
    ("fov", value(&cangle)->default_value(60),
      "Horizontal field of view angle in degrees. "
      "The vertical angle depends on window size.")
    ("topview", bool_switch(&showTopView), // FIXME starts out with a view from the front, need to call topView()
      "Initialize the camera above the point cloud with parallel projection.")
    ("no-points", bool_switch(&no_points),
      "Initially, do not draw points.")
    ("no-cameras", bool_switch(&no_cameras),
      "Initially, do not draw cameras.")
    ("no-path", bool_switch(&no_path),
      "Initially, do not draw camera path.")
    ("no-fog", bool_switch(&no_fog),
      "Initially turn off fog.")
    ("fog-type", value(&show_fog)->default_value(1),
      "How fog dims points with distance:\n"
      "0   = no fog\n"
      "1   = exponential\n"
      "2   = exponential squared\n"
      "3   = linear\n"
      "4-6 = inverted options 1-3 (further is brighter)") // FIXME? during testing, these did not do anything
    ("fog-density", value(&fogDensity)->default_value(0.1),
      "Fog density. Useful values are between 0 and 1.")
    ("position", value<string>(),
      "Camera starting position, given as \"%lf,%lf,%lf\" for x, y, z.")
    ("rotation", value<string>(),
      "Camera starting rotation, given as a quaternion \"%lf,%lf,%lf,%lf\".")
    ;

  options_description color_options("Point coloring");
  color_options.add_options()
    ("color,c", bool_switch(),
      "Use included RGB values for coloring points.")
    ("reflectance,R", bool_switch(), // XXX had to drop --reflectivity
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
    ("colormap", value<string>(),
      "With which colors to color the points, according to their color value "
      "in a spectrum. Available color maps are: solid, grey, hsv, jet, hot, "
      "rand, shsv, temp.")
    ("colormin", value(&mincolor_value),
      "Minimum value for mapping the color spectrum.")
    ("colormax", value(&maxcolor_value),
      "Maximum value for mapping the color spectrum.")
    ("noanimcolor,A",
      bool_switch(&coloranim)
        ->implicit_value(false)
        ->default_value(true),
      "Do not switch to different color settings when displaying animation")
    ("time,t", bool_switch()) // TODO description
    ;

  options_description scan_options("Scan selection");
  scan_options.add_options()
    ("scanserver,S", bool_switch(&scanserver),
      "Use the scanserver as an input method and for handling scan data.")
    ("start,s", value(&start), "Start at this scan number (0-based)")
    ("end,e", value(&end), "Stop at this scan number (0-based)")
    ("format,f", value(&format)->default_value("uos"),
      "The input files are read with this shared library.\n"
      "Available values: uos, uos_map, uos_rgb, uos_frames, uos_map_frames, "
      "old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply, "
      "wrl, xyz, zuf, iais, front, x3d, rxp, ais.")
    ;

  options_description reduction_options("Point reduction");
  reduction_options.add_options()
    ("min,M", value(&minDist),
      "Neglect all points closer than this to the origin")
    ("max,m", value(&maxDist),
      "Neglect all points further than this from the origin")
    ("reduce,r", value(&red),
      "Turn on octree based point reduction with a voxel size of n.")
    ("octree,O", value(&octree)->implicit_value(1),
      "Enable randomized octree based point reduction with arg points per voxel. "
      "Requires --reduce (-r).") // TODO where is this enforced?
    ("stepsize", value(&stepsize)->default_value(1),
      "Reduce point cloud by including only every arg-th scanline.")
    ;

  options_description point_options("Point transformation");
  point_options.add_options()
    ("origin,o", value(&origin),
      "Set the starting and reset position according to n. By default, the "
      "starting and reset position are at the origin of the "
      "coordinate system. Also try --position.\n"
      "arg = 0             : the center of mass of all scans\n"
      "arg = [1,2,3,...]   : the center of scan 0,1,2,... (arg-1)\n"
      "arg = [-1,-2,-3,...]: the position of scan 0,1,2,... (-arg-1)\n")
    ("sphere,b", value(&sphereMode),
      "Map all measurements on a sphere of this radius.")
    ("pointsize", value(&pointsize),
      "Size of each point in pixels.")
    ;

  options_description file_options("Octree caching");
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

  options_description other_options("Other options");
  other_options.add_options()
    ("help,?", "Display this help text")
    ("screenshot", bool_switch(&takescreenshot), "Take screenshot and exit")
    ("loadObj,l", value(&loadObj),
      "Load objects specified in this file")
    ("customFilter,u", value(&customFilter),
      "Apply a custom filter. Filter mode and data are specified as a "
      "semicolon-seperated string:\n"
      "\"{filterMode};{nrOfParams}[;param1][;param2][...]\"\n"
      "Multiple filters can be specified in a file (syntax in file is same as "
      "direct specification)\n"
      "\"FILE;{fileName}\"\n"
      "See filter implementation in src/slam6d/pointfilter.cc for more detail.")
    ("no-anim-convert-jpg,J",
      bool_switch(&anim_convert_jpg)
        ->implicit_value(false)
        ->default_value(true)) // TODO description
    ("trajectory-file", value(&trajectoryFile)) // TODO description
    ("identity,i", bool_switch(&identity)) //TODO description
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
    ("input-dir", value(&dir)) // Where the scan files are located
    ;

  positional_options_description pd;
  pd.add("input-dir", 1);

  // Parse the options into this map
  variables_map vm;

  // First parse, but we are only interested in the input directory
  try {
    store(
      command_line_parser(argc, argv)
        .positional(pd)
        .options(cmdline_options)
        .run()
      , vm);
  } catch (const logic_error &e) {
    // logic_error is the superclass for all boost::program_options errors
    cerr << "Error: " << e.what() << endl;
    return 1;
  }
  notify(vm);

  // Parse global config file in $XDG_CONFIG_HOME/3dtk/show.ini
  char *home_c = getenv("HOME");
  char *config_home_c = getenv("XDG_CONFIG_HOME");
  string config_home;
  if (config_home_c && *config_home_c != '\0') {
    config_home = config_home_c;
  } else {
    config_home = string(home_c) + "/.config";
  }

  ifstream user_config_file((config_home + "/3dtk/show.ini").c_str());
  if (user_config_file) {
    try {
      store(parse_config_file(user_config_file, visible_options), vm);
    } catch (const logic_error &e) {
      cerr << "Error: " << e.what() << endl;
      return 1;
    }
    notify(vm);
  }

  // Parse ./config file in the input directory
  ifstream local_config_file((dir + "/config").c_str());
  if (local_config_file) {
    try {
      store(parse_config_file(local_config_file, visible_options), vm);
    } catch (const logic_error &e) {
      cerr << "Error: " << e.what() << endl;
      return 1;
    }
    notify(vm);
  }

  // Command line options now overwrite ./config file
  try {
    store(
      command_line_parser(argc, argv)
        .options(cmdline_options)
        // no .positional() here
        .run()
      , vm);
  } catch (const logic_error &e) {
    cerr << "Error: " << e.what() << endl;
    return 1;
  }
  notify(vm);

  // Help text
  if (argc == 1 || vm.count("help")) {
    cout << "Usage: " << argv[0] << " [options] <input-dir>" << endl;
    cout << visible_options << endl;
    return 1;
  }

  // Error handling
  if (start < 0) {
    cerr << "Error: Cannot start at a negative scan number." << endl;
    return 1;
  }
  if (vm.count("end") && end < 0) {
    cerr << "Error: Cannot end at a negative scan number." << endl;
    return 1;
  }
  if (0 < end && end < start) {
    cerr << "Error: <end> (" << end << ") cannot be smaller than <start> ("
         << start << ")." << endl;
    return 1;
  }

  // Additional logic

  // autoOct implies loadOct and saveOct
  if (autoOct) {
    loadOct = true;
    saveOct = true;
  }

  // Read --format to IOType
  if (vm.count("format")) {
    try {
      type = formatname_to_io_type(format.c_str());
    } catch (const std::runtime_error &e) {
      cerr << "Error while reading --format: " << e.what() << endl;
      return 1;
    }

    // RGB formats imply colored points
    switch (type) {
      case UOS_RGB:
      case UOS_RRGBT:
      case RIEGL_RGB:
      case XYZ_RGB:
      case KS_RGB:
        types |= PointType::USE_COLOR;
        colorScanVal = 2;
        break;
      default:
        break;
    }
  }

  // Set drawing options from flags
  show_points = !no_points;
  show_cameras = !no_cameras;
  show_path = !no_path;
  if (no_fog) show_fog = 0;

  // Translate color bool_switches to a bitset
  std::map<string, unsigned int> point_type_flags({
    {"reflectance", PointType::USE_REFLECTANCE},
    {"temperature", PointType::USE_TEMPERATURE},
    {"amplitude", PointType::USE_AMPLITUDE},
    {"deviation", PointType::USE_DEVIATION},
    {"height", PointType::USE_HEIGHT},
    {"type", PointType::USE_TYPE},
    {"color", PointType::USE_COLOR},
    {"time", PointType::USE_TIME}
  });

  for (auto const &kv_pair : point_type_flags) {
    if (vm.count(kv_pair.first)) {
      types |= kv_pair.second;
    }
  }

  ptype = PointType(types);

  // Parse --colormap
  if (vm.count("colormap")) {
    std::map<string, int> colormap_values({
      {"solid", 0},
      {"grey", 1},
      {"hsv", 2},
      {"jet", 3},
      {"hot", 4},
      {"rand", 5},
      {"shsv", 6},
      {"temp", 7}
    });

    for (auto const &kv_pair : colormap_values) {
      if (vm["colormap"].as<string>() == kv_pair.first) {
        listboxColorMapVal = kv_pair.second;
	// FIXME also need to call changeColorMap after some unknown thing is initialized
	break;
      }
    }
  }

  // Parse window dimensions
  if (vm.count("dimensions")) {
    if (sscanf(vm["dimensions"].as<string>().c_str(), "%dx%d",
               &START_WIDTH, &START_HEIGHT) != 2) {
      cerr << "Error: Window dimensions must be given in format WxH" << endl;
      return 1;
    }
    aspect = (double)START_WIDTH/(double)START_HEIGHT;
    current_width  = START_WIDTH;
    current_height = START_HEIGHT;
  }

  // Parse camera coordinates
  if (vm.count("position")) {
    if (sscanf(vm["position"].as<string>().c_str(), "%lf,%lf,%lf",
               &RVX, &RVY, &RVZ) != 3) {
      cerr << "Error: Camera coordinates must be given as \"X,Y,Z\"" << endl;
      return 1;
    }
    X = RVX;
    Y = RVY;
    Z = RVZ;
  }

  // Parse camera rotation
  if (vm.count("rotation")) {
    if (sscanf(vm["rotation"].as<string>().c_str(), "%lf,%lf,%lf,%lf",
               &Rquat[0], &Rquat[1], &Rquat[2], &Rquat[3]) != 4) {
      cerr << "Error: Camera rotation must be given as \"X,Y,Z,W\"" << endl;
      return 1;
    }
    for (int i = 0; i < 4; i++) {
      quat[i] = Rquat[i];
    }

    // FIXME does not work as intended
    QuaternionToMatrix4(Rquat, view_rotate_button); // TODO move this dirty hack into show_menu.cc
  }

  if (vm.count("origin")) {
    originset = true;
  }

  // Append trailing slash to input directory
#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif

  return 0;
}

