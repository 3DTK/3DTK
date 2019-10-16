#include <map>

#include <boost/regex.hpp>

#include "show/program_options.h"
#include "parsers/range_set_parser.h"

using namespace boost::program_options;

void setDatasetColorOptions(bool& color, ShowColormap& colormap,
		     float& colormin, float& colormax,
		     int& scansColored,
		     options_description& color_options)
{
  color_options.add_options()
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
    ;
}

void setOtherDatasetOptions(std::string& objFileName,
  std::string& customFilter,
  std::string& trajectoryFileName, bool& identity, bool& no_config,
  options_description& other_options)
{
  other_options.add_options()
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
    ("no-config", bool_switch(&no_config), "Disable config file parsing")
    ("trajectory-file", value(&trajectoryFileName)) // TODO description
    ("identity,i", bool_switch(&identity)) //TODO description
    ;
}

GenericProgramOptions::GenericProgramOptions()
 : other_options("Other options"), m_no_config(false)
{
  other_options.add_options()
    ("help,?", "Display this help text");
}

options_description GenericProgramOptions::visibleOptions()
{
  options_description visible_options;
  visible_options.add(other_options);
  return visible_options;
}

parsed_options GenericProgramOptions::parse(int argc, char **argv, variables_map &vm)
{
  options_description visible_options = visibleOptions();
  parsed_options parsed = command_line_parser(argc, argv)
    .options(visible_options)
    .allow_unregistered()
    .run();
  store(parsed, vm);
  notify(vm);

  // Help text
  if (vm.count("help"))
    help(argv[0]);
  return parsed;
}

boost::program_options::parsed_options GenericProgramOptions::parseConfig(std::string filename, const options_description &desc, variables_map &vm)
{
  parsed_options parsed = parsed_options(nullptr);
  std::ifstream user_config_file(filename.c_str());
  if (!m_no_config && user_config_file) {
    std::cout << "Parsing configuration file " << filename << "..." << std::endl;
    parsed = parse_config_file(user_config_file, desc, true);
    store(parsed, vm);
    notify(vm);
  }

  return parsed;
}

ScanProgramOptions::ScanProgramOptions(dataset_settings& dss, bool* directory_present)
  : GenericProgramOptions(), reduction_options("Point reduction"), color_options("Point coloring"), transform_options("Point transformation"),
  octtree_caching_options("OctTree caching"), m_dss(&dss), m_directory_present(directory_present)
{
  setScanOptions(dss.use_scanserver, dss.scan_numbers.min,
    dss.scan_numbers.max, dss.format, datasource_options);

  setReductionOptions(dss.distance_filter.min, dss.distance_filter.max,
    dss.octree_reduction_voxel,
    dss.octree_reduction_randomized_bucket,
    dss.skip_files, reduction_options);

  setDatasetColorOptions(dss.coloring.explicit_coloring,
    dss.coloring.colormap, dss.coloring.colormap_values.min,
    dss.coloring.colormap_values.max,
    dss.coloring.scans_colored,
    color_options);

  setPointOptions(dss.origin_type, dss.sphere_radius, transform_options);

  setFileOptions(dss.save_octree, dss.load_octree, dss.cache_octree,
		 octtree_caching_options);

  setOtherDatasetOptions(dss.objects_file_name,
    dss.custom_filter,
    dss.trajectory_file_name, dss.identity, m_no_config,
		  other_options);
}

boost::program_options::options_description ScanProgramOptions::visibleOptions()
{
  options_description visible_options;
  visible_options
    .add(color_options)
    .add(datasource_options)
    .add(reduction_options)
    .add(transform_options)
    .add(octtree_caching_options)
    .add(other_options)
    ;
  return visible_options;
}

parsed_options ScanProgramOptions::parse(int argc, char **argv, variables_map &vm)
{
  m_vm = &vm;

  std::cout << "CHECK: Scan parse" << std::endl;

  parsed_options parsed = GenericProgramOptions::parse(argc, argv, vm);
  std::vector<std::string> unrecognized = collect_unrecognized(parsed.options, include_positional);

  // TODO make all defaults declared here the initial values for the settings structs, then use that initial value as the default here
  options_description visopts = visibleOptions();
  options_description cmdline_options("");
  cmdline_options.add(visopts);
  cmdline_options.add_options()
    ("input-dir,data-sources", value<std::vector<std::string> >(&m_data_sources), "Scan directory or data-sources definition")
    ;

  positional_options_description pd;
  pd.add("input-dir", -1);

  // First parse, but we are only interested in the input directory
  parsed = command_line_parser(unrecognized)
    .positional(pd)
    .options(cmdline_options)
    .allow_unregistered()
    .run();

  store(parsed, vm);
  notify(vm);

  return parsed;
}

void ScanProgramOptions::process()
{
  if (m_directory_present == nullptr && !m_vm->count("help") && m_vm->count("input-dir") == 0) {
    std::cerr << "Error: Please specify a directory. See --help for options." << std::endl;
    exit(1);
  }

  // Scan number range
  if (m_dss->scan_numbers.min < 0) {
    throw std::logic_error("Cannot start at a negative scan number.");
  }
  if (m_dss->scan_numbers.max < -1) {
    throw std::logic_error("Cannot end at a negative scan number.");
  }
  if (0 < m_dss->scan_numbers.max && m_dss->scan_numbers.max < m_dss->scan_numbers.min) {
    throw std::logic_error("<end> (" + to_string(m_dss->scan_numbers.max) + ") cannot be smaller than <start> (" + to_string(m_dss->scan_numbers.min) + ").");
  }

  // cache_octree implies load_octree and save_octree
  if (m_dss->cache_octree) {
    m_dss->load_octree = true;
    m_dss->save_octree = true;
  }

  // Bitset for initializing PointTypes
  unsigned int types = PointType::USE_NONE;

  // RGB formats imply colored points
  switch (m_dss->format) {
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
  std::vector<std::pair<std::string, unsigned int>> point_type_flags {
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
  std::array < std::string, 6 > colorval_names{ "height", "reflectance", "temperature", "amplitude", "deviation", "type" };

  for (auto const &kv_pair : point_type_flags) {
    if ((*m_vm)[kv_pair.first].as<bool>()) {
      // Translate color bool_switches to a bitset
      types |= kv_pair.second;

      // remember the most important one as default for listboxColorVal
      auto colorval_it = std::find(colorval_names.begin(), colorval_names.end(), kv_pair.first);
      if (colorval_it != colorval_names.end()) {
        m_dss->coloring.colorval = colorval_it - colorval_names.begin();
      }
    }
  }

  m_dss->coloring.ptype = PointType(types);

  if (m_vm->count("origin")) {
    m_dss->origin_type_set = true;
  }

  const char separator =
#ifdef _MSC_VER
  '\\';
#else
  '/';
#endif

  if (m_vm->count("input-dir") == 1) {
    if (m_data_sources.size() > 1) {
      //m_dss.set(data_sources);
      m_dss->data_type = dataset_settings::MULTI_SRCS;
      for (auto dss_def : m_data_sources) {
        dataset_settings *child_set = new dataset_settings(m_dss);
        parse_dataset(dss_def, *child_set);
        child_set->scan_ranges.setLimits(m_dss->scan_numbers.min, m_dss->scan_numbers.max);
        child_set->data_type = dataset_settings::SINGLE_SRC;
        if (child_set->data_source.back() != separator) child_set->data_source += separator;
      }
    }
    else {
      //parse again to overwrite global parameters
      parse_dataset(m_data_sources[0], *m_dss);
      m_dss->scan_ranges.setLimits(m_dss->scan_numbers.min, m_dss->scan_numbers.max);
      if (m_dss->data_source.back() != separator) m_dss->data_source += separator;
    }
  }

  if (m_directory_present != nullptr) {
    *m_directory_present = m_vm->count("input-dir") != 0;
  }
}

ShowProgramOptions::ShowProgramOptions(dataset_settings & dss, window_settings & ws, display_settings & ds, bool * directory_present)
  : ScanProgramOptions(dss, directory_present), gui_options("GUI options"), display_options("Display options"), m_ds(&ds), m_ws(&ws)
{
  setGUIOptions(ws.nogui, ws.max_fps, ws.dimensions, ws.advanced_controls,
    ws.invert_mouse_x, ws.invert_mouse_y,
    ws.capture_mouse, ws.hide_widgets, gui_options);

  setDisplayOptions(dss.scale, ds.camera.fov, ds.init_with_viewmode,
    m_no_points, m_no_cameras, m_no_path, m_no_poses,
    m_no_fog, ds.fog.type, ds.fog.density,
    ds.camera.position, ds.camera.rotation, ds.pointsize,
    display_options);

  color_options.add_options()
    ("bgcolor", value(&dss.coloring.bgcolor)->default_value(Color(0, 0, 0), "0,0,0"),
      "Drawing area background color, given as \"%f,%f,%f\" for red, green and blue.")
    ("noanimcolor,A", bool_switch(&m_no_anim_color),
      "Do not switch to different color settings when displaying animation");

  other_options.add_options()
    ("screenshot", bool_switch(&ws.take_screenshot), "Take screenshot and exit")
    ("screenshot-filename", value(&ws.screenshot_filename), "Output filename for --screenshot")
    ("no-anim-convert-jpg,J", bool_switch(&m_no_anim_convert_jpg)); // TODO description
}

options_description ShowProgramOptions::visibleOptions()
{
  options_description visible_options("");
  visible_options
    .add(gui_options)
    .add(display_options)
    .add(datasource_options)
    .add(color_options)
    .add(reduction_options)
    .add(transform_options)
    .add(octtree_caching_options)
    .add(other_options);
  return visible_options;
}

parsed_options ShowProgramOptions::parse(int argc, char **argv, variables_map &vm)
{
  parsed_options parsed = ScanProgramOptions::parse(argc, argv, vm);
  std::vector<std::string> unrecognized = collect_unrecognized(parsed.options, exclude_positional);

  options_description visopts = visibleOptions();
  options_description cmdline_options("");
  cmdline_options.add(visopts);
  cmdline_options.add_options()
    ("hide-label", bool_switch(&m_ds->hide_label))
    ;

  store(
    command_line_parser(unrecognized)
      .options(cmdline_options)
      .run()
    , vm);
  notify(vm);

  return parsed;
}

void ShowProgramOptions::process()
{
  // Parse user config file
  std::string config_home = getConfigHome() + "/3dtk/show.ini";

  options_description cmdline_options("");
  cmdline_options.add(visibleOptions());
  cmdline_options.add_options()
    ("hide-label", bool_switch(&m_ds->hide_label))
    ;

  parseConfig(config_home, cmdline_options, *m_vm);
  if (!m_no_config && m_dss->data_type == dataset_settings::SINGLE_SRC) {
    //dss.set(data_sources[0]);
    std::string config_ini = m_dss->data_source + "config.ini";
    parseConfig(config_ini, cmdline_options, *m_vm);
  }

  // Set drawing options from flags
  m_ds->draw_points      = !m_no_points;
  m_ds->draw_cameras     = !m_no_cameras;
  m_ds->draw_path        = !m_no_path;
  m_ds->draw_poses       = !m_no_poses;
  m_ds->color_animation  = !m_no_anim_color;
  m_ds->anim_convert_jpg = !m_no_anim_convert_jpg;
  if (m_no_fog) m_ds->fog.type = 0;

  ScanProgramOptions::process();
}


void parse_show_args(int argc, char **argv, dataset_settings& dss, window_settings& ws, display_settings& ds, bool *directory_present) {
  boost::program_options::variables_map vm;
  ShowProgramOptions show_parser(dss, ws, ds);
  show_parser.parse(argc, argv, vm);
  show_parser.process();
}

std::string getConfigHome()
{
  std::string config_home;
#ifdef _MSC_VER
  // in %APPDATA%/3dtk/show.ini
  char *appdata = getenv("APPDATA");
  if (appdata != NULL) {
    config_home = std::string(appdata);
  }
#else
  // in $XDG_CONFIG_HOME/3dtk/show.ini
  char *home_c = getenv("HOME");
  char *config_home_c = getenv("XDG_CONFIG_HOME");
  if (config_home_c != NULL && *config_home_c != '\0') {
    config_home = config_home_c;
  } else if (home_c != NULL) {
    config_home = std::string(home_c) + "/.config";
  }
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
    ("scale,y", value(&scale)->default_value(0.01, "0.01"),
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
     "Drawing area background color, given as \"%f,%f,%f\" for red, green and blue.");
  setDatasetColorOptions(color, colormap, colormin, colormax, scansColored, color_options);
  color_options.add_options()
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

void setReductionOptions(double& distMin, double& distMax, double& reduce,
			 int& octree, int& stepsize,
			 options_description& reduction_options)
{
  reduction_options.add_options()
    ("min,M", value<double>(&distMin)->default_value(0.0),
     "Neglect all points closer than this to the origin")
    ("max,m", value<double>(&distMax)->default_value(-1.0),
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

void setOtherOptions(bool& screenshot, std::string& screenshot_filename, std::string& objFileName,
		     std::string& customFilter,	bool& noAnimConvertJPG,
		     std::string& trajectoryFileName, bool& identity, bool& no_config,
		     options_description& other_options)
{
  setOtherDatasetOptions(objFileName,
    customFilter, trajectoryFileName, identity, no_config,
    other_options);

  other_options.add_options()
    ("screenshot", bool_switch(&screenshot), "Take screenshot and exit")
    ("screenshot-filename", value(&screenshot_filename), "Output filename for --screenshot")
    ("no-anim-convert-jpg,J", bool_switch(&noAnimConvertJPG)) // TODO description
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
