/*
 * show_common implementation
 *
 * Copyright (C) Kai Lingemann, Andreas Nuechter, Jan Elseberg, Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */


#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#include <windows.h>
#endif

#ifdef WITH_GLEE
#include "glee/GLee.h"
#endif

#include "show/show_Boctree.h"
#include "show/compacttree.h"
#include "show/NurbsPath.h"
#include "show/vertexarray.h"
#ifndef DYNAMIC_OBJECT_REMOVAL
#include "slam6d/scan.h"
#include "slam6d/managedScan.h"
#else
#include "veloslam/veloscan.h"
#endif
#include "show/show.h"
#include "GL/glui.h"  /* Header File For The glui functions */
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <map>

#include <boost/program_options.hpp>

#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif


#ifdef _MSC_VER
#if !defined _OPENMP && defined OPENMP 
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

#include "slam6d/point_type.h"
#include "slam6d/io_utils.h"
#include "show/display.h"

using std::ifstream;
using std::exception;

/**
 * This vector contains the pointer to a vertex array for
 * all colors (inner vector) and all scans (outer vector)
 */
vector< vector<vertexArray*> > vvertexArrayList;

vector< ::SDisplay*> displays;
/**
 * the octrees that store the points for each scan
 */
//Show_BOctTree **octpts;
vector<colordisplay*> octpts;
/**
 * Storing the base directory
 */
string scan_dir;

/**
 * Storing the ID of the main windows
 */
int window_id;

/**
 * Size of points
 */
GLfloat pointsize          = 1.0;
int     anim_delay         = 5;

/**
 * Select Color Buffer
 */
GLenum buffermode = GL_BACK;

/**
 * Indicator whether and how the drawing window
 * has to be updated.
 *
 * haveToUpdate == 1 redisplay
 * haveToUpdate == 2 reshape
 * haveToUpdate == 3 animation scan matching
 * haveToUpdate == 4 stop animation scan matching
 * haveToUpdate == 6 path animation
 * haveToUpdate == 7 force redisplay with all points
 */
int haveToUpdate         = 0;

/**
 * Flag for invert the scene
 */
bool invert              = true;

/**
 * Flag for indicating brid eyes view
 */
bool showTopView         = false;

/**
 * Flag for idicating camera add mode
 */
bool addCameraView       = false;         //Is the view in add box mode?

/**
 * Storing the apex angle of the camera
 */
GLfloat cangle          = 60.0;          // Current camera opening mode
GLfloat cangle_old      = cangle;

/**
 * Current rotation axis of the scene as quaternion
 */
GLdouble quat[4]         = {0.0, 0.0, 0.0, 1.0};
GLdouble Rquat[4]        = {0.0, 0.0, 0.0, 1.0};

/**
 * Current translation of the scene
 */
GLdouble X   = 0.0, Y   = 0.0, Z   = 0.0;
GLdouble RVX = 0.0, RVY = 0.0, RVZ = 0.0;

/**
 * Center of Mass coordinates
 */
GLdouble CoM[3] = { 0.0, 0.0, 0.0 };

/**
 * parallel zoom (similar to apex angle) for parallel projection
 */
GLfloat pzoom = 2000.0;
GLfloat pzoom_old = pzoom;


/**
 * Mode of the fog (exp, exp2, linear)
 */
GLint fogMode            = GL_EXP;

/**
 * Indicates if fog should be shown
 */
int show_fog             = 1;

/**
 * Indicates if the points should be shown
 */
int show_points          = 1;             // Show data points in the viewer?

/**
 * Indicates if camera boxes should be shown
 */
int show_cameras         = 1;             // Show the camera boxes in the viewer?

/**
 * Indicates if camera path or robot path should be shown
 */
int show_path            = 1;             // Show the camera movement path ?

/**
 * Camera navigation by mouse or by panel
 */
int cameraNavMouseMode  = 1;

int mouseNavX, mouseNavY;
int mouseNavButton = -1;

double mouseRotX = 0.0;
double mouseRotY = 0.0;
double mouseRotZ = 0.0;

bool keymap[256];

/**
 * draw scans in different color during animation
 */
bool coloranim = true;

/**
 * hide the gui
 */
bool nogui = false;

//@@@
//int animate_both         = 0;             // Animate both scan matchin and path?

int frameNr = 0;

int path3D = 0;
float shifted = 20.0;

// custom scan range selection from GUI
int startScanIdx;
int endScanIdx;
int startRangeScanIdx;
int endRangeScanIdx;
bool readIni;
IOType scanIOtype;

/**
 * Storing of all transformation (frames for animation) of all scans
 */
vector < vector <double*> > MetaMatrix;

/**
 * Storing of AlgoType for all frames
 */
vector < vector <Scan::AlgoType> > MetaAlgoType;

/**
 * Trajectory loaded from file for visualization
 */
vector<double*> trajectory;

/**
 * Window position
 */
int START_X              = 0;
int START_Y              = 0;
int START_WIDTH          = 960;
int START_HEIGHT         = 540;
// Current aspect ratio
GLdouble aspect          = (double)START_WIDTH/(double)START_HEIGHT;  
bool advanced_controls = false;
bool anim_convert_jpg  = true;

bool fullscreen    = false;
int current_width  = START_WIDTH;
int current_height = START_HEIGHT;


double scale = 0.01; // in m
// the following values are scale dependant, i.e. all values are in m
float neardistance     = 0.10;
double oldneardistance = 0.10;
float maxfardistance   = 400.0;
double fardistance     = 400.0;
double oldfardistance  = 40000.0;
double movementSpeed   = 0.1;
double defaultZoom     = 20.0;
GLfloat fogDensity     = 0.1;
double voxelSize       = 0.20;

float adaption_rate = 1.0;
float LevelOfDetail = 0.0001;

// Defines for Point Semantic
#define TYPE_UNKNOWN         0x0000
#define TYPE_OBJECT          0x0001
#define TYPE_GROUND          0x0002
#define TYPE_CEILING         0x0003

unsigned int cam_choice             = 0;

static unsigned int path_iterator = 0;
static int oldcamNavMode = 0;

/**
 * Animation sould be saved to file
 */
int save_animation         = 0;

/**
 * If true, interpolation for camera path is based on distance, else always
 * the same number of images between two cameras.
 */
int inter_by_dist          = 1;

/** some variables for the camera path **/
vector<PointXY> path_vectorX, path_vectorZ;
vector<PointXY> lookat_vectorX, lookat_vectorZ;
vector<PointXY> ups_vectorX, ups_vectorZ;
vector<Point> cams;
vector<Point> lookats;
vector<Point> ups;

NurbsPath cam_nurbs_path;
char *path_file_name;
char *pose_file_name;
float path_interp_factor = 0.1;

/** Factor for saved image size */
int factor = 1;

/**
 * program tries to have this framerate
 */
float idealfps = 20.0;
/**
 * value of the listBox fo Color Value and Colormap
 */
int listboxColorVal = 0;
int listboxColorMapVal = 0;
int colorScanVal = 0;
ScanColorManager *cm;
float mincolor_value = 0.0;
float maxcolor_value = 0.0;
//unsigned int  types = Point::USE_HEIGHT;
PointType pointtype;

/**
 * Contains the selected points for each scan
 */
set<sfloat *> *selected_points;
/**
 * Select single points?
 */
int select_voxels         = 0;
/**
 * Select or unselect points ?
 */
int selectOrunselect      = 1;
/** octree depth for selecting groups of points */
int selection_depth = 1;
int brush_size = 0;
char *selection_file_name;

// show_gl needs to know this function for correct handling of close event
void deinitShow();

int current_frame = 0;
#include "show_menu.cc"
#include "show_animate.cc"
#include "show_gl.cc"

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

void setResetView(int origin) {
    if (origin == 0) {
        // set origin to the center of mass of all scans
        for (size_t i = 0; i < octpts.size(); ++i) {
            vector <sfloat*> points;
#ifndef USE_COMPACT_TREE
            BOctTree<sfloat>* cur_tree = ((Show_BOctTree<sfloat>*)octpts[i])->getTree();
            cur_tree->AllPoints( points );
#endif

            double centroid[3] = {0., 0., 0.};
            double centroid_transformed[3];;
            for (size_t j = 0; j < points.size(); ++j) {
                for (unsigned int k = 0; k < 3; ++k)
                    centroid[k] += points[j][k];
            }
            for (unsigned int k = 0; k < 3; ++k) {
                centroid[k] /= (double)points.size();
            }
            transform3(MetaMatrix[i].back(), centroid, centroid_transformed);
            for (unsigned int k = 0; k < 3; ++k) {
                CoM[k] += centroid_transformed[k];
            }
        }
        for (unsigned int k = 0; k < 3; ++k)
            CoM[k] /= octpts.size() * 1.;

        RVX = -CoM[0];
        RVY = -CoM[1];
        RVZ = -CoM[2];
        X = RVX;
        Y = RVY;
        Z = RVZ;
    } else if (origin > 0) {
        // set origin to the center of the octree of scan origin-1
        double center[3], center_transformed[3];
#ifdef USE_COMPACT_TREE
        ((compactTree*)octpts[origin-1])->getCenter(center);
#else
        ((Show_BOctTree<sfloat>*)octpts[origin-1])->getCenter(center);
#endif
        transform3(MetaMatrix[origin-1].back(), center, center_transformed);
        RVX = -center_transformed[0];
        RVY = -center_transformed[1];
        RVZ = -center_transformed[2];
        X = RVX;
        Y = RVY;
        Z = RVZ;
    } else {
        // set origin to the pose of scan 1-origin
        double *transmat = MetaMatrix[1-origin].back();

        RVX = -transmat[12];
        RVY = -transmat[13];
        RVZ = -transmat[14];
        Matrix4ToQuat(transmat, Rquat);
        X = RVX;
        Y = RVY;
        Z = RVZ;
        quat[0] = Rquat[0];
        quat[1] = Rquat[1];
        quat[2] = Rquat[2];
        quat[3] = Rquat[3];
    }
}

/*
 * A function that read the .frame files created by slam6D
 *
 * @param dir the directory
 * @param start starting at scan number 'start'
 * @param end stopping at scan number 'end'
 * @param read a file containing a initial transformation matrix and apply it
 */
int readFrames(string dir, int start, int end, bool readInitial, IOType &type)
{

  // convert to OpenGL coordinate system
  double mirror[16];
  M4identity(mirror);
  mirror[10] = -1.0;
  
  double initialTransform[16];
  if (readInitial) {
    cout << "Initial Transform:" << endl;
    string initialTransformFileName = dir + "initital.frame";
    ifstream initial_in(initialTransformFileName.c_str());
    if (!initial_in.good()) {
      cout << "Error opening " << initialTransformFileName << endl;
      exit(-1);
    }
    initial_in >> initialTransform;
    cout << initialTransform << endl;
    
    // update the mirror to apply the initial frame for all frames
    double tempxf[16];
    MMult(mirror, initialTransform, tempxf);
    memcpy(mirror, tempxf, sizeof(tempxf));
  }

  int ScanNr=0;
  for(std::vector<Scan*>::iterator it = Scan::allScans.begin();
      it != Scan::allScans.end();
      ++it) {
    ScanNr++;
    const double* transformation;
    Scan::AlgoType algoType;
    vector<double*> Matrices;
    vector<Scan::AlgoType> algoTypes;
    
    // iterate over frames (stop if none were created) and
    // pull/convert the frames into local containers
    unsigned int frame_count;
    try {
      frame_count = (*it)->readFrames();
    } catch(std::ios_base::failure& e) {
      break;
    }

	if (frame_count == 0) {
		break;
	}

    for(unsigned int i = 0; i < frame_count; ++i) {
      (*it)->getFrame(i, transformation, algoType);
      double* transMatOpenGL = new double[16];
      
      // apply mirror to convert (and initial frame if requested)
      // the frame and save in opengl
      MMult(mirror, transformation, transMatOpenGL);

      Matrices.push_back(transMatOpenGL);
      algoTypes.push_back(algoType);
    }
    
    MetaAlgoType.push_back(algoTypes);
    MetaMatrix.push_back(Matrices);
    
    if((type == UOS_MAP || type == UOS_MAP_FRAMES || type == RTS_MAP)
       && it == Scan::allScans.begin()) {
      MetaAlgoType.push_back(algoTypes);
      MetaMatrix.push_back(Matrices);
    }
    current_frame = MetaMatrix.back().size() - 1;
  }
  
  if (MetaMatrix.size() == 0) {
    cerr << "*****************************************" << endl;
    cerr << "** ERROR: No .frames could be found!   **" << endl;
    cerr << "*****************************************" << endl;
    cerr << " ERROR: Missing or empty directory: " << dir << endl << endl;
    return -1;
  }
  return 0;
}

void generateFrames(int start, int end, bool identity) {
  if (identity) {
    cout << "using Identity for frames " << endl;
  } else {
    cout << "using pose information for frames " << endl;
  }
  int  fileCounter = start;
  int index = 0;
  for (;;) {
    if (fileCounter > end) break; // 'nuf read
    fileCounter++;

    vector <double*> Matrices;
    vector <Scan::AlgoType> algoTypes;

    double mirror[16];
    M4identity(mirror);
    mirror[10] = -1.0;
    for (int i = 0; i < 3; i++) {
      double * transMat = new double[16];

      if (identity) {
        for(int j = 0; j < 16; j++) transMat[j] = mirror[j];
      } else {
        double tmpxf[16];
        EulerToMatrix4(Scan::allScans[index]->get_rPos(),
                       Scan::allScans[index]->get_rPosTheta(),
                       tmpxf );
        MMult(mirror, tmpxf, transMat);
      }

      Matrices.push_back(transMat);
      algoTypes.push_back(Scan::ICP);

    }
    index++;
    MetaAlgoType.push_back(algoTypes);
    MetaMatrix.push_back(Matrices);
    current_frame = MetaMatrix.back().size() - 1;
  }
}

void cycleLOD() {
  LevelOfDetail = 0.00001;
  for (unsigned int i = 0; i < octpts.size(); i++)
    octpts[i]->cycleLOD();
}


void reloadFrames() {
  // reload all frame files for live changes
  // drop previously stored information

  cout << "Reloading frame files..." << endl;

  MetaMatrix.clear();
  MetaAlgoType.clear();

  if (readFrames(scan_dir, startScanIdx, endScanIdx, readIni, scanIOtype))
    generateFrames(startScanIdx, endScanIdx, false /*use .pose*/);
}

void initShow(int argc, char **argv){

  /***************/
  /* init OpenGL */
  /***************/
  glutInit(&argc,argv);

  cout << "(wx)show - A highly efficient 3D point cloud viewer" << endl
       << "(c) University of Wuerzburg, Germany, since 2013" << endl
       << "    Jacobs University Bremen gGmbH, Germany, 2009 - 2013" << endl
       << "    University of Osnabrueck, Germany, 2006 - 2009" << endl << endl;

  double red   = -1.0;
  int start = 0, end = -1, maxDist = -1, minDist = -1;
  string dir;
  bool readInitial = false;
  IOType type  = UOS;
  unsigned int octree = 0;
  bool loadOct = false;
  bool saveOct = false;
  bool autoOct = false;
  string loadObj;
  int origin = 0;
  bool originset = false;
  double scale = 0.01; // in m
  bool scanserver = false;
  double sphereMode = 0.0;
  bool customFilterActive = false;
  string customFilter;
  string trajectoryFile;
  int stepsize = 1;
  bool identity = false;

  pose_file_name = new char[1024];
  path_file_name = new char[1024];
  selection_file_name = new char[1024];

  strncpy(pose_file_name, "pose.dat", 1024);
  strncpy(path_file_name, "path.dat", 1024);
  strncpy(selection_file_name, "selected.3d", 1024);

  int parseError =
  parseArgs(argc, argv, dir, start, end, maxDist, minDist, red, readInitial,
            octree, pointtype, idealfps, loadObj, loadOct, saveOct, autoOct,
            origin, originset, scale, type, scanserver, sphereMode,
            customFilter, trajectoryFile, stepsize, identity);

  if (parseError != 0) {
    exit(parseError);
  }

  // modify all scale dependant variables
  scale = 1.0 / scale;
  movementSpeed *= scale;
  neardistance *= scale;
  oldneardistance *= scale;
  maxfardistance *= scale;
  fardistance *= scale;
  fogDensity /= scale;
  defaultZoom *= scale;
  voxelSize *= scale;

  if (type == B3D ) {
	  voxelSize = 0.2;
  }

  ////////////////////////
  SDisplay::readDisplays(loadObj, displays);
  ////////////////////

  if (type == OCT) {
    loadOct = true;
  }

  // if we want to load display file get pointtypes from the files first
  if (loadOct) {
    string scanFileName = dir + "scan" + to_string(start,3) + ".oct";
    cout << "Getting point information from " << scanFileName << endl;
    cout << "Attention! All subsequent oct-files must be of the same type!"
         << endl;
  }
  scan_dir = dir;

  // init and create display
  M4identity(view_rotate_button);
  obj_pos_button[0] = obj_pos_button[1] = obj_pos_button[2] = 0.0;
  
  // Loading scans, reducing, loading frames and generation if neccessary
  
  // load all available scans
  Scan::openDirectory(scanserver, dir, type, start, end);
  
  if (Scan::allScans.size() == 0) {
    cerr << "No scans found. Did you use the correct format?" << endl;
    exit(-1);
  }

  // custom filter set? quick check, needs to contain at least one ';' 
  // (proper checking will be done case specific in pointfilter.cc)
  size_t pos = customFilter.find_first_of(";");
  if (pos != std::string::npos){
    customFilterActive = true;

    // check if customFilter is specified in file
    if (customFilter.find("FILE;") == 0){
      string selection_file_name = customFilter.substr(5, customFilter.length());
      ifstream selectionfile;
      // open the input file
      selectionfile.open(selection_file_name, ios::in);

      if (!selectionfile.good()){
        cerr << "Error loading custom filter file " << selection_file_name << "!" << endl;
        cerr << "Data will NOT be filtered.!" << endl;
        customFilterActive = false;
      }
      else {
        string line;
        string custFilt;
        while (std::getline(selectionfile, line)){
          // allow comment or empty lines
          if (line.find("#") == 0) continue;
          if (line.length() < 1) continue;
          custFilt = custFilt.append(line);
          custFilt = custFilt.append("/");
        }
        if (custFilt.length() > 0) {
          // last '/'
          customFilter = custFilt.substr(0, custFilt.length() - 1);
        }
      }
      selectionfile.close();
    }
  }
  else {
    // give a warning if custom filter has been inproperly specified
    if (customFilter.length() > 0){
      cerr << "Custom filter: specifying string has not been set properly, data will NOT be filtered." << endl;
    }
  }
  
  int scanNr = 0;
  vector<Scan*> valid_scans;
  for (ScanVector::iterator it = Scan::allScans.begin();
       it != Scan::allScans.end();
       ++it) {
    Scan* scan = *it;
    scan->setRangeFilter(maxDist, minDist);
    if (customFilterActive) scan->setCustomFilter(customFilter);
    if (sphereMode > 0.0) scan->setRangeMutation(sphereMode);
    if (red > 0) {
      // scanserver differentiates between reduced for slam and
      // reduced for show, can handle both at the same time
      if(scanserver) {
        dynamic_cast<ManagedScan*>(scan)->setShowReductionParameter(red, octree);
      } else {
        scan->setReductionParameter(red, octree);
      }
    }
    scanNr++;
    if ((scanNr-1)%stepsize != 0) delete scan; 
    else valid_scans.push_back(scan);
  }
  //Remove scans if some got invalid due to filtering
  if(Scan::allScans.size() > valid_scans.size()) Scan::allScans = valid_scans;

  if (sphereMode > 0.0) {
    cm = new ScanColorManager(4096, pointtype, /* animation_color = */ false);
  } else {
    cm = new ScanColorManager(4096, pointtype, /* animation_color = */ true);
  }
  
#ifdef USE_COMPACT_TREE
  cout << "Creating compact display octrees.." << endl;
#else
  cout << "Creating display octrees.." << endl;
#endif

  if (loadOct)
    cout << "Loading octtrees from file where possible instead of creating them from scans."
         << endl;
  
  // for managed scans the input phase needs to know how much it can handle
  std::size_t free_mem = 0;
  if(scanserver)
    free_mem = ManagedScan::getMemorySize();
  
  for(unsigned int i = 0; i < Scan::allScans.size(); ++i) {
    Scan* scan = Scan::allScans[i];
  
  // create data structures
#ifdef USE_COMPACT_TREE // FIXME: change compact tree, then this case can be removed
    compactTree* tree;
    try {
      if (loadOct) {
        string sfName = dir + "scan" + to_string(i,3) + ".oct";
        cout << "Load " << sfName;
        tree = new compactTree(sfName, cm);
        cout << " done." << endl;
      } else {
        if (red > 0) { // with reduction, only xyz points
          DataXYZ xyz_r(scan->get("xyz reduced show"));
          tree = new compactTree(PointerArray<double>(xyz_r).get(),
                                 xyz_r.size(),
                                 voxelSize,
                                 pointtype,
                                 cm);
        } else { // without reduction, xyz + attribute points
          sfloat** pts = pointtype.createPointArray<sfloat>(scan);
          unsigned int nrpts = scan->size<DataXYZ>("xyz");
          tree = new compactTree(pts, nrpts, voxelSize, pointtype, cm);
          for(unsigned int i = 0; i < nrpts; ++i) delete[] pts[i];
          delete[] pts;
          if (saveOct) {
            string sfName = dir + "scan" + to_string(i,3) + ".oct";
            tree->serialize(sfName);
          }
        }
      }
    } catch(...) {
      cout << "Scan " << i
           << " could not be loaded into memory, stopping here."
           << endl;
      break;
    }
#else // FIXME: remove the case above
    scan->setOcttreeParameter(red, voxelSize, pointtype, loadOct, saveOct, autoOct);
    
    DataOcttree* data_oct;
    try {
      data_oct = new DataOcttree(scan->get("octtree"));
    } catch(runtime_error& e) {
      cout << "Scan " << i
           << " could not be loaded into memory, stopping here. Reason: "
           << e.what()
           << endl;
      break;
    }
    BOctTree<float>* btree = &(data_oct->get());
    unsigned int tree_size = btree->getMemorySize();
    
    if(scanserver) {
      // check if the octtree would actually fit with all the others
      if(tree_size > free_mem) {
        delete data_oct;
        cout << "Stopping at scan " << i
             << ", no more octtrees could fit in memory." << endl;
        break;
      } else {
        // subtract available memory
        free_mem -= tree_size;
      }
    }
#endif //FIXME: COMPACT_TREE
    
#if !defined USE_COMPACT_TREE
    // show structures
    // associate show octtree with the scan and
    // hand over octtree pointer ownership

    Show_BOctTree<sfloat>* tree = new Show_BOctTree<sfloat>(scan, data_oct, cm);
    
    // unlock cached octtree to enable creation
    // of more octtres without blocking the space for full scan points
    tree->unlockCachedTree();
#endif

    // octtrees have been created successfully
    octpts.push_back(tree);
    
    // print something
#ifdef USE_COMPACT_TREE
    // TODO: change compact tree for memory footprint output, remove this case
    cout << "Scan " << i << " octree finished." << endl;
#else
    cout << "Scan " << i << " octree finished (";
    bool space = false;
    if (tree_size/1024/1024 > 0) {
      cout << tree_size/1024/1024 << "M";
      space = true;
    }      
    if ((tree_size/1024)%1024 > 0) {
      if (space) cout << " ";
      cout << (tree_size/1024)%1024 << "K";
      space = true;
    }
    if (tree_size%1024 > 0) {
      if (space) cout << " ";
      cout << tree_size%1024 << "B";
    }
    cout << ")." << endl;
#endif
  }

/*
  TODO: to maximize space for octtrees, implement a heuristic to remove all
  CacheObjects sequentially from the start and pack octtrees one after another
  until it reaches the maximum size allowed, resetting the index,
  but causing the
  first to be locked again and stopping by catching the exception
  set heuristic, do locking, catch exception, reset heuristic to default or old
*/
#if !defined USE_COMPACT_TREE
  if(scanserver) {
    // activate all octtrees until they don't fit anymore
    cout << "Locking managed octtrees in memory " << flush;
    bool stop = false;
    unsigned int loaded = 0;
    unsigned int dots = (octpts.size() / 20);
    if(dots == 0) dots = 1;
    vector<colordisplay*>::iterator it_remove_first = octpts.end();
    for(vector<colordisplay*>::iterator it = octpts.begin();
        it != octpts.end();
        ++it) {
      if(!stop) {
        // try to lock the octtree in cache
        try {
          Show_BOctTree<sfloat>* stree = dynamic_cast<Show_BOctTree<sfloat>*>(*it);
          stree->lockCachedTree();
          loaded++;
          if(loaded % dots == 0) cout << '.' << flush;
        } catch(runtime_error& e) {
          stop = true;
          it_remove_first = it;
        }
      }
      if(stop) {
        // delete the octtree, resize after iteration
        delete *it;
      }
    }
    // now remove iterators for deleted octtrees
    if(stop) octpts.erase(it_remove_first, octpts.end());
    cout << ' ' << loaded << " octtrees loaded." << endl;
  }
  
#endif // !COMPACT_TREE


  // load frames now that we know how many scans we actually loaded
  unsigned int real_end = min((unsigned int)(end), 
                              (unsigned int)(start + octpts.size() - 1));

  // necessary to save these to allow filtering of scans from view and reloading frames; could also make those global..
  startScanIdx = start;
  endScanIdx = real_end;
  readIni = readInitial;
  scanIOtype = type;
  
  if(readFrames(dir, start, real_end, readInitial, type))
    generateFrames(start, real_end, identity /*use .pose or identity*/);
  else cout << "Using existing frames..." << endl;

  cm->setCurrentType(PointType::USE_HEIGHT);
  //ColorMap cmap;
  //cm->setColorMap(cmap);
  resetMinMax(0);

  selected_points = new set<sfloat*>[octpts.size()];

  // sets (and computes if necessary) the pose that is used for the reset button
  if (originset) {
    setResetView(origin);
  }
  if (X != 0 || Y != 0 || Z != 0) {
    cout << "View set to: " << X << ", " << Y << ", " << Z << endl;
  }

  for (unsigned int i = 0; i < 256; i++) {
    keymap[i] = false;
  }
  setScansColored(colorScanVal);

  if (trajectoryFile.size() > 0) {
    ifstream file(trajectoryFile);

    if (file.good()) {
      double tmp[3];
      string line;
      while (getline(file, line)) {
        istringstream iss(line);
        iss >> tmp;

        double* position = new double[3];
        position[0] = tmp[0];
        position[1] = tmp[1];
        position[2] = -tmp[2];

        trajectory.push_back(position);
      }

      cout << "Loaded trajectory from file with " << trajectory.size() << " positions." << endl;

      file.close();
    } else {
      cout << "Couldn't open trajectory file for reading!" << endl;
    }
  }
}

void deinitShow()
{
  static volatile bool done = false;
  if(done) return;
  done = true;
  
  cout << "Cleaning up octtrees and scans." << endl;
  if(octpts.size()) {
    // delete octtrees to release the cache locks within
    for(vector<colordisplay*>::iterator it = octpts.begin();
        it!= octpts.end();
        ++it) {
      delete *it;
    }
  }
  
  Scan::closeDirectory();

  for (double* it : trajectory) {
    delete it;
  }
  trajectory.clear();
}

/**
 * Global program scope destructor workaround to clean up data regardless of
 * the way of program exit.
 */
struct Deinit { ~Deinit() { deinitShow(); } } deinit;
