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

#include "show/show.h"
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
#include "glui/glui.h"  /* Header File For The glui functions */
#include <fstream>
using std::ifstream;
#include <stdexcept>
using std::exception;
#include <algorithm>


#ifdef _MSC_VER
#include "XGetopt.h"
#else
#include <getopt.h>
#endif

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

//@@@
//int animate_both         = 0;             // Animate both scan matchin and path?

int frameNr = 0;

/**
 * Storing of all transformation (frames for animation) of all scans
 */
vector < vector <double*> > MetaMatrix;

/**
 * Storing of AlgoType for all frames
 */
vector < vector <Scan::AlgoType> > MetaAlgoType;

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

bool fullscreen    = false;
int current_width  = START_WIDTH;
int current_height = START_HEIGHT;


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

int current_frame = 0;
#include "show_menu.cc"
#include "show_animate.cc"
#include "show_gl.cc"

/**
 * Explains the usage of this program's command line parameters
 * @param prog name of the program
 */
void usage(char* prog)
{
#ifndef _MSC_VER
  const string bold("\033[1m");
  const string normal("\033[m");
#else
  const string bold("");
  const string normal("");
#endif

  cout << endl
       << bold << "USAGE " << normal << endl
       << "   " << prog << " [options] directory" << endl << endl;
  cout << bold << "OPTIONS" << normal << endl

       << bold << "  -e" << normal << " NR, " << bold << "--end=" << normal << "NR" << endl
       << "         end after scan NR" << endl
       << endl
       << bold << "  -f" << normal << " F, " << bold << "--format=" << normal << "F" << endl
       << "         using shared library F for input" << endl
       << "         (chose F from {uos, uos_map, uos_rgb, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_txt, riegl_rgb, riegl_bin, zahn, ply, wrl, xyz, zuf, iais, front, x3d, rxp, ais })" << endl
       << endl
       << bold << "  -F" << normal << " NR, " << bold << "--fps=" << normal << "NR [default: 20]" << endl
       << "         will attempt to display points with a framerate of NR" << endl
       << endl
       << bold << "  -l" << normal << " FILE, " << bold << "--loadObj=" << normal <<
    "FILE" << endl
       << "         load objects specified in <FILE>" << endl
       << endl
       << endl
       << bold << "  -m" << normal << " NR, " << bold << "--max=" << normal << "NR" << endl
       << "         neglegt all data points with a distance larger than NR 'units'" << endl
       << endl
       << bold << "  -M" << normal << " NR, " << bold << "--min=" << normal << "NR" << endl
       << "         neglegt all data points with a distance smaller than NR 'units'" << endl
       << endl
       << bold << "  -O" << normal << "NR (optional), " << bold << "--octree=" << normal << "NR (optional)" << endl
       << "         use randomized octree based point reduction (pts per voxel=<NR>)" << endl
       << "         requires " << bold << "-r" << normal <<" or " << bold << "--reduce" << endl
       << endl
       << bold << "  -o" << normal << " NR, " << bold << "--origin=" << normal << "NR (optional)" << endl
       << "         sets the starting and reset position to: " << endl
       << "           0 = the origin of the coordinate system (default)" << endl
       << "           1 = the position of the first scan (default if --origin is in argument list)" << endl
       << "           2 = the center of the first scan" << endl
       << endl
            << bold << "  -S, --scanserver" << normal << endl
            << "           Use the scanserver as an input method and handling of scan data" << endl
            << endl
       << bold << "  -r" << normal << " NR, " << bold << "--reduce=" << normal << "NR" << endl
       << "         turns on octree based point reduction (voxel size=<NR>)" << endl
       << endl
       << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
       << "         start at scan NR (i.e., neglects the first NR scans)" << endl
       << "         [ATTENTION: counting naturally starts with 0]" << endl
       << endl
       << bold << "  -C" << normal << " NR, " << bold << "--scale=" << normal << "NR" << endl
       << "         scale factor to use (default: 0.01), modifies movement speed etc. " << endl
       << "         use 1 when point coordinates are in m, 0.01 when in cm and so forth. " << endl
       << "         " << endl
       << endl

    << bold << "  -R, --reflectance, --reflectivity" << normal << endl
       << "         use reflectivity values for coloring point clouds" << endl
       << "         only works when using octree display" << endl
       << endl
    << bold << "  -D, --temperature, --degree" << normal << endl
       << "         use temperature values for coloring point clouds" << endl
       << "         only works when using octree display" << endl
       << endl
    << bold << "  -a, --amplitude" << endl << normal
       << "         use amplitude values for coloring point clouds" << endl
       << "         only works when using octree display" << endl
       << endl
    << bold << "  -d, --deviation" << endl << normal
       << "         use amplitude values for coloring point clouds" << endl
       << "         only works when using octree display" << endl
       << endl
    << bold << "  -h, --height" << endl << normal
       << "         use y-values for coloring point clouds" << endl
       << "         only works when using octree display" << endl
       << endl
    << bold << "  -T, --type" << endl << normal
       << "         use type values for coloring point clouds" << endl
       << "         only works when using octree display" << endl
    << bold << "  -c, --color" << endl << normal
       << "         use color RGB values for coloring point clouds" << endl
    << bold << "  -b" << normal << " NR, " << bold << "--sphere=" << normal << "NR" << endl
       << "         map all measurements on a sphere (of radius NRcm)" << endl
    << bold << "  --saveOct" << endl << normal
       << "         stores all used scans as octrees in the given directory" << endl
       << "         All reflectivity/amplitude/deviation/type settings are stored as well." << endl
       << "         only works when using octree display" << endl
    << bold << "  --loadOct" << endl << normal
       << "         only reads octrees from the given directory" << endl
       << "         All reflectivity/amplitude/deviation/type settings are read from file." << endl
       << "         --reflectance/--amplitude and similar parameters are therefore ignored." << endl
       << "         only works when using octree display" << endl
    << endl << endl;

  exit(1);
}

/**
 * A function that parses the command-line arguments and sets the respective flags.
 *
 * @param argc the number of arguments
 * @param argv the arguments
 * @param dir parsing result - the directory
 * @param start parsing result - starting at scan number 'start'
 * @param end parsing result - stopping at scan number 'end'
 * @param maxDist parsing result - maximal distance
 * @param minDist parsing result - minimal distance
 * @param readInitial parsing result -  read a file containing a initial transformation matrix
 * @param type parsing result - file format to be read
 * @return 0, if the parsing was successful, 1 otherwise
 */
int parseArgs(int argc,char **argv,
              string &dir, int& start, int& end, int& maxDist, int& minDist, 
              double &red, bool &readInitial, int &octree,
              PointType &ptype, float &fps, string &loadObj,
              bool &loadOct, bool &saveOct, int &origin, double &scale,
              IOType &type, bool& scanserver, 
              double& sphereMode)
{
  unsigned int types = PointType::USE_NONE;
  start   = 0;
  end     = -1; // -1 indicates no limitation
  maxDist = -1; // -1 indicates no limitation
  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;
  
  WriteOnce<IOType> w_type(type);
  WriteOnce<int> w_start(start), w_end(end);

  cout << endl;
  static struct option longopts[] = {
    { "origin",          optional_argument,   0,  'o' },
    { "format",          required_argument,   0,  'f' },
    { "fps",             required_argument,   0,  'F' },
    { "scale",           required_argument,   0,  'C' },
    { "start",           required_argument,   0,  's' },
    { "end",             required_argument,   0,  'e' },
    { "reduce",          required_argument,   0,  'r' },
    { "max",             required_argument,   0,  'm' },
    { "min",             required_argument,   0,  'M' },
    { "octree",          optional_argument,   0,  'O' },
    { "reflectance",     no_argument,         0,  'R' },
    { "reflectivity",    no_argument,         0,  'R' },
    { "temperature",     no_argument,         0,  'D' },
    { "degree",          no_argument,         0,  'D' },
    { "amplitude",       no_argument,         0,  'a' },
    { "deviation",       no_argument,         0,  'd' },
    { "height",          no_argument,         0,  'h' },
    { "type",            no_argument,         0,  'T' },
    { "color",           no_argument,         0,  'c' },
    { "loadObj",         required_argument,   0,  'l' },
    { "saveOct",         no_argument,         0,  '0' },
    { "loadOct",         no_argument,         0,  '1' },
    { "advanced",        no_argument,         0,  '2' },
    { "scanserver",      no_argument,         0,  'S' },
    { "sphere",          required_argument,   0,  'b' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  while ((c = getopt_long(argc, argv,"F:f:s:e:r:m:M:O:o:l:C:SwtRDadhTcb", longopts, NULL)) != -1) {
    switch (c) {
      case 's':
        w_start = atoi(optarg);
        if (start < 0) {
          cerr << "Error: Cannot start at a negative scan number.\n"; exit(1);
        }
        break;
      case 'e':
        w_end = atoi(optarg);
        if (end < 0) {
          cerr << "Error: Cannot end at a negative scan number.\n"; exit(1);
        }
        if (end < start) {
          cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1);
        }
        break;
      case 'm':
        maxDist = atoi(optarg);
        break;
      case 'M':
        minDist = atoi(optarg);
        break;
      case 'r':
        red = atof(optarg);
        break;
      case 't':
        readInitial = true;
        break;
      case 'O':
        if (optarg) {
          octree = atoi(optarg);
        } else {
          octree = 1;
        }
        break;
      case 'f':
        try {
          w_type = formatname_to_io_type(optarg);
        } catch (...) { // runtime_error
          cerr << "Format " << optarg << " unknown." << endl;
          abort();
        }
        switch (w_type) {
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
        break;
      case '?':
        usage(argv[0]);
        return 1;
      case 'R':
        types |= PointType::USE_REFLECTANCE;
        break;
      case 'D':
        types |= PointType::USE_TEMPERATURE;
        break;
      case 'a':
        types |= PointType::USE_AMPLITUDE;
        break;
      case 'd':
        types |= PointType::USE_DEVIATION;
        break;
      case 'h':
        types |= PointType::USE_HEIGHT;
        break;
      case 'T':
        types |= PointType::USE_TYPE;
        break;
      case 'c':
        types |= PointType::USE_COLOR;
        break;
      case 'F':
        fps = atof(optarg);
        break;
      case 'C':
        scale = atof(optarg);
        break;
      case 'S':
        scanserver = true;
        break;
      case 'o':
        if (optarg) {
          origin = atoi(optarg);
        } else {
          origin = 1;
        }
        break;
      case '0':
        saveOct = true;
        break;
      case '1':
        loadOct = true;
        break;
      case 'l':
        loadObj = optarg;
        break;
      case '2':
        advanced_controls = true; 
        break;
      case 'b':
        sphereMode = atof(optarg);
        break;
      default:
        abort ();
    }
  }

  if (optind != argc-1) {
    cerr << "\n*** Directory missing ***" << endl;
    usage(argv[0]);
  }
  dir = argv[optind];

#ifndef _MSC_VER
  if (dir[dir.length()-1] != '/') dir = dir + "/";
#else
  if (dir[dir.length()-1] != '\\') dir = dir + "\\";
#endif
  
  parseFormatFile(dir, w_type, w_start, w_end);

  ptype = PointType(types);
  return 0;
}

void setResetView(int origin) {
  if (origin == 1) {
    // set origin to the pose of the first scan
    double *transmat = MetaMatrix[0].back();
    cout << transmat << endl;

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
  } else if (origin == 2) {
    // set origin to the center of the first octree
    double center[3], center_transformed[3];
#ifdef USE_COMPACT_TREE
    ((compactTree*)octpts[0])->getCenter(center);
#else
    ((Show_BOctTree<sfloat>*)octpts[0])->getCenter(center);
#endif
    transform3(MetaMatrix[0].back(), center, center_transformed);
    RVX = -center_transformed[0];
    RVY = -center_transformed[1];
    RVZ = -center_transformed[2];
    X = RVX;
    Y = RVY;
    Z = RVZ;
  } else if (origin == 3) {
    // set origin to the center of mass of all scans
  for (size_t i = 0; i < octpts.size(); ++i) {
    vector <sfloat*> points;
#ifndef USE_COMPACT_TREE
    BOctTree<sfloat>* cur_tree = ((Show_BOctTree<sfloat>*)octpts[i])->getTree();
    cur_tree->AllPoints( points );
#endif

    cout << "Scan " << i << " size: " << points.size() << endl;
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

  cout << "Center of Mass at: "
       << CoM[0] << ", " << CoM[1] << ", " << CoM[2] << endl;

  RVX = -CoM[0];
  RVY = -CoM[1];
  RVZ = -CoM[2];
  X = RVX;
  Y = RVY;
  Z = RVZ;
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

  for(std::vector<Scan*>::iterator it = Scan::allScans.begin();
      it != Scan::allScans.end();
      ++it) {
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

    for (int i = 0; i < 3; i++) {
      double *transMat = new double[16];

      if (identity) {
        M4identity(transMat);
        transMat[10] = -1.0;
      } else {
        EulerToMatrix4(Scan::allScans[index]->get_rPos(),
                       Scan::allScans[index]->get_rPosTheta(),
                       transMat );
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


void initShow(int argc, char **argv){

  /***************/
  /* init OpenGL */
  /***************/
  glutInit(&argc,argv);

  cout << "(wx)show - A highly efficient 3D point cloud viewer" << endl
       << "(c) Jacobs University Bremen gGmbH, Germany, since 2009" << endl
       << "    University of Osnabrueck, Germany, 2006 - 2009" << endl << endl;

  if(argc <= 1){
    usage(argv[0]);
  }

  double red   = -1.0;
  int start = 0, end = -1, maxDist = -1, minDist = -1;
  string dir;
  bool readInitial = false;
  IOType type  = UOS;
  int octree = 0;
  bool loadOct = false;
  bool saveOct = false;
  string loadObj;
  int origin = 0;
  double scale = 0.01; // in m
  bool scanserver = false;
  double sphereMode = 0.0;

  pose_file_name = new char[1024];
  path_file_name = new char[1024];
  selection_file_name = new char[1024];

  strncpy(pose_file_name, "pose.dat", 1024);
  strncpy(path_file_name, "path.dat", 1024);
  strncpy(selection_file_name, "selected.3d", 1024);

  parseArgs(argc, argv, dir, start, end, maxDist, minDist, red, readInitial,
            octree, pointtype, idealfps, loadObj, loadOct, saveOct, origin,
            scale, type, scanserver, sphereMode);

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
  
  for (ScanVector::iterator it = Scan::allScans.begin();
       it != Scan::allScans.end();
       ++it) {
    Scan* scan = *it;
    scan->setRangeFilter(maxDist, minDist);
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
  }
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
    scan->setOcttreeParameter(red, voxelSize, pointtype, loadOct, saveOct);
    
    DataOcttree* data_oct;
    try {
      data_oct = new DataOcttree(scan->get("octtree"));
    } catch(runtime_error& e) {
      cout << "Scan " << i
           << " could not be loaded into memory, stopping here."
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
  
  if(readFrames(dir, start, real_end, readInitial, type))
    generateFrames(start, real_end, true);
  
  cm->setCurrentType(PointType::USE_HEIGHT);
  //ColorMap cmap;
  //cm->setColorMap(cmap);
  resetMinMax(0);

  selected_points = new set<sfloat*>[octpts.size()];

  // sets (and computes if necessary) the pose that is used for the reset button
  setResetView(origin);

  for (unsigned int i = 0; i < 256; i++) {
    keymap[i] = false;
  }
  setScansColored(colorScanVal);
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
}

/**
 * Global program scope destructor workaround to clean up data regardless of
 * the way of program exit.
 */
struct Deinit { ~Deinit() { deinitShow(); } } deinit;
