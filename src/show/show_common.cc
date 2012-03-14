#ifdef WITH_GLEE
#include <GLee.h>
#endif

#include "show/show.h"
#include "show/show_Boctree.h"
#include "show/compacttree.h"
#include "show/NurbsPath.h"
#include "show/vertexarray.h"
#ifndef DYNAMIC_OBJECT_REMOVAL
#include "slam6d/scan.h"
#else
#include "veloslam/veloscan.h"
#endif
#include "glui/glui.h"  /* Header File For The glui functions */
#include <fstream>
using std::ifstream;
#include <stdexcept>
using std::exception;
#include <algorithm>

#ifdef WITH_METRICS
#include "scanserver/metrics.h"
#endif

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

#ifdef WITH_SCANSERVER
#include "scanserver/clientInterface.h"
#endif //WITH_SCANSERVER

#include "slam6d/point_type.h"
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
GLdouble quat[4]         ={0.0, 0.0, 0.0, 1.0};
GLdouble Rquat[4]         ={0.0, 0.0, 0.0, 1.0};

/**
 * Current translation of the scene 
 */
GLdouble X               = 0.0, Y               = 0.0, Z               = 0.0;
GLdouble RVX = 0.0, RVY = 0.0, RVZ = 0.0;

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
int START_WIDTH          = 720;
int START_HEIGHT         = 576;
GLdouble aspect          = (double)START_WIDTH/(double)START_HEIGHT;          // Current aspect ratio
bool advanced_controls = false;

bool fullscreen = false;
int current_width = START_WIDTH;
int current_height = START_HEIGHT;


// the following values are scale dependant, i.e. all values are in m
float neardistance = 0.10;
double oldneardistance = 0.10;
float maxfardistance = 400.0;; 
double fardistance = 400.0;
double oldfardistance = 40000.0;
double movementSpeed = 0.1;
double defaultZoom = 20.0;
GLfloat fogDensity       = 0.1;
double voxelSize = 0.20;


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

/**some variables for the camera path**/
vector<PointXY> path_vectorX, path_vectorZ,  lookat_vectorX, lookat_vectorZ, ups_vectorX, ups_vectorZ;
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
int selectOrunselect         = 1;
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
	  << bold << "  -r" << normal << " NR, " << bold << "--reduce=" << normal << "NR" << endl
	  << "         turns on octree based point reduction (voxel size=<NR>)" << endl
	  << endl
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << endl
	  << "         [ATTENTION: counting naturally starts with 0]" << endl
	  << endl
	  << bold << "  -S" << normal << " NR, " << bold << "--scale=" << normal << "NR" << endl
	  << "         scale factor to use (default: 0.01), modifies movement speed etc. " << endl
	  << "         use 1 when point coordinates are in m, 0.01 when in cm and so forth. " << endl
	  << "         " << endl
	  << endl
	  
    << bold << "  -R, --reflectance, --reflectivity" << normal << endl
	  << "         use reflectivity values for coloring point clouds" << endl
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
 * Make type, start and end a write once only class for parseFormatFile because
 * we know the directory only after the type, start and end parameters may have
 * been written already, so encapsulate this write once behaviour in this class.
 */
template<typename T>
class WriteOnce {
public:
  WriteOnce(T& value) : value(value), written(false) {}
  WriteOnce& operator=(const T& other) { if(!written) { value = other; written = true; } return *this; }
  operator T() const { return value; }
private:
  T& value;
  bool written;
};

/**
 * Parsing of a formats file in the scan directory for default type and scan
 * index ranges without overwriting user set parameters. Does nothing if
 * file doesn't exist.
 * 
 * @param dir directory the scans and format file are contained in
 * @param type which ScanIO to use for the scans in that directory
 * @param start index for the first valid scan
 * @param end index for the last valid scan
 */
void parseFormatFile(string& dir, WriteOnce<IOType>& type, WriteOnce<int>& start, WriteOnce<int>& end)
{
  ifstream file((dir+"format").c_str());
  if(!file.good()) return;
  
  string line, key, value, format;
  while(getline(file, line)) {
    size_t pos = line.find('=');
    key = trim(line.substr(0, pos - 0));
    value = trim(line.substr(pos+1));
    if(key == "format") {
      try {
        format = value;
        type = formatname_to_io_type(format.c_str());
      } catch (...) { // runtime_error
        cerr << "Error while parsing format file: Format '" << format << "' unknown." << endl;
        break;
      }
    } else if(key == "start") {
      stringstream str(value.c_str());
      int s;
      str >> s;
      start = s;
    } else if(key == "end") {
      stringstream str(value.c_str());
      int e;
      str >> e;
      end = e;
    } else {
      cerr << "Error while parsing format file: Unknown key '" << key << "'" << endl;
      break;
    }
  }
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
int parseArgs(int argc,char **argv, string &dir, int& start, int& end, int& maxDist, int& minDist, 
              double &red, bool &readInitial, int &octree, PointType &ptype, float &fps, string &loadObj, bool &loadOct, bool &saveOct, int &origin, double &scale, IOType &type)
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
    { "scale",           required_argument,   0,  'S' },
    { "start",           required_argument,   0,  's' },
    { "end",             required_argument,   0,  'e' },
    { "reduce",          required_argument,   0,  'r' },
    { "max",             required_argument,   0,  'm' },
    { "min",             required_argument,   0,  'M' },
    { "octree",          optional_argument,   0,  'O' },
    { "reflectance",     no_argument,         0,  'R' },
    { "reflectivity",    no_argument,         0,  'R' },
    { "amplitude",       no_argument,         0,  'a' },
    { "deviation",       no_argument,         0,  'd' },
    { "height",          no_argument,         0,  'h' },
    { "type",            no_argument,         0,  'T' },
    { "color",           no_argument,         0,  'c' },
    { "loadObj",         required_argument,   0,  'l' },
    { "saveOct",         no_argument,         0,  '0' },
    { "loadOct",         no_argument,         0,  '1' },
    { "advanced",        no_argument,         0,  '2' },
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  while ((c = getopt_long(argc, argv,"F:f:s:e:r:m:M:O:o:l:wtRadhTc", longopts, NULL)) != -1) {
    switch (c) {
      case 's':
        w_start = atoi(optarg);
        if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
        break;
      case 'e':
        w_end = atoi(optarg);
        if (end < 0)     { cerr << "Error: Cannot end at a negative scan number.\n"; exit(1); }
        if (end < start) { cerr << "Error: <end> cannot be smaller than <start>.\n"; exit(1); }
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
        break;
      case '?':
        usage(argv[0]);
        return 1;
      case 'R':
        types |= PointType::USE_REFLECTANCE;
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
      case 'S':
        scale = atof(optarg);
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
    double center[3];
#ifdef USE_COMPACT_TREE
    ((compactTree*)octpts[0])->getCenter(center);
#else
    ((Show_BOctTree<sfloat>*)octpts[0])->getCenter(center);
#endif
    RVX = -center[0];
    RVY = -center[1];
    RVZ = -center[2];
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

#ifndef WITH_SCANSERVER
  ifstream frame_in;
  int  fileCounter = start;
  string frameFileName;
  for (;;) {
    if (end > -1 && fileCounter > end) break; // 'nuf read
    frameFileName = dir + "scan" + to_string(fileCounter++,3) + ".frames";

    frame_in.open(frameFileName.c_str());

    // read 3D scan
    if (!frame_in.good()) break; // no more files in the directory

    cout << "Reading Frames for 3D Scan " << frameFileName << "...";
    vector <double*> Matrices;
    vector <Scan::AlgoType> algoTypes;

    while (frame_in.good()) {
      double *transMatOpenGL = new double[16];
      int algoTypeInt;
      Scan::AlgoType algoType;
      try {
        double transMat[16];
        frame_in >> transMat >> algoTypeInt;
        algoType = (Scan::AlgoType)algoTypeInt;
        
        //memcpy(transMatOpenGL, transMat, 16*sizeof(double));
        // apply mirror to convert (and initial frame if requested) the frame and save in opengl
        MMult(mirror, transMat, transMatOpenGL);
      }
      catch (const exception &e) {   
        break;
      }
#else //WITH_SCANSERVER
  for(std::vector<Scan*>::iterator it = Scan::allScans.begin(); it != Scan::allScans.end(); ++it) {
    vector<double*> Matrices;
    vector<Scan::AlgoType> algoTypes;
    
    // iterate over frames (stop if none were created) and pull/convert the frames into local containers
    const FrameVector& frames = (*it)->getFrames();
    if(frames.size() == 0) break;
    for(FrameVector::const_iterator jt = frames.begin(); jt != frames.end(); ++jt) {
      double *transMatOpenGL = new double[16];
      Scan::AlgoType algoType = (Scan::AlgoType)jt->type;
      
      // apply mirror to convert (and initial frame if requested) the frame and save in opengl
      MMult(mirror, jt->transformation, transMatOpenGL);
#endif //WITH_SCANSERVER
      Matrices.push_back(transMatOpenGL);
      algoTypes.push_back(algoType);
    }
    
    MetaAlgoType.push_back(algoTypes);
    MetaMatrix.push_back(Matrices);
    
#ifndef WITH_SCANSERVER
    if((type == UOS_MAP || type == UOS_MAP_FRAMES || type == RTS_MAP) && fileCounter == start+1) {
      MetaAlgoType.push_back(algoTypes);
      MetaMatrix.push_back(Matrices);
    }

    frame_in.close();
    frame_in.clear();
    cout << MetaMatrix.back().size() << " done." << endl;
#else //WITH_SCANSERVER
    if((type == UOS_MAP || type == UOS_MAP_FRAMES || type == RTS_MAP) && it == Scan::allScans.begin()) {
      MetaAlgoType.push_back(algoTypes);
      MetaMatrix.push_back(Matrices);
    }
#endif //WITH_SCANSERVER
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
        EulerToMatrix4(Scan::allScans[index]->get_rPos(), Scan::allScans[index]->get_rPosTheta(), transMat ); 
      }

      Matrices.push_back(transMat);
      algoTypes.push_back(Scan::ICP);

    }
    index++;
    MetaAlgoType.push_back(algoTypes);
    MetaMatrix.push_back(Matrices);
  }
}



/*
 * create display lists
 * @to do general framework for color & type definitions
 */
void createDisplayLists(bool reduced)
{
  for(unsigned int i = 0; i < Scan::allScans.size() ; i++) {

    // count points
    int color1 = 0, color2 = 0;
#ifndef WITH_SCANSERVER
    if (!reduced) {
      for (unsigned int jterator = 0; jterator < Scan::allScans[i]->get_points()->size(); jterator++) {
        if (Scan::allScans[i]->get_points()->at(jterator).type & TYPE_GROUND) {
          color1++;
        } else {
          color2++;
        }
      }
    } else {
      color2 = 3* Scan::allScans[i]->get_points_red_size();
    }
#else
    // TODO: points isn't used anymore, but it could be filled with empty points but types
    color2 = 3* Scan::allScans[i]->getCountReduced();
#endif
    
    // allocate memory
    vertexArray* myvertexArray1 = new vertexArray(color1);
    vertexArray* myvertexArray2 = new vertexArray(color2);

    // fill points
    color1 = 0, color2 = 0;
    if (reduced) {
#ifndef WITH_SCANSERVER
      for (int jterator = 0; jterator < Scan::allScans[i]->get_points_red_size(); jterator++) {
        myvertexArray2->array[color2] = Scan::allScans[i]->get_points_red()[jterator][0];
        myvertexArray2->array[color2+1] = Scan::allScans[i]->get_points_red()[jterator][1];
        myvertexArray2->array[color2+2] = Scan::allScans[i]->get_points_red()[jterator][2];
          color2 += 3;
      }
#else //WITH_SCANSERVER
      DataXYZ xyz_r(Scan::allScans[i]->getXYZReduced());
      for(int k = 0, i = 0; i < color2; ++i) {
        for(unsigned int j = 0; j < 3; ++j) {
          myvertexArray2->array[k++] = xyz_r[i][j];
        }
      }
#endif //WITH_SCANSERVER

    } else {
#ifndef WITH_SCANSERVER
      // TODO: still no colors, get_points is gone and types aren't supported at the moment
      for (unsigned int jterator = 0; jterator < Scan::allScans[i]->get_points()->size(); jterator++) {
        if (Scan::allScans[i]->get_points()->at(jterator).type & TYPE_GROUND) {
          myvertexArray1->array[color1] = Scan::allScans[i]->get_points()->at(jterator).x;
          myvertexArray1->array[color1+1] = Scan::allScans[i]->get_points()->at(jterator).y;
          myvertexArray1->array[color1+2] = Scan::allScans[i]->get_points()->at(jterator).z;
          color1 += 3;
        } else {
          myvertexArray2->array[color2] = Scan::allScans[i]->get_points()->at(jterator).x;
          myvertexArray2->array[color2+1] = Scan::allScans[i]->get_points()->at(jterator).y;
          myvertexArray2->array[color2+2] = Scan::allScans[i]->get_points()->at(jterator).z;
          color2 += 3;
        }
      }
#endif //WITH_SCANSERVER
    }

    glNewList(myvertexArray1->name, GL_COMPILE);
    //@
    //glColor4d(0.44, 0.44, 0.44, 1.0);	
    //glColor4d(0.66, 0.66, 0.66, 1.0);	
    glVertexPointer(3, GL_FLOAT, 0, myvertexArray1->array);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, myvertexArray1->numPointsToRender);
    glDisableClientState(GL_VERTEX_ARRAY);
    glEndList();	

    glNewList(myvertexArray2->name, GL_COMPILE);
    //glColor4d(1.0, 1.0, 1.0, 1.0);	
    //glColor4d(0.0, 0.0, 0.0, 1.0);	
    glVertexPointer(3, GL_FLOAT, 0, myvertexArray2->array);
    glEnableClientState(GL_VERTEX_ARRAY);
    glDrawArrays(GL_POINTS, 0, myvertexArray2->numPointsToRender);
    glDisableClientState(GL_VERTEX_ARRAY);
    glEndList();	
    
    // append to vector
    vector<vertexArray*> vvertexArray;
    vvertexArray.push_back(myvertexArray1);
    vvertexArray.push_back(myvertexArray2);
    vvertexArrayList.push_back(vvertexArray);
  }
}

void cycleLOD() {
  LevelOfDetail = 0.00001;
  for (unsigned int i = 0; i < octpts.size(); i++)
    octpts[i]->cycleLOD();
}


sfloat** createPointArray(Scan* scan, PointType& pointtype)
{
#ifdef WITH_SCANSERVER
  // access data with prefetching
  pointtype.useScan(scan);
#endif //WITH_SCANSERVER
  
  // create a float array with requested attributes by pointtype via createPoint
  unsigned int nrpts = scan->getCount();
  sfloat** pts = new sfloat*[nrpts];
  for(unsigned int i = 0; i < nrpts; i++) {
#ifndef WITH_SCANSERVER
    pts[i] = pointtype.createPoint<sfloat>(scan->get_points()->at(i));
#else
    pts[i] = pointtype.createPoint<sfloat>(i);
#endif
  }
  
#ifndef WITH_SCANSERVER
  // unload full points which are not needed anymore
  scan->clearPoints();
#else //WITH_SCANSERVER
  // unlock access to data
  pointtype.clearScan();
#endif
  
  return pts;
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

#ifdef WITH_SCANSERVER
  ClientInterface* client;
  try {
    client = ClientInterface::create();
  } catch(std::runtime_error& e) {
    cerr << "ClientInterface could not be created: " << e.what() << endl;
    cerr << "Start the scanserver first." << endl;
    exit(-1);
  }
#endif

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

  pose_file_name = new char[1024];
  path_file_name = new char[1024];
  selection_file_name = new char[1024];
   
  strncpy(pose_file_name, "pose.dat", 1024);  
  strncpy(path_file_name, "path.dat", 1024);  
  strncpy(selection_file_name, "selected.3d", 1024);  
  
  parseArgs(argc, argv, dir, start, end, maxDist, minDist, red, readInitial,
  octree, pointtype, idealfps, loadObj, loadOct, saveOct, origin, scale, type);

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
//  oldfardistance *= scale;

  ////////////////////////
  SDisplay::readDisplays(loadObj, displays);
  ////////////////////

  if (type == OCT) {
    loadOct = true;
  }

  // if we want to load display file get pointtypes from the files first
  if(loadOct) {
    string scanFileName = dir + "scan" + to_string(start,3) + ".oct";
    cout << "Getting point information from " << scanFileName << endl;
    cout << "Attention! All subsequent oct-files must be of the same type!" << endl;

    pointtype = BOctTree<sfloat>::readType(scanFileName);
  }
  scan_dir = dir;

  // init and create display
  M4identity(view_rotate_button);
  obj_pos_button[0] = obj_pos_button[1] = obj_pos_button[2] = 0.0;
  
  // Loading scans, reducing, loading frames and generation if neccessary
#ifdef WITH_SCANSERVER
  // load all available scans
  Scan::readScans(type, start, end, dir, maxDist, minDist, 0);
  
  // apply (on-demand) reduction
  if(red > 0) {
    // create show-only reduced points
    for(vector<Scan*>::iterator it = Scan::allScans.begin(); it != Scan::allScans.end(); ++it) {
      Scan* scan = *it;
      scan->calcReducedShow(red, octree);
    }
  }
  
  // generate frames later after on-cache-full event has been handled
#else //WITH_SCANSERVER
  // read frames first, to get notified of missing frames before all scans are read in
  int r = readFrames(dir, start, end, readInitial, type);
  
  // Get Scans and create frames if needed
  if (!loadOct) {
#ifndef DYNAMIC_OBJECT_REMOVAL
    Scan::readScans(type, start, end, dir, maxDist, minDist, 0);
#else
    VeloScan::readScans(type, start, end, dir, maxDist, minDist, 0);
#endif
    if(r) generateFrames(start, start + Scan::allScans.size() - 1, false); 
  } else {
    cout << "Skipping files.." << endl;
    if(r) generateFrames(start, start + octpts.size() - 1, true); 
  }
  
  int end_reduction = (int)Scan::allScans.size();
  #ifdef _OPENMP
  #pragma omp parallel for schedule(dynamic)
  #endif
  for (int iterator = 0; iterator < end_reduction; iterator++) {
    // reduction filter for current scan!
    if (red > 0) {
      cout << "Reducing Scan No. " << iterator << endl;
      // TODO do another reduction so reflectance values etc are carried over
      Scan::allScans[iterator]->calcReducedPoints(red, octree);
    } // no copying necessary for show!
  }
#endif //WITH_SCANSERVER

  cm = new ScanColorManager(4096, pointtype);
  
#ifndef WITH_SCANSERVER
  // Generating octtrees
  if (loadOct) {
    for (int i = start; i <= end; i++) {
      string scanFileName = dir + "scan" + to_string(i,3) + ".oct";
      cout << "Reading octree " << scanFileName << endl;
#ifdef USE_COMPACT_TREE
      octpts.push_back(new compactTree(scanFileName, cm));
#else
      octpts.push_back(new Show_BOctTree<sfloat>(scanFileName, cm));
#endif
    }
  } else
#endif //WITH_SCANSERVER
  {
#ifdef USE_GL_POINTS // use octtrees
#ifdef USE_COMPACT_TREE
    cout << "Creating compact display octrees.." << endl;
#else
    cout << "Creating display octrees.." << endl;
#endif
#ifdef WITH_SCANSERVER
    if(loadOct) cout << "Loading octtrees from file where possible instead of creating them from scans." << endl;
    std::size_t free_mem = client->getCacheSize();
    for(unsigned int i = 0; i < Scan::allScans.size() ; i++) {
      Scan* scan = Scan::allScans[i];
    
    // create data structures
#ifdef USE_COMPACT_TREE // FIXME: change compact tree, then this case can be removed
      compactTree* tree;
      try {
        if (red > 0) { // with reduction, only xyz points
          tree = new compactTree(Array<float>(scan->getXYZReducedShow()).get(), scan->getXYZReducedShow().size(), voxelSize, pointtype, cm);
        } else { // without reduction, xyz + attribute points
          sfloat** pts = createPointArray(scan, pointtype);
          unsigned int nrpts = scan->getCount();
          tree = new compactTree(pts, nrpts, voxelSize, pointtype, cm);
          for(unsigned int i = 0; i < nrpts; ++i) delete[] pts[i]; delete[] pts;
        }
      } catch(runtime_error& e) {
        // handle scanserver max cache lock by not loading any more scans, no data has to be cleand up
        // Array<float> should never be created in thise case, PointType cleans the whole scan up on error by itself, before creating the float array
        cout << "Scan " << i << " could not be loaded into memory, stopping here." << endl;
        break;
      }
#else // FIXME: remove the case above
      BOctTree<float>* btree = 0;
      // set octtree parameters to invalidate cached ones with other parameters (changing range/height is already handled)
      stringstream s;
      s << red << " " << voxelSize << " " << pointtype.toFlags();
      scan->setOcttreeParameters(s.str().c_str());
      // try to get cached boctrees first
      DataOct* cached;
      try {
        cached = new DataOct(scan->getOcttree());
        if(!(*cached)) {
          // remove empty access object
          delete cached; cached = 0;
          // if loadOct is given, load the octtree under blind assumption that parameters match
          if (loadOct) {
            string scanFileName = dir + "scan" + to_string(start+i,3) + ".oct";
            btree = new BOctTree<sfloat>(scanFileName);
          } else {
            // create a new octtree and cache it
            if (red > 0) { // with reduction, only xyz points
              btree = new BOctTree<sfloat>(Array<float>(scan->getXYZReducedShow()).get(), scan->getXYZReducedShow().size(), voxelSize, pointtype, true);
            } else { // without reduction, xyz + attribute points
              sfloat** pts = createPointArray(scan, pointtype);
              unsigned int nrpts = scan->getCount();
              btree = new BOctTree<sfloat>(pts, nrpts, voxelSize, pointtype, true);
              for(unsigned int i = 0; i < nrpts; ++i) delete[] pts[i]; delete[] pts;
            }
          }
          // copy tree into cache
          cached = new DataOct(scan->copyOcttreeToCache(btree));
          delete btree; btree = 0;
        }
      } catch(runtime_error& e) {
        // handle scanserver max cache lock by not loading any more scans, no data has to be cleand up
        // Array<float> should never be created in the case of reduction, for non reduced case the PointType cleans the whole scan up on error by itself, before creating the float array
        // delete tree if copy to cache failed
        if(btree != 0) { delete btree; btree = 0; }
        cout << "Scan " << i << " could not be loaded into memory, stopping here." << endl;
        break;
      }
      // check if the octtree would actually fit with all the others
      unsigned int tree_size = cached->get().getMemorySize();
      if(tree_size > free_mem) {
        // delete dangling access object
        delete cached;
        cout << "Stopping at scan " << i << ", no more octtrees could fit in memory." << endl;
        break;
      } else {
        // subtract available memory
        free_mem -= tree_size;
      }
      // show structures
#ifdef USE_COMPACT_TREE
      compactTree* tree = new compactTree(cached, cm);
#else
      Show_BOctTree<sfloat>* tree = new Show_BOctTree<sfloat>(scan, cached, cm);
#endif
      // unlock cached octtree to enable creation of more octtres without blocking the space for full scan points
      tree->unlockCachedTree();
#endif //FIXME: remove this line

#else //WITH_SCANSERVER

    for(int i = 0; i < (int)Scan::allScans.size() ; i++) {
      Scan* scan = Scan::allScans[i];
      
    // create data structures
#ifdef USE_COMPACT_TREE
      compactTree* tree;
#else
      Show_BOctTree<float>* tree;
#endif
      if (red > 0) { // with reduction, only xyz points
#ifdef USE_COMPACT_TREE
        tree = new compactTree(scan->get_points_red(), scan->get_points_red_size(), voxelSize, pointtype, cm);
#else
        tree = new Show_BOctTree<sfloat>(scan->get_points_red(), scan->get_points_red_size(), voxelSize, pointtype, cm);
#endif
      } else { // without reduction, xyz + attribute points
        unsigned int nrpts = scan->getCount();
        sfloat** pts = createPointArray(scan, pointtype);
#ifdef USE_COMPACT_TREE
        tree = new compactTree(pts, nrpts, voxelSize, pointtype, cm);
#else
        tree = new Show_BOctTree<sfloat>(pts, nrpts, voxelSize, pointtype, cm);
#endif
        for(unsigned int i = 0; i < nrpts; ++i) delete[] pts[i]; delete[] pts;
      }
#ifndef USE_COMPACT_TREE
      unsigned int tree_size = tree->getMemorySize();
#endif
#endif //WITH_SCANSERVER

      // octtrees have been created successfully, register (and save) them
      if (saveOct) {
        string scanFileName = dir + "scan" + to_string(i+start,3) + ".oct";
        cout << "Saving octree " << scanFileName << endl;
        tree->serialize(scanFileName);
      }
      octpts.push_back(tree);
      
      // print something
#ifdef USE_COMPACT_TREE // TODO: change compact tree for memory footprint output, remove this case
      cout << "Scan " << i << " octree finished." << endl;
#else
      cout << "Scan " << i << " octree finished (";
      bool space = false;
      if(tree_size/1024/1024 > 0) {
        cout << tree_size/1024/1024 << "M";
        space = true;
      }      
      if((tree_size/1024)%1024 > 0) {
        if(space) cout << " ";
        cout << (tree_size/1024)%1024 << "K";
        space = true;
      }
      if(tree_size%1024 > 0) {
        if(space) cout << " ";
        cout << tree_size%1024 << "B";
      }
      cout << ")." << endl;
#endif
    }

#if defined WITH_SCANSERVER && !defined USE_COMPACT_TREE
/*
TODO: to maximize space for octtrees, implement a heuristic to remove all
CacheObjects sequentially from the start and pack octtrees one after another
until it reaches the maximum size allowed, resetting the index, but causing the
first to be locked again and stopping by catching the exception
set heuristic, do locking, catch exception, reset heuristic to default or old
*/
    // activate all octtrees until they don't fit anymore
    cout << "Locking octtrees in memory " << flush;
    bool stop = false;
    unsigned int loaded = 0;
    unsigned int dots = (octpts.size() / 20);
    if(dots == 0) dots = 1;
    vector<colordisplay*>::iterator it_remove_first = octpts.end();
    for(vector<colordisplay*>::iterator it = octpts.begin(); it != octpts.end(); ++it) {
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
#endif //WITH_SCANSERVER & COMPACT_TREE

#else // not using octtrees
    createDisplayLists(red > 0);
#endif
  }
  
#ifdef WITH_SCANSERVER
  // load frames now that we know how many scans we actually loaded
  unsigned int real_end = std::min((unsigned int)(end), (unsigned int)(start + octpts.size() - 1));
  int r = readFrames(dir, start, real_end, readInitial, type);
  if(r) generateFrames(start, real_end, true);
#endif //WITH_SCANSERVER
  
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
}

void deinitShow()
{
  static volatile bool done = false;
  if(done) return;
  done = true;
  
#ifdef WITH_SCANSERVER
  if(octpts.size()) {
#ifdef WITH_METRICS
    // print metric information
    ClientInterface::getInstance()->printMetrics();
#endif //WITH_METRICS
    cout << "Cleaning up scans and octtrees." << endl;
    // delete octtrees to release the cache locks within
    for(vector<colordisplay*>::iterator it = octpts.begin(); it!= octpts.end(); ++it) {
      delete *it;
    }
    // clean up Scans
    Scan::clearScans();
  }
  // close client interface
  ClientInterface::destroy();
#endif //WITH_SCANSERVER
}

/**
 * Global program scope destructor workaround to clean up data regardless of
 * the way of program exit.
 */
struct Deinit { ~Deinit() { deinitShow(); } } deinit;
