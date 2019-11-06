#include <cmath>
#include <csignal>

#include "show/show_common.h"

std::vector< ::SDisplay*> displays;
/**
 * the octrees that store the points for each scan
 */
//Show_BOctTree **octpts;
std::vector<colordisplay*> octpts;
/**
 * Storing the base directory
 */
std::string scan_dir;

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
 * haveToUpdate == 8 path and matching animation
 */
int haveToUpdate         = 0;

#ifdef WITH_OPENGL
std::function<void()> update_callback = glutPostRedisplay;
#else
std::function<void()> update_callback = 0;
#endif

/**
 * Flag for invert the scene
 */
bool invert              = true;

/**
 * Flag for indicating view mode
 * 0: default view
 * 1: top view
 * 2: rotate view
 */
int showViewMode         = 0;

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
 * Indicates if scan poses should be shown
 */
int show_poses           = 1;            // Show the coordinate axes of the poses ?

/**
 * Camera navigation by mouse or by panel
 */
int cameraNavMouseMode  = 1;

double mouseNavX, mouseNavY;
int mouseNavButton = -1;
int mousePresX, mousePresY;

double now_quat[4] = {1.0, 0.0, 0.0, 0.0};
double tmp_quat[4];

double mouseRotX = 0.0;
double mouseRotY = 0.0;
double mouseRotZ = 0.0;

bool keymap[256];

/**
 * draw scans in different color during animation
 */
int coloranim = 1;

/**
 * hide the gui
 */
bool nogui = false;

bool hide_label = false;

/**
 * take a screenshot and exit
 */
bool takescreenshot = false;
std::string screenshot_filename;

/**
 * rendering a png is done in the background. If an animation is rendered,
 * multiple threads can run at the same time. We need to make sure to not run
 * too many threads at the same time and to wait for all threads to finish
 * before closing show.
 */


int png_workers = 0;
int png_workers_max = OPENMP_NUM_THREADS; // FIXME: make this configurable
std::mutex png_workers_mutex;
std::condition_variable png_workers_cv;

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
std::vector < std::vector <double*> > MetaMatrix;

/**
 * Storing of AlgoType for all frames
 */
std::vector < std::vector <Scan::AlgoType> > MetaAlgoType;

/**
 * Trajectory loaded from file for visualization
 */
std::vector<double*> trajectory;

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
bool invertMouseX = false, invertMouseY = false;
bool anim_convert_jpg  = true;
bool captureMouseCursor = false;
bool hideWidgetsInFullscreen = true;

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
GLfloat rzoom          = 100.0;


float adaption_rate = 1.0;
float LevelOfDetail = 0.0001;

// Defines for Point Semantic
#define TYPE_UNKNOWN         0x0000
#define TYPE_OBJECT          0x0001
#define TYPE_GROUND          0x0002
#define TYPE_CEILING         0x0003

unsigned int cam_choice             = 0;

unsigned int path_iterator = 0;
int oldcamNavMode = 0;

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
std::vector<PointXY> path_vectorX, path_vectorZ;
std::vector<PointXY> lookat_vectorX, lookat_vectorZ;
std::vector<PointXY> ups_vectorX, ups_vectorZ;
std::vector<Point> cams;
std::vector<Point> lookats;
std::vector<Point> ups;

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

float bgcolor[3];

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
std::set<sfloat *> *selected_points;
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

// These only exist to be overwritten by QtShow right now
// We can remove these once we have proper logging
void noop1(const std::string&) {}
void noop2(int, int, int) {}
std::function<void(const std::string&)> loading_status = noop1;
std::function<void(int, int, int)> loading_progress = noop2;

std::function<void(int)> exitFunc = exit;

void setResetView(int origin) {
    if (origin == 0) {
        // set origin to the center of mass of all scans
        for (size_t i = 0; i < octpts.size(); ++i) {
            std::vector <sfloat*> points;
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
        Matrix4ToQuat(transmat, (double *)Rquat);
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
int readFrames(std::string dir, int start, int end, bool readInitial, IOType &type)
{

  // convert to OpenGL coordinate system
  double mirror[16];
  M4identity(mirror);
  mirror[10] = -1.0;

  double initialTransform[16];
  if (readInitial) {
    std::cout << "Initial Transform:" << std::endl;
    std::string initialTransformFileName = dir + "initital.frame";
    std::ifstream initial_in(initialTransformFileName.c_str());
    if (!initial_in.good()) {
      std::cout << "Error opening " << initialTransformFileName << std::endl;
      exit(-1);
    }
    initial_in >> initialTransform;
    std::cout << initialTransform << std::endl;

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
    std::vector<double*> Matrices;
    std::vector<Scan::AlgoType> algoTypes;

    // iterate over frames (stop if none were created) and
    // pull/convert the frames into local containers
    unsigned int frame_count;
    frame_count = (*it)->readFrames();

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
    std::cerr << "*****************************************" << std::endl;
    std::cerr << "** ERROR: No .frames could be found!   **" << std::endl;
    std::cerr << "*****************************************" << std::endl;
    std::cerr << " ERROR: Missing or empty directory: " << dir << std::endl << std::endl;
    return -1;
  }
  return 0;
}

void generateFrames(int start, int end, bool identity) {
  if (identity) {
    std::cout << "using Identity for frames " << std::endl;
  } else {
    std::cout << "using pose information for frames " << std::endl;
  }
  int  fileCounter = start;
  int index = 0;
  for (;;) {
    if (fileCounter > end) break; // 'nuf read
    fileCounter++;

    std::vector <double*> Matrices;
    std::vector <Scan::AlgoType> algoTypes;

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

  std::cout << "Reloading frame files..." << std::endl;

  MetaMatrix.clear();
  MetaAlgoType.clear();

  if (readFrames(scan_dir, startScanIdx, endScanIdx, readIni, scanIOtype))
    generateFrames(startScanIdx, endScanIdx, false /*use .pose*/);
}

/**
 * Copies settings stored in dataset_settings and window_settings to the appropriate
 * global or passed variables. The big goal is to not need this function at all,
 * but instead to use the _settings structs everywhere exclusively.
 */
void copy_settings_to_globals(
  const dataset_settings& dss, const window_settings& ws, const display_settings& ds,
  std::string &dir, int& start, int& end, double& maxDist, double& minDist,
  double &red, bool &readInitial, unsigned int &octree,
  PointType &ptype, float &fps, std::string &loadObj,
  bool &loadOct, bool &saveOct, bool &autoOct, int &origin, bool &originset,
  IOType &type, bool& scanserver,
  double& sphereMode, std::string& customFilter, std::string& trajectoryFile,
  int &stepsize, bool &identity)
{
  nogui = ws.nogui;
  fps = ws.max_fps;
  START_WIDTH = ws.dimensions.w;
  START_HEIGHT = ws.dimensions.h;
  aspect = (double)START_WIDTH/(double)START_HEIGHT;
  current_width  = START_WIDTH;
  current_height = START_HEIGHT;
  advanced_controls = ws.advanced_controls;
  invertMouseX = ws.invert_mouse_x;
  invertMouseY = ws.invert_mouse_y;
  captureMouseCursor = ws.capture_mouse;
  hideWidgetsInFullscreen = ws.hide_widgets;

  scale = dss.scale;
  cangle = ds.camera.fov;
  showViewMode = ds.init_with_viewmode;
  show_points = ds.draw_points;
  show_cameras = ds.draw_cameras;
  show_path = ds.draw_path;
  show_poses = ds.draw_poses;
  show_fog = ds.fog.type;
  fogDensity = ds.fog.density;
  pointsize = ds.pointsize;

  bgcolor[0] = dss.coloring.bgcolor.r;
  bgcolor[1] = dss.coloring.bgcolor.g;
  bgcolor[2] = dss.coloring.bgcolor.b;

  RVX = ds.camera.position.x;
  RVY = ds.camera.position.y;
  RVZ = ds.camera.position.z;
  X = RVX;
  Y = RVY;
  Z = RVZ;

  QuatToMouseRot(ds.camera.rotation, mouseRotX, mouseRotY, mouseRotZ);

  ptype = dss.coloring.ptype;
  listboxColorVal = (dss.coloring.colorval != -1 ? dss.coloring.colorval : 0);
  listboxColorMapVal = static_cast<int>(dss.coloring.colormap);
  mincolor_value = dss.coloring.colormap_values.min;
  maxcolor_value = dss.coloring.colormap_values.max;
  colorScanVal = dss.coloring.scans_colored;
  coloranim = !ds.color_animation;

  dir = dss.data_source;
  scanserver = dss.use_scanserver;
  start = dss.scan_numbers.min;
  end = dss.scan_numbers.max;
  type = dss.format;

  minDist = dss.distance_filter.min;
  maxDist = dss.distance_filter.max;
  red = dss.octree_reduction_voxel;
  octree = dss.octree_reduction_randomized_bucket;
  stepsize = dss.skip_files;

  origin = dss.origin_type;
  originset = dss.origin_type_set;
  sphereMode = dss.sphere_radius;

  saveOct = dss.save_octree;
  loadOct = dss.load_octree;
  autoOct = dss.cache_octree;

  takescreenshot = ws.take_screenshot;
  screenshot_filename = ws.screenshot_filename;
  loadObj = dss.objects_file_name;
  customFilter = dss.custom_filter;
  anim_convert_jpg = ds.anim_convert_jpg;
  trajectoryFile = dss.trajectory_file_name;
  identity = dss.identity;

  hide_label = ds.hide_label;

  // Start in RGB mode if the user requests it or they request no other coloring
  if (dss.coloring.explicit_coloring || (dss.coloring.colorval == -1 && dss.coloring.ptype.hasColor())) {
    colorScanVal = 2;
  }
}

void initShow(dataset_settings& dss, const window_settings& ws, const display_settings &ds){
  std::cout << "(wx)show - A highly efficient 3D point cloud viewer" << std::endl
       << "(c) University of Wuerzburg, Germany, since 2013" << std::endl
       << "    Jacobs University Bremen gGmbH, Germany, 2009 - 2013" << std::endl
       << "    University of Osnabrueck, Germany, 2006 - 2009" << std::endl << std::endl;

  double red   = -1.0;
  int start = 0, end = -1;
  double maxDist = -1, minDist = 0;
  std::string dir;
  bool readInitial = false;
  IOType type  = UOS;
  unsigned int octree = 0;
  bool loadOct = false;
  bool saveOct = false;
  bool autoOct = false;
  std::string loadObj;
  int origin = 0;
  bool originset = false;
  bool scanserver = false;
  double sphereMode = 0.0;
  bool customFilterActive = false;
  std::string customFilter;
  std::string trajectoryFile;
  int stepsize = 1;
  bool identity = false;

  pose_file_name = new char[1024];
  path_file_name = new char[1024];
  selection_file_name = new char[1024];

  strncpy(pose_file_name, "pose.dat", 1024);
  strncpy(path_file_name, "path.dat", 1024);
  strncpy(selection_file_name, "selected.3d", 1024);

  copy_settings_to_globals(dss, ws, ds, dir, start, end, maxDist, minDist, red,
                           readInitial, octree, pointtype, idealfps, loadObj,
                           loadOct, saveOct, autoOct, origin, originset,
                           type, scanserver, sphereMode, customFilter,
                           trajectoryFile, stepsize, identity);

  // modify all scale dependant variables
  movementSpeed   = 0.1 / scale;
  neardistance    = 0.1 / scale;
  oldneardistance = 0.1 / scale;
  maxfardistance  = 400 / scale;
  fardistance     = 400 / scale;
  fogDensity      = 0.1 * scale;
  defaultZoom     =  20 / scale;
  voxelSize       = 0.2 / scale;
  rzoom           = 1.0 / scale;

  loading_progress(0, 0, 0);
  loading_status("Loading extra objects");
  ////////////////////////
  SDisplay::readDisplays(loadObj, displays);
  ////////////////////

  if (type == OCT) {
    loadOct = true;
  }

  // if we want to load display file get pointtypes from the files first
  if (loadOct) {
    std::string scanFileName = dir + "scan" + to_string(start,3) + ".oct";
    std::cout << "Getting point information from " << scanFileName << std::endl;
    std::cout << "Attention! All subsequent oct-files must be of the same type!"
         << std::endl;
  }
  scan_dir = dir;

  // init and create display
  //M4identity(view_rotate_button);
  obj_pos_button[0] = obj_pos_button[1] = obj_pos_button[2] = 0.0;

  // Loading scans, reducing, loading frames and generation if neccessary

  loading_status("Loading scans");
  // We would have to hook loading_progress into there really uglily
  Scan::openDirectory(dss);

  if (Scan::allScans.size() == 0) {
    std::cerr << "No scans found. Did you use the correct format?" << std::endl;
    exit(-1);
  }

  if (sphereMode > 0.0) {
    cm = new ScanColorManager(4096, pointtype, /* animation_color = */ false);
  } else {
    cm = new ScanColorManager(4096, pointtype, /* animation_color = */ true);
  }

  loading_status("Creating display octrees");

#ifdef USE_COMPACT_TREE
  std::cout << "Creating compact display octrees.." << std::endl;
#else
  std::cout << "Creating display octrees.." << std::endl;
#endif

  if (loadOct)
    std::cout << "Loading octtrees from file where possible instead of creating them from scans."
         << std::endl;

  // for managed scans the input phase needs to know how much it can handle
  std::size_t free_mem = 0;
  if(scanserver)
    free_mem = ManagedScan::getMemorySize();

  loading_progress(0, 0, Scan::allScans.size());
  for(unsigned int i = 0; i < Scan::allScans.size(); ++i) {
    Scan* scan = Scan::allScans[i];

  // create data structures
#ifdef USE_COMPACT_TREE // FIXME: change compact tree, then this case can be removed
    compactTree* tree;
    try {
      if (loadOct) {
        std::string sfName = dir + "scan" + to_string(i,3) + ".oct";
        std::cout << "Load " << sfName;
        tree = new compactTree(sfName, cm);
        std::cout << " done." << std::endl;
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
            std::string sfName = dir + "scan" + to_string(i,3) + ".oct";
            tree->serialize(sfName);
          }
        }
      }
    } catch(...) {
      std::cout << "Scan " << i
           << " could not be loaded into memory, stopping here."
           << std::endl;
      break;
    }
#else // FIXME: remove the case above
    scan->setOcttreeParameter(red, voxelSize, pointtype, loadOct, saveOct, autoOct);

    DataOcttree* data_oct;
    try {
      data_oct = new DataOcttree(scan->get("octtree"));
    } catch(std::runtime_error& e) {
      std::cout << "Scan " << i
           << " could not be loaded into memory, stopping here. Reason: "
           << e.what()
           << std::endl;
      break;
    }
    BOctTree<float>* btree = &(data_oct->get());
    unsigned int tree_size = btree->getMemorySize();

    if(scanserver) {
      // check if the octtree would actually fit with all the others
      if(tree_size > free_mem) {
        delete data_oct;
        std::cout << "Stopping at scan " << i
             << ", no more octtrees could fit in memory." << std::endl;
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
    std::cout << "Scan " << i << " octree finished." << std::endl;
#else
    std::cout << "Scan " << i << " octree finished (";
    bool space = false;
    if (tree_size/1024/1024 > 0) {
      std::cout << tree_size/1024/1024 << "M";
      space = true;
    }
    if ((tree_size/1024)%1024 > 0) {
      if (space) std::cout << " ";
      std::cout << (tree_size/1024)%1024 << "K";
      space = true;
    }
    if (tree_size%1024 > 0) {
      if (space) std::cout << " ";
      std::cout << tree_size%1024 << "B";
    }
    std::cout << ")." << std::endl;
#endif
    loading_progress(i+1, 0, Scan::allScans.size());
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
    std::cout << "Locking managed octtrees in memory " << std::flush;
    bool stop = false;
    unsigned int loaded = 0;
    unsigned int dots = (octpts.size() / 20);
    if(dots == 0) dots = 1;
    std::vector<colordisplay*>::iterator it_remove_first = octpts.end();
    for(std::vector<colordisplay*>::iterator it = octpts.begin();
        it != octpts.end();
        ++it) {
      if(!stop) {
        // try to lock the octtree in cache
        try {
          Show_BOctTree<sfloat>* stree = dynamic_cast<Show_BOctTree<sfloat>*>(*it);
          stree->lockCachedTree();
          loaded++;
          if(loaded % dots == 0) std::cout << '.' << std::flush;
        } catch(std::runtime_error& e) {
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
    std::cout << ' ' << loaded << " octtrees loaded." << std::endl;
  }

#endif // !COMPACT_TREE

  loading_status("Loading frames");

  // load frames now that we know how many scans we actually loaded
  unsigned int real_end = std::min((unsigned int)(end),
                              (unsigned int)(start + octpts.size() - 1));

  // necessary to save these to allow filtering of scans from view and reloading frames; could also make those global..
  startScanIdx = start;
  endScanIdx = real_end;
  readIni = readInitial;
  scanIOtype = type;

  if(readFrames(dir, start, real_end, readInitial, type) != 0)
    generateFrames(start, real_end, identity /*use .pose or identity*/);
  else std::cout << "Using existing frames..." << std::endl;

  mapColorToValue(0); // uses listboxColorVal
  changeColorMap(0);  // uses listboxColorMapVal

  if (std::isnan(mincolor_value)) {
    mincolor_value = cm->getMin();
  }
  if (std::isnan(maxcolor_value)) {
    maxcolor_value = cm->getMax();
  }

  selected_points = new std::set<sfloat*>[octpts.size()];

  // sets (and computes if necessary) the pose that is used for the reset button
  if (originset) {
    setResetView(origin);
  }
  if (X != 0 || Y != 0 || Z != 0) {
    std::cout << "View set to: " << X << ", " << Y << ", " << Z << std::endl;
  }

  for (unsigned int i = 0; i < 256; i++) {
    keymap[i] = false;
  }
  setScansColored(0);

  if (trajectoryFile.size() > 0) {
    std::ifstream file(trajectoryFile);

    if (file.good()) {
      double tmp[3];
      std::string line;
      while (getline(file, line)) {
        std::istringstream iss(line);
        iss >> tmp;

        double* position = new double[3];
        position[0] = tmp[0];
        position[1] = tmp[1];
        position[2] = -tmp[2];

        trajectory.push_back(position);
      }

      std::cout << "Loaded trajectory from file with " << trajectory.size() << " positions." << std::endl;

      file.close();
    } else {
      std::cout << "Couldn't open trajectory file for reading!" << std::endl;
    }
  }


  loading_status("Done");
  loading_progress(0, 1, 0); // max < min means we're done
}

void deinitShow()
{
  static volatile bool done = false;
  if(done) return;
  done = true;

  // FIXME: this doesn't look right... I guess I need another condition
  // variable here because the below probably doesn't work if more than one
  // thread is currently locking the mutex
  {
	  std::unique_lock<std::mutex> lock(png_workers_mutex);
	  png_workers_cv.wait(lock, []{return png_workers == 0;});
  }

  std::cout << "Cleaning up octtrees and scans." << std::endl;
  if(octpts.size()) {
    // delete octtrees to release the cache locks within
    for(std::vector<colordisplay*>::iterator it = octpts.begin();
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

void signal_segv(int v)
{
  static bool segfault = false;
  if(!segfault) {
    segfault = true;
    std::cout << std::endl << "Segmentation fault" << std::endl;
    deinitShow();
  }
  exitFunc(-1);
}

void signal_interrupt(int v)
{
  static bool segfault = false;
  if(!segfault) {
    segfault = true;
    std::cout << std::endl << "Exiting by interrupt" << std::endl;
    deinitShow();
  }
  exitFunc(-1);
}

void setSignalHandling()
{
#ifndef WIN32
  struct sigaction actSigSegv;
  struct sigaction actSigInt;
  sigset_t sigset;

  sigemptyset(&sigset);
  sigaddset(&sigset, SIGSEGV);
  sigaddset(&sigset, SIGINT);
  sigaddset(&sigset, SIGTERM);

  memset(&actSigSegv, 0, sizeof(actSigSegv));
  memset(&actSigSegv, 0, sizeof(actSigInt));

  actSigSegv.sa_handler = signal_segv;
  actSigSegv.sa_mask = sigset;
  sigaction(SIGSEGV, &actSigSegv, NULL);

  actSigInt.sa_handler = signal_interrupt;
  actSigInt.sa_mask = sigset;
  sigaction(SIGINT, &actSigInt, NULL);
  sigaction(SIGTERM, &actSigInt, NULL);
#endif
}


void QuatToMouseRot(Quaternion q, double& x, double& y, double& z)
{
  Rquat[0] = q.x;
  Rquat[1] = q.y;
  Rquat[2] = q.z;
  Rquat[3] = q.w;
  for (char i = 0; i <= 3; i++) quat[i] = Rquat[i];
  double t[3] = {};
  double mat[16];
  QuatToMatrix4((const double *)quat, t, (double *)mat);
  glMultMatrixd((const double *)mat);
  double rot[3];
  Matrix4ToEuler((const double *)mat, rot);
  x = deg(rot[0]);
  y = deg(rot[1]);
  z = deg(rot[2]);
}

struct Deinit { ~Deinit() { deinitShow(); } } deinit;
