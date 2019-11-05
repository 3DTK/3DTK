/*
 * show_common implementation
 *
 * Copyright (C) Kai Lingemann, Andreas Nuechter, Jan Elseberg, Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#ifndef __SHOW_COMMON_H__
#define __SHOW_COMMON_H__

#ifdef _WIN32
#define  _USE_MATH_DEFINES
#include <windows.h>
#endif

#ifdef WITH_GLEE
#include "GLee.h"
#endif

#include "show/show_Boctree.h"
#include "show/compacttree.h"
#include "show/NurbsPath.h"
#include "show/vertexarray.h"
#include "show/program_options.h"
#ifndef DYNAMIC_OBJECT_REMOVAL
#include "slam6d/scan.h"
#include "slam6d/managedScan.h"
#else
#include "veloslam/veloscan.h"
#endif
#include "show/show.h"
#ifdef WITH_OPENGL
#include "GL/glui.h"  /* Header File For The glui functions */
#else
#include "show/dummygl.h"
#endif
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <map>
#include <functional>
#include <mutex>
#include <condition_variable>

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
 * Display objects from the loadObj parameter.
 */
extern std::vector< ::SDisplay*> displays;

/**
 * the octrees that store the points for each scan
 */
//Show_BOctTree **octpts;
extern std::vector<colordisplay*> octpts;
/**
 * Storing the base directory
 */
extern std::string scan_dir;

/**
 * Storing the ID of the main windows
 */
extern int window_id;

/**
 * Size of points
 */
extern GLfloat pointsize;
extern int     anim_delay;

/**
 * Select Color Buffer
 */
extern GLenum buffermode;

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
extern int haveToUpdate;

extern std::function<void()> update_callback;

/**
 * Flag for invert the scene
 */
extern bool invert;

/**
 * Flag for indicating view mode
 * 0: default view
 * 1: top view
 * 2: rotate view
 */
extern int showViewMode;

/**
 * Flag for idicating camera add mode
 */
extern bool addCameraView;         //Is the view in add box mode?

/**
 * Storing the apex angle of the camera
 */
extern GLfloat cangle;          // Current camera opening mode
extern GLfloat cangle_old;

/**
 * Current rotation axis of the scene as quaternion
 */
extern GLdouble quat[4];
extern GLdouble Rquat[4];

/**
 * Current translation of the scene
 */
extern GLdouble X, Y, Z;
extern GLdouble RVX, RVY, RVZ;

/**
 * Center of Mass coordinates
 */
extern GLdouble CoM[3];

/**
 * parallel zoom (similar to apex angle) for parallel projection
 */
extern GLfloat pzoom;
extern GLfloat pzoom_old;

/**
 * Mode of the fog (exp, exp2, linear)
 */
extern GLint fogMode;

/**
 * Indicates if fog should be shown
 */
extern int show_fog;

/**
 * Indicates if the points should be shown
 */
extern int show_points;             // Show data points in the viewer?

/**
 * Indicates if camera boxes should be shown
 */
extern int show_cameras;             // Show the camera boxes in the viewer?

/**
 * Indicates if camera path or robot path should be shown
 */
extern int show_path;             // Show the camera movement path ?

/**
 * Indicates if scan poses should be shown
 */
extern int show_poses;            // Show the coordinate axes of the poses ?

/**
 * Camera navigation by mouse or by panel
 */
extern int cameraNavMouseMode;

extern double mouseNavX, mouseNavY;
extern int mouseNavButton;
extern int mousePresX, mousePresY;

extern double now_quat[4];     // Save current quaternion values for rotation mode
extern double tmp_quat[4];     // Save temporary quaternion values while dragging

extern double mouseRotX;
extern double mouseRotY;
extern double mouseRotZ;

extern bool keymap[256];

/**
 * draw scans in different color during animation
 */
extern int coloranim;

/**
 * hide the gui
 */
extern bool nogui;

extern bool hide_label;

/**
 * take a screenshot and exit
 */
extern bool takescreenshot;
extern std::string screenshot_filename;

/**
 * rendering a png is done in the background. If an animation is rendered,
 * multiple threads can run at the same time. We need to make sure to not run
 * too many threads at the same time and to wait for all threads to finish
 * before closing show.
 */

extern int png_workers;
extern int png_workers_max;
extern std::mutex png_workers_mutex;
extern std::condition_variable png_workers_cv;

//@@@
//int animate_both         = 0;             // Animate both scan matchin and path?

extern int frameNr;

extern int path3D;
extern float shifted;

// custom scan range selection from GUI
extern int startScanIdx;
extern int endScanIdx;
extern int startRangeScanIdx;
extern int endRangeScanIdx;
extern bool readIni;
extern IOType scanIOtype;

/**
 * Storing of all transformation (frames for animation) of all scans
 */
extern std::vector < std::vector <double*> > MetaMatrix;

/**
 * Storing of AlgoType for all frames
 */
extern std::vector < std::vector <Scan::AlgoType> > MetaAlgoType;

/**
 * Trajectory loaded from file for visualization
 */
extern std::vector<double*> trajectory;

/**
 * Window position
 */
extern int START_X;
extern int START_Y;
extern int START_WIDTH;
extern int START_HEIGHT;
// Current aspect ratio
extern GLdouble aspect;
extern bool advanced_controls;
extern bool invertMouseX, invertMouseY;
extern bool anim_convert_jpg;
extern bool captureMouseCursor;
extern bool hideWidgetsInFullscreen;

extern bool fullscreen;
extern int current_width;
extern int current_height;


extern double scale; // in m
// the following values are scale dependant, i.e. all values are in m
extern float neardistance;
extern double oldneardistance;
extern float maxfardistance;
extern double fardistance;
extern double oldfardistance;
extern double movementSpeed;
extern double defaultZoom;
extern GLfloat fogDensity;
extern double voxelSize;
extern GLfloat rzoom;


extern float adaption_rate;
extern float LevelOfDetail;

// Defines for Point Semantic
#define TYPE_UNKNOWN         0x0000
#define TYPE_OBJECT          0x0001
#define TYPE_GROUND          0x0002
#define TYPE_CEILING         0x0003

extern unsigned int cam_choice;

extern unsigned int path_iterator;
extern int oldcamNavMode;

/**
 * Animation sould be saved to file
 */
extern int save_animation;

/**
 * If true, interpolation for camera path is based on distance, else always
 * the same number of images between two cameras.
 */
extern int inter_by_dist;

/** some variables for the camera path **/
extern std::vector<PointXY> path_vectorX, path_vectorZ;
extern std::vector<PointXY> lookat_vectorX, lookat_vectorZ;
extern std::vector<PointXY> ups_vectorX, ups_vectorZ;
extern std::vector<Point> cams;
extern std::vector<Point> lookats;
extern std::vector<Point> ups;

extern NurbsPath cam_nurbs_path;
extern char *path_file_name;
extern char *pose_file_name;
extern float path_interp_factor;

/** Factor for saved image size */
extern int factor;

/**
 * program tries to have this framerate
 */
extern float idealfps;

extern float bgcolor[3];

/**
 * value of the listBox fo Color Value and Colormap
 */
extern int listboxColorVal;
extern int listboxColorMapVal;
extern int colorScanVal;
extern ScanColorManager *cm;
extern float mincolor_value;
extern float maxcolor_value;
//unsigned int  types = Point::USE_HEIGHT;
extern PointType pointtype;

/**
 * Contains the selected points for each scan
 */
extern std::set<sfloat *> *selected_points;
/**
 * Select single points?
 */
extern int select_voxels;
/**
 * Select or unselect points ?
 */
extern int selectOrunselect;
/** octree depth for selecting groups of points */
extern int selection_depth;
extern int brush_size;
extern char *selection_file_name;

// show_gl needs to know this function for correct handling of close event
void deinitShow();

extern int current_frame;

extern std::function<void(const std::string&)> loading_status;
extern std::function<void(int, int, int)> loading_progress;

#include "show/show_menu.h"
#include "show/show_animate.h"
#include "show/show_gl.h"

void setResetView(int origin);

/**
 * A function that read the .frame files created by slam6D
 *
 * @param dir the directory
 * @param start starting at scan number 'start'
 * @param end stopping at scan number 'end'
 * @param read a file containing a initial transformation matrix and apply it
 */
int readFrames(std::string dir, int start, int end, bool readInitial, IOType &type);

void generateFrames(int start, int end, bool identity);

void cycleLOD();

void reloadFrames();

/**
 * Read scan files and initialize OpenGL.
 */
void initShow(dataset_settings& dss, const window_settings& ws, const display_settings& ds);

void deinitShow();

/**
 * Signal handling functions
 */
void signal_segv(int v);
void signal_interrupt(int v);
void setSignalHandling();

extern std::function<void(int)> exitFunc;

/**
 * Convert quaternion into mouse rotation
 */
void QuatToMouseRot(Quaternion q, double& x, double& y, double& z);

/**
 * Global program scope destructor workaround to clean up data regardless of
 * the way of program exit.
 */
extern struct Deinit deinit;

#endif
