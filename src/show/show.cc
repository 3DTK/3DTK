/**
 * @file
 * @brief Implementation for displaying of a matched 3D scene
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Amuz T, Jacobs University Bremen, German
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "show_Boctree.h"
#include "show.h"
#include "camera.h"
#include "NurbsPath.h"
#include "vertexarray.h"
#include "../scan.h"
#include "glui/glui.h"  /* Header File For The glui funktions */
#include <fstream>
using std::ifstream;
#include <stdexcept>
using std::exception;

#ifdef _MSC_VER
#include "..\..\Visual_Studio_Projects\6DSLAM\6D_SLAM\XGetopt.h"
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
#ifdef OPENMP
#define _OPENMP
#endif
#endif

#ifdef _OPENMP
#include <omp.h>
#endif

/**
 * This vector contains the pointer to a vertex array for
 * all colors (inner vector) and all scans (outer vector)
 */
vector< vector<vertexArray*> > vvertexArrayList;

/**
 * the octrees that store the points for each scan
 */
Show_BOctTree **octpts;
/**
 * Storing the base directory
 */
string scandir;

/**
 * Storing the ID of the main windows
 */
int window_id;

/**
 * Size of points
 */
GLfloat pointsize          = 1.7;
int     anim_delay         = 5;

/**
 * Indicator whether and how the drawing window
 * has to be updated.
 *
 * haveToUpdate == 1 redisplay
 * haveToUpdate == 2 reshape
 * haveToUpdate == 3 animation scan matching
 * haveToUpdate == 4 stop animation scan matching
 * haveToUpdate == 6 path animation
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
GLdouble cangle          = 60.0;          // Current camera opening mode

/**
 * Current rotation angle of the scene 
 */
GLdouble angle           = 0.0;

/**
 * Current rotation axis of the scene as quaternion
 */
GLdouble quat[4]         ={0.0, 0.0, 0.0, 1.0};

/**
 * Current translation of the scene 
 */
GLdouble X               = 0.0, Y               = 0.0, Z               = 0.0;

/**
 * parallel zoom (similar to apex angle) for parallel projection
 */
GLdouble pzoom;

/**
 * Density of the fog
 */ 
GLfloat fogDensity       = 0.001;

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
int mouseNavButton;

double mouseRotX = 0.0;
double mouseRotY = 0.0;

//@@@
//int animate_both         = 0;             // Animate both scan matchin and path?

int frameNr;
int scanNr;

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
//int START_WIDTH          = 1475;
//int START_HEIGHT         = 860;
int START_WIDTH          = 720;
int START_HEIGHT         = 576;
GLdouble aspect          = (double)START_WIDTH/(double)START_HEIGHT;          // Current aspect ratio

// Defines for Point Semantic
#define TYPE_UNKNOWN         0x0000
#define TYPE_OBJECT          0x0001
#define TYPE_GROUND          0x0002 
#define TYPE_CEILING         0x0003 

vector <Camera*> cam_list;
int cam_choice             = 0;
bool camMode               = false;
/**camera rotation variables**/
GLdouble quat1[4] = {0.0, 0.0, 0.0, 1.0};
GLdouble angle1  = 0.0f;
double axis1[3];
float x_c, y_c, z_c;
  
static unsigned int path_iterator = 0;
static int oldcamNavMode = 0;

/**
 * Animation sould be saved to file
 */
int save_animation         = 0;

/**some variables for the camera path**/

vector<PointXY> path_listXY, path_listXZ, path_vectorX, path_vectorZ;
vector<PointXY> lookat_listXY, lookat_listXZ, lookat_vectorX, lookat_vectorZ;
NurbsPath cam_nurbs_path;
char *path_file_name;
char *pose_file_name;
float flength;

/** Factor for saved image size */
int factor = 1;

/**
 * value of the listBox fo Color Value and Colormap
 */
int listboxColorVal = 0;
int listboxColorMapVal = 0;
ScanColorManager *cm;
float mincolor_value = 0.0;
float maxcolor_value = 0.0;
unsigned int  types = ScanColorManager::USE_HEIGHT;
int scans_colored = 0;


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
	  << "         (chose F from {uos, uos_map, uos_frames, uos_map_frames, old, rts, rts_map, ifp, riegl_txt, riegl_bin, zahn, ply, wrl, xyz, zuf, iais, front, x3d, rxp })" << endl
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
	  << bold << "  -r" << normal << " NR, " << bold << "--reduce=" << normal << "NR" << endl
	  << "         turns on octree based point reduction (voxel size=<NR>)" << endl
	  << endl
	  << bold << "  -s" << normal << " NR, " << bold << "--start=" << normal << "NR" << endl
	  << "         start at scan NR (i.e., neglects the first NR scans)" << endl
	  << "         [ATTENTION: counting naturally starts with 0]" << endl
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
int parseArgs(int argc,char **argv, string &dir, int& start, int& end, int& maxDist, int& minDist, 
              double &red, bool &readInitial, int &octree, unsigned int &types , reader_type &type)
{
  start   = 0;
  end     = -1; // -1 indicates no limitation
  maxDist = -1; // -1 indicates no limitation
  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  cout << endl;
  static struct option longopts[] = {
    { "format",          required_argument,   0,  'f' },  
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
    { 0,           0,   0,   0}                    // needed, cf. getopt.h
  };

  while ((c = getopt_long(argc, argv,"f:s:e:r:m:M:O:wtRadh", longopts, NULL)) != -1)
    switch (c)
	 {
	 case 's':
	   start = atoi(optarg);
	   if (start < 0) { cerr << "Error: Cannot start at a negative scan number.\n"; exit(1); }
	   break;
	 case 'e':
	   end = atoi(optarg);
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
	   octree = atoi(optarg);
	   break;
	 case 'f':
	   if (strcasecmp(optarg, "uos") == 0) type = UOS;
	   else if (strcasecmp(optarg, "uos_map") == 0) type = UOS_MAP;
	   else if (strcasecmp(optarg, "uos_frames") == 0) type = UOS_FRAMES;
	   else if (strcasecmp(optarg, "uos_map_frames") == 0) type = UOS_MAP_FRAMES;
	   else if (strcasecmp(optarg, "old") == 0) type = OLD;
	   else if (strcasecmp(optarg, "rts") == 0) type = RTS;
	   else if (strcasecmp(optarg, "rts_map") == 0) type = RTS_MAP;
	   else if (strcasecmp(optarg, "ifp") == 0) type = IFP;
	   else if (strcasecmp(optarg, "riegl_txt") == 0) type = RIEGL_TXT;
	   else if (strcasecmp(optarg, "riegl_bin") == 0) type = RIEGL_BIN;
	   else if (strcasecmp(optarg, "zahn") == 0) type = ZAHN;
	   else if (strcasecmp(optarg, "ply") == 0) type = PLY;
	   else if (strcasecmp(optarg, "wrl") == 0) type = WRL;
	   else if (strcasecmp(optarg, "xyz") == 0) type = XYZ;
	   else if (strcasecmp(optarg, "zuf") == 0) type = ZUF;
	   else if (strcasecmp(optarg, "iais") == 0) type = IAIS;
	   else if (strcasecmp(optarg, "front") == 0) type = FRONT;
	   else if (strcasecmp(optarg, "x3d") == 0) type = X3D;
	   else if (strcasecmp(optarg, "rxp") == 0) type = RXP;
	   else {
		abort ();
	   }
	   break;
	 case '?':
     usage(argv[0]);
     return 1;
   case 'R':
     types |= ScanColorManager::USE_REFLECTANCE;
     break;
   case 'a':
     types |= ScanColorManager::USE_AMPLITUDE;
     break;
   case 'd':
     types |= ScanColorManager::USE_DEVIATION;
     break;
   case 'h':
     types |= ScanColorManager::USE_HEIGHT;
     break;
   default:
     abort ();
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
  return 0;
}


/*
 * A function that read the .frame files created by slam6D
 *
 * @param dir the directory
 * @param start starting at scan number 'start'
 * @param end stopping at scan number 'end'
 * @param read a file containing a initial transformation matrix and apply it
 */
void readFrames(string dir, int start, int end, bool readInitial, reader_type &type)
{

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
  }

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
    int frameCounter = 0;

    while (frame_in.good()) {
      frameCounter++;	 
      double *transMatOpenGL = new double[16];
      int algoTypeInt;
      Scan::AlgoType algoType;
      try {
        double transMat[16];
        frame_in >> transMat >> algoTypeInt;
        algoType = (Scan::AlgoType)algoTypeInt;

        // convert to OpenGL coordinate system
        double mirror[16];
        M4identity(mirror);
        mirror[10] = -1.0;
        if (readInitial) {
          double tempxf[16];
          MMult(mirror, initialTransform, tempxf);
          memcpy(mirror, tempxf, sizeof(tempxf));
        }
        //@@@
        //	   memcpy(transMatOpenGL, transMat, 16*sizeof(double));
        MMult(mirror, transMat, transMatOpenGL);
      }
      catch (const exception &e) {   
        break;
      }
      // don't store the very first entry, since it's the identity matrix.	 
      if (frameCounter > 1)
      {
        Matrices.push_back(transMatOpenGL);
        algoTypes.push_back(algoType);
      }
    }
    MetaAlgoType.push_back(algoTypes);

    MetaMatrix.push_back(Matrices);

    if((type == UOS_MAP || type == UOS_MAP_FRAMES || type == RTS_MAP) && fileCounter == start+1) {
      MetaAlgoType.push_back(algoTypes);
      MetaMatrix.push_back(Matrices);
    }

    frame_in.close();
    frame_in.clear();
    cout << MetaMatrix.back().size() << " done." << endl;
  }
  if (MetaMatrix.size() == 0) {
    cerr << "ERROR: Missing or empty directory: " << dir << endl << endl;
    cerr << "aborting..." << endl;
    exit(1);
  }
}

//-----------------------------------------------------------------------------------


/*
 * create display lists
 * @to do general framework for color & type definitions
 */
void createDisplayLists(bool reduced, unsigned int types)
{
#ifndef USE_GL_POINTS
  for(unsigned int i = 0; i < Scan::allScans.size() ; i++) {

    // count points
    int color1 = 0, color2 = 0;
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
    
    // allocate memory
    vertexArray* myvertexArray1 = new vertexArray(color1);
    vertexArray* myvertexArray2 = new vertexArray(color2);

    // fill points
    color1 = 0, color2 = 0;
    if (reduced) {
      for (unsigned int jterator = 0; jterator < Scan::allScans[i]->get_points_red_size(); jterator++) {
        myvertexArray2->array[color2] = Scan::allScans[i]->get_points_red()[jterator][0];
        myvertexArray2->array[color2+1] = Scan::allScans[i]->get_points_red()[jterator][1];
        myvertexArray2->array[color2+2] = Scan::allScans[i]->get_points_red()[jterator][2];
          color2 += 3;
      }
    } else {
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

#else
//#ifdef USE_GL_POINTS
  cout << "Creating display octrees.." << endl;
  octpts = new Show_BOctTree*[Scan::allScans.size()];

#ifdef _OPENMP
  omp_set_num_threads(OPENMP_NUM_THREADS);
  omp_set_nested(1);
#pragma omp parallel for schedule(dynamic)
#endif
  for(int i = 0; i < (int)Scan::allScans.size() ; i++) {
    if (reduced) {
      double **pts = new double*[Scan::allScans[i]->get_points_red_size()];
      for (unsigned int jterator = 0; jterator < Scan::allScans[i]->get_points_red_size(); jterator++) {
        pts[jterator] = new double[3];
        pts[jterator][0] = Scan::allScans[i]->get_points_red()[jterator][0];
        pts[jterator][1] = Scan::allScans[i]->get_points_red()[jterator][1];
        pts[jterator][2] = Scan::allScans[i]->get_points_red()[jterator][2];
      }
      octpts[i] = new Show_BOctTree(pts, Scan::allScans[i]->get_points_red_size(), 50.0);  // TODO remove magic number
      for (unsigned int jterator = 0; jterator < Scan::allScans[i]->get_points_red_size(); jterator++) {
        delete[] pts[jterator];
      }
      delete[] pts;

    } else {
      if (types != ScanColorManager::USE_NONE) {
        unsigned int pointdim = cm->getPointDim();
        unsigned int nrpts = Scan::allScans[i]->get_points()->size();
        double **pts = new double*[nrpts];
        unsigned int counter;
        for (unsigned int jterator = 0; jterator < nrpts; jterator++) {
          counter = 0;
          pts[jterator] = new double[pointdim];
          pts[jterator][counter++] = Scan::allScans[i]->get_points()->at(jterator).x;
          pts[jterator][counter++] = Scan::allScans[i]->get_points()->at(jterator).y;
          pts[jterator][counter++] = Scan::allScans[i]->get_points()->at(jterator).z;
          if (types & ScanColorManager::USE_REFLECTANCE) {
            pts[jterator][counter++] = Scan::allScans[i]->get_points()->at(jterator).reflectance;
          }
          if (types & ScanColorManager::USE_AMPLITUDE) {
            pts[jterator][counter++] = Scan::allScans[i]->get_points()->at(jterator).amplitude;
          }
          if (types & ScanColorManager::USE_DEVIATION) {  
            pts[jterator][counter++] = Scan::allScans[i]->get_points()->at(jterator).deviation;
          }

        }
        Scan::allScans[i]->clearPoints();
        octpts[i] = new Show_BOctTree(pts, nrpts , 50.0, pointdim, cm);  //TODO remove magic number
        for (unsigned int jterator = 0; jterator < nrpts; jterator++) {
          delete[] pts[jterator];
        }
        delete[] pts;
      } else {
        unsigned int nrpts = Scan::allScans[i]->get_points()->size();
        double **pts = new double*[nrpts];
        for (unsigned int jterator = 0; jterator < nrpts; jterator++) {
          pts[jterator] = new double[3];
          pts[jterator][0] = Scan::allScans[i]->get_points()->at(jterator).x;
          pts[jterator][1] = Scan::allScans[i]->get_points()->at(jterator).y;
          pts[jterator][2] = Scan::allScans[i]->get_points()->at(jterator).z;
        }
        Scan::allScans[i]->clearPoints();
        octpts[i] = new Show_BOctTree(pts, nrpts , 50.0);  //TODO remove magic number
        for (unsigned int jterator = 0; jterator < nrpts; jterator++) {
          delete[] pts[jterator];
        }
        delete[] pts;
      }
    }
    cout << "Scan " << i << " octree finished. Deleting original points.." << endl;
  }
#endif
}

			    

/**
 * Main function.
 * Reads the scan (scan000.3d, ...) and frames files (scan000.frames, ...) from the data directory.
 * The frames are used for animation of the matching process.
 */

int main(int argc, char **argv){
  
  cout << "(c) University of Osnabrueck, 2006 - 2007" << endl << endl
	  << "Restricted Usage" << endl
	  << "Don't use without permission" << endl;

  if(argc <= 1){
    usage(argv[0]);
  }
  double red   = -1.0;
  int start = 0, end = -1, maxDist = -1, minDist = -1;
  string dir;
  bool readInitial = false;
  reader_type type  = UOS;
  int octree = 0;

  pose_file_name = new char[sizeof(GLUI_String)];
  path_file_name = new char[sizeof(GLUI_String)];
   
  strncpy(pose_file_name, "pose.dat", sizeof(GLUI_String));  
  strncpy(path_file_name, "file.path", sizeof(GLUI_String));  
  
  parseArgs(argc, argv, dir, start, end, maxDist, minDist, red, readInitial, octree, types, type);
  scandir = dir;

  // init and create display
  M4identity(view_rotate_button);
  obj_pos_button[0] = obj_pos_button[1] = obj_pos_button[2] = 0.0;

  /***************/
  /* init OpenGL */
  /***************/
  glutInit(&argc,argv);
  
  initScreenWindow();
  newMenu();

  if (invert)
    glClearColor(0.0, 0.0, 0.0, 0.0);
  else
    glClearColor(1.0, 1.0, 1.0, 1.0);

  scanNr = frameNr = -1;

  // read frames first, to get notifyied of missing frames before all scans are read in
  readFrames(dir, start, end, readInitial, type);

  // Get Scans
  Scan::readScans(type, start, end, dir, maxDist, minDist, 0);
  
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


  cout << "Exporting the trajectory to \"trajectory.dat\"." << endl;
  ofstream traj("trajectory.dat");
  double *test;
  traj << "# corrected pose (x,y,z), original pose (x,y,z)" << endl;
  for(unsigned int i = 0; i<Scan::allScans.size(); i++){
    if(frameNr > -1 && frameNr < (int)MetaMatrix[1].size()) {
	 if (MetaAlgoType[i][frameNr] == Scan::INVALID) continue;
	 test = MetaMatrix[i][frameNr];
    } else {
	 test = MetaMatrix[i].back();
    }
    //glVertex3f(test[12], test[13] + 500, test[14]);
    traj << test[12] << " " << test[13] << " " << (fabs(test[14]) < 0.0001 ? test[14]:-1.0*test[14])  << " "
	    << Scan::allScans[i]->get_rPos()[0] << " "
	    << Scan::allScans[i]->get_rPos()[1] << " "
	    << Scan::allScans[i]->get_rPos()[2] 
	    << endl;
  }
  traj.close();
  traj.clear();
 
  cm = new ScanColorManager(4096, types);
  if (red > 0) {
    createDisplayLists(true, types);
  } else {
    createDisplayLists(false, types);
  }
  cm->setCurrentType(ScanColorManager::USE_HEIGHT);
  ColorMap cmap;
  cm->setColorMap(cmap);
  resetMinMax(0);

  glutMainLoop();

  delete [] path_file_name;
  delete [] pose_file_name;
}
