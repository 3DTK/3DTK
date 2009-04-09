/**
 * @file
 * @brief Implementation for displaying of a matched 3D scene
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Amuz T, Jacobs University Bremen, German
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "show.h"

/**
 * This vector contains the pointer to a vertex array for
 * all colors (inner vector) and all scans (outer vector)
 */
vector< vector<vertexArray*> > vvertexArrayList;

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
int     anim_delay         = 0;

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
GLint fogMode;

/**
 * Indicates if fog should be shown
 */
int show_fog             = 0;

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

//@@@
int animate_both         = 0;             // Animate both scan matchin and path?

int frameNr;
int scanNr;

/**
 * Storing of all transformation (frames for animation) of all scans
 */
vector < vector <double*> > MetaMatrix;

/**
 * Storing of colours for all frames
 */
vector < vector <double*> > MetaColour;

/**
 * Window position
 */
int START_X              = 0;
int START_Y              = 0;
int START_WIDTH          = 720;
int START_HEIGHT         = 576;

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

/**
 * Animation sould be saved to file
 */
int save_animation         = 0;

int enable_camera_angle    = 0;

/**some variables for the camera path**/

vector<PointXY> path_listXY, path_listXZ, path_vectorX, path_vectorZ;
vector<PointXY> lookat_listXY, lookat_listXZ, lookat_vectorX, lookat_vectorZ;
NurbsPath cam_nurbs_path;
char *path_file_name;
float flength;

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
	  << "Usage: " << prog << "  [-s NR] [-e NR] [-m NR] directory" << endl << endl;

  cout << "  -s NR   start at scan NR (i.e., neglects the first NR scans)" << endl
       << "          [ATTENTION: counting starts with 0]" << endl
	  << "  -e NR   end after scan NR" << "" << endl
	  << "  -m NR   set the maximal range distance to NR 'units' (unit of scan data, e.g. cm)" << endl
	  << bold << "  -f" << normal << " F, " << bold << "--format=" << normal << "F" << endl
	  << "         using shared library F for input" << endl
	  << "         (chose F from {uos, uos_map, old, rts, rts_map, ifp, riegl, zahn, ply})" << endl << endl
	  << endl;
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
		    bool &readInitial, reader_type &type)
{
  start   = 0;
  end     = -1; // -1 indicates no limitation
  maxDist = -1; // -1 indicates no limitation
  int  c;
  // from unistd.h
  extern char *optarg;
  extern int optind;

  cout << endl;
  while ((c = getopt (argc, argv, "f:s:e:m:M:p:wt")) != -1)
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
	 case 't':
	   readInitial = true;
	   break;
	 case 'f':
	   if (strcasecmp(optarg, "uos") == 0) type = UOS;
	   else if (strcasecmp(optarg, "uos_map") == 0) type = UOS_MAP;
	   else if (strcasecmp(optarg, "old") == 0) type = OLD;
	   else if (strcasecmp(optarg, "rts") == 0) type = RTS;
	   else if (strcasecmp(optarg, "rts_map") == 0) type = RTS_MAP;
	   else if (strcasecmp(optarg, "ifp") == 0) type = IFP;
	   else if (strcasecmp(optarg, "riegl") == 0) type = RIEGL;
	   else if (strcasecmp(optarg, "zahn") == 0) type = ZAHN;
	   else if (strcasecmp(optarg, "ply") == 0) type = PLY;
	   else if (strcasecmp(optarg, "wrl") == 0) type = WRL;
	   else if (strcasecmp(optarg, "xyz") == 0) type = XYZ;
	   else if (strcasecmp(optarg, "zuf") == 0) type = ZUF;
	   else if (strcasecmp(optarg, "iais") == 0) type = IAIS;
	   else if (strcasecmp(optarg, "front") == 0) type = FRONT;
	   else {
		abort ();
	   }
	   break;
	 case '?':
	   usage(argv[0]);
	   return 1;
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
void readFrames(string dir, int start, int end, bool readInitial)
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
    vector <double*> ColMatrices;
    int frameCounter = 0;
    
    while (frame_in.good()) {
	 frameCounter++;	 
	 double *transMatOpenGL = new double[16];
	 double *colourMat = new double[4];
	 try {
	   double transMat[16];
	   frame_in >> transMat;

	   for (int i = 0; i < 4 ; i++) {
		frame_in >> colourMat[i];
	   }

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
	   ColMatrices.push_back(colourMat);
	 }
    }
    MetaColour.push_back(ColMatrices);
    
    MetaMatrix.push_back(Matrices);
    
    /////////////////!!!!!!!!!!!!!!!!!!!!!!!!
    //@@@
//     if (fileCounter == start+1) {
//   	 MetaColour.push_back(ColMatrices);
//   	 MetaMatrix.push_back(Matrices);
//     }
    
    frame_in.close();
    frame_in.clear();
    cout << MetaMatrix.back().size() << " done." << endl;
  }
  if (MetaMatrix.size() == 0) cerr << "ERROR: Missing or empty directory: " << dir << endl << endl;
  /////////////////!!!!!!!!!!!!!!!!!!!!!!!!
  //@@@
  /*
  string filename;
  for (unsigned int i = 0; i < MetaMatrix.size(); i++) {
    filename = "6Dreg" + to_string(i,2) + ".dat";
    ofstream out(filename.c_str());
    out << MetaMatrix[i].back()[0] << " ";
    out << MetaMatrix[i].back()[1] << " ";
    out << MetaMatrix[i].back()[2] << " ";
    out << MetaMatrix[i].back()[3] << " " << endl;
    out << MetaMatrix[i].back()[4] << " ";
    out << MetaMatrix[i].back()[5] << " ";
    out << MetaMatrix[i].back()[6] << " ";
    out << MetaMatrix[i].back()[7] << " " << endl;
    out << MetaMatrix[i].back()[8] << " ";
    out << MetaMatrix[i].back()[9] << " ";
    out << MetaMatrix[i].back()[10] << " ";
    out << MetaMatrix[i].back()[11] << " " << endl;
    out << MetaMatrix[i].back()[12]*0.01 << " ";
    out << MetaMatrix[i].back()[13]*0.01 << " ";
    out << MetaMatrix[i].back()[14]*0.01 << " ";
    out << MetaMatrix[i].back()[15] << " ";
    out << endl;
  }
  */
  /*
  
    for (unsigned int i = 0; i < MetaMatrix.size(); i++) {
	 double euler[6];
	 cout << MetaMatrix[i].back() << endl;
	 Matrix4ToEuler(MetaMatrix[i].back(), euler+3, euler);
	 euler[3] *= -1.0;
	 euler[5] *= -1.0;
	 EulerToMatrix4(euler, euler+3, MetaMatrix[i].back());
	 cout << MetaMatrix[i].back() << endl << endl;
    }
    ofstream out("6Dreg.dat");
    for (unsigned int i = 0; i < MetaMatrix.size(); i++) {
      for (int j = 0; j < 16; j++) out << MetaMatrix[i].back()[j] << " ";
       out << endl;
    }
  */
  /*
   ofstream out1("trajectory.pose.dat");
   for (unsigned int i = 0; i < MetaMatrix.size(); i++) {
	double rPos[3], rPosTheta[3];
	Matrix4ToEuler(MetaMatrix[i].back(), rPosTheta, rPos);
	out1 << rPos[0] << " " << rPos[1] << " " << rPos[2] << " "
		<< rPosTheta[0] << " " << rPosTheta[1] << " " << rPosTheta[2] << endl;
	out1 << endl;
   }
  */
}

//-----------------------------------------------------------------------------------


/*
 * create display lists
 * @to do general framework for color & type definitions
 */
void createDisplayLists()
{
  for(unsigned int i = 0; i < Scan::allScans.size() ; i++) {
 
    // count points
    int color1 = 0, color2 = 0;
    for (unsigned int jterator = 0; jterator < Scan::allScans[i]->get_points()->size(); jterator++) {
	 if (Scan::allScans[i]->get_points()->at(jterator).type & TYPE_GROUND) {
	   color1++;
	 } else {
		color2++;
	 }
    }
    
    // allocate memory
    vertexArray* myvertexArray1 = new vertexArray(color1);
    vertexArray* myvertexArray2 = new vertexArray(color2);

    // fill points
    color1 = 0, color2 = 0;
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

  int start = 0, end = -1, maxDist = -1, minDist = -1;
  string dir;
  bool readInitial = false;
  reader_type type  = UOS;

  path_file_name = new char[255];
  
  parseArgs(argc, argv, dir, start, end, maxDist, minDist, readInitial, type);
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

  // Get Scans
  Scan::readScans(type, start, end, dir, maxDist, minDist, 0);
  readFrames(dir, start, end, readInitial);
  createDisplayLists();
  
  glutMainLoop();

  delete [] path_file_name;
}


