/**
 * @file
 * @brief Displaying of a matched 3D scene
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SHOW_H__
#define __SHOW_H__

#include <stdlib.h>
#include "GL/glut.h"    /* Header File For The glu toolkit */

#include "../scan.h"

#ifdef _MSC_VER
  #define  _USE_MATH_DEFINES
  #include <windows.h>
#else
  #include <unistd.h>
#endif

#include <math.h>
#ifdef _MSC_VER
#define strcasecmp _stricmp
#define strncasecmp _strnicmp
#else
#include <strings.h>
#endif
#include <fstream>

#include <iostream>
using std::cout;
using std::cerr;
using std::endl;
using std::ofstream;
using std::ios;
using std::ifstream;

#include <vector>
using std::vector;

#include <GL/gl.h>	    /* Header File For The OpenGL32 Library */
#include "glui/glui.h"  /* Header File For The glui funktions */

#ifndef _MSC_VER
#include <getopt.h>
#else
#include "..\..\Visual_Studio_Projects\6DSLAM\6D_SLAM\XGetopt.h"
#endif


/** for Glut display mode */
#define RGBA 4   ///< colors for GLUT display
#define RGB 3    ///< colors for GLUT display

/** float value "nearly" zero? */
#define DIV_EPSILON     0.001
/** comparing two floats/doubles */
#define COMPARE_EPSILON 0.000001

#define BUFSIZE 1048576 ///< defining the buffer size

#include "vertexarray.h"
#include "show1.icc"
#include "../globals.icc"
#include "camera.h"
#include "PathGraph.h"
#include "NurbsPath.h"
#include "show.icc"

void lookAt(const double dir[3],
 		  const double X, const double Y, const double Z,
 		  double *mat, const double *up);


void CallBackReshapeFunc(int width, int height);
void CallBackIdleFunc(void);
void DisplayItFunc(GLenum mode);
void DrawPoints(GLenum mode);
void glDumpWindowPPM(const char *filename, GLenum mode);
void ProcessHitsFunc(GLint hits, GLuint buffer[],int button);
int parseArgs(int argc, char **argv, string &dir, int& start, int& end, int& maxDist, bool& wanim, bool &readInitial);
void usage(char * prog);
int query_extension(char* extName);
void myNewMenu();
void topView();
void resetView(int dummy);
void update_view_rotate(int);
void update_view_translation(int);
void startAnimation(int dummy);
void invertView(int dummy);
void callTopView(int dummy);
void drawCameras(void);
void callCameraView(int dummy);
void cameraView();
void callDeleteCamera(int dummy);

void callCameraMode(int dummy);

void drawPath();
void pathAnimate(int dummy);
void savePath(int dummy);
void loadPath(int dummy);
void CallBackKeyboardFunc(unsigned char key, int x, int y);
void CallBackMouseFunc(int button, int state, int x, int y);
void CallBackSpecialFunc(int key, int x, int y);
void InterfaceFunc(unsigned char key);
void calcLookAtPath();
void drawRobotPath(int dummy);
void pathAnimate1(int i);
int calcFrameNo();
int calcNoOfPoints(vector<PointXY>);


enum { ROTATION_X, ROTATION_RX, ROTATION_Y, ROTATION_RY, ROTATION_Z, ROTATION_RZ };
  /** enumeration for translation */
enum { TRANS_X, TRANS_RX, TRANS_Y, TRANS_RY, TRANS_Z, TRANS_RZ };
  /** enumeration for the menue */
enum { MENU_SCREEN, MENU_LIST, MENU_QUIT };





#endif

