/**
 * @file
 * @brief Displaying of a matched 3D scene
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#ifndef __SHOW_H__
#define __SHOW_H__

#ifdef _MSC_VER
#define  _USE_MATH_DEFINES
#include <windows.h>
#endif
#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "PathGraph.h"

/** for Glut display mode */
#define RGBA 4   ///< colors for GLUT display
#define RGB 3    ///< colors for GLUT display

/** float value "nearly" zero? */
#define DIV_EPSILON     0.001
/** comparing two floats/doubles */
#define COMPARE_EPSILON 0.000001

#define BUFSIZE 1048576 ///< defining the buffer size

#include "show1.icc"
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
void CallBackMouseMotionFunc(int x, int y);
void CallBackSpecialFunc(int key, int x, int y);
void InterfaceFunc(unsigned char key);
void calcLookAtPath();
void drawRobotPath(int dummy);
void pathAnimate1(int i);
int calcFrameNo();
int calcNoOfPoints(vector<PointXY>);

void createDisplayLists(bool reduced=false);

enum { ROTATION_X, ROTATION_RX, ROTATION_Y, ROTATION_RY, ROTATION_Z, ROTATION_RZ };
  /** enumeration for translation */
enum { TRANS_X, TRANS_RX, TRANS_Y, TRANS_RY, TRANS_Z, TRANS_RZ };
  /** enumeration for the menue */
enum { MENU_SCREEN, MENU_LIST, MENU_QUIT };

#endif

