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
#elif WITH_FREEGLUT
#include <GL/freeglut.h>
#else
#include <GL/glut.h>
#endif

#include <string>
using std::string;
#include <vector>
using std::vector;

#include "show/PathGraph.h"
#include "show/colormanager.h"
#include "show/scancolormanager.h"

/** for Glut display mode */
#define RGBA 4   ///< colors for GLUT display
#define RGB 3    ///< colors for GLUT display

/** float value "nearly" zero? */
#define DIV_EPSILON     0.001
/** comparing two floats/doubles */
#define COMPARE_EPSILON 0.000001

#define BUFSIZE 1048576 ///< defining the buffer size

// This defines the floating point precision of the show program
typedef float sfloat;

#include "show/show1.icc"
#include "show/show.icc"

void CallBackReshapeFunc(int width, int height);
void CallBackIdleFunc(void);
void DisplayItFunc(GLenum mode, bool interruptable = false);
void DrawPoints(GLenum mode, bool interruptable = false);
void DrawUrl();
void glDumpWindowPPM(const char *filename, GLenum mode);
void glWriteImagePPM(const char *filename, int scale, GLenum mode);
void ProcessHitsFunc(GLint hits, GLuint buffer[]);
int parseArgs(int argc, char **argv, string &dir, int& start, int& end, int& maxDist, bool& wanim, bool &readInitial);
void usage(char * prog);
void myNewMenu();
void topView();
void resetView(int dummy);
void setView(double pos[3], double new_quat[4], 
             double newMouseRotX, double newMouseRotY, double newMouseRotZ,
             double newCangle,
             bool sTV, bool cNMM, double pzoom_new, 
             bool s_points, bool s_path, bool s_cameras, double ps, int
             sf, double fD, bool invert);
void update_view_rotate(int);
void update_view_translation(int);
void startAnimation(int dummy);
void invertView(int dummy);
void callTopView(int dummy);
void drawCameras(void);
void callAddCamera(int dummy);
void callCameraUpdate(int dummy);
void callDeleteCamera(int dummy);

void pathAnimate(int dummy);
void savePath(int dummy);
void loadPath(int dummy);
void savePose(int dummy);
void loadPose(int dummy);
void saveImage(int dummy);
void CallBackInterfaceFunc(unsigned char key, int x, int y);
void CallBackKeyboardFunc(unsigned char key, int x, int y);
void CallBackKeyboardUpFunc(unsigned char key, int x, int y);
void CallBackMouseFunc(int button, int state, int x, int y);
void CallBackMouseFuncMoving(int button, int state, int x, int y);
void CallBackMouseMotionFunc(int x, int y);
void CallBackSpecialFunc(int key, int x, int y);
void InterfaceFunc(unsigned char key);
void updateCamera();
void drawRobotPath(int dummy);
int calcFrameNo();
int calcNoOfPoints(vector<PointXY>, vector<PointXY>);
void calcInterpolatedCameras(vector<PointXY>, vector<PointXY>);
void calcPointSequence(vector<int> &sequence, int frameNr);

void createDisplayLists(bool reduced=false);

void updatePointModeControls();
void changePointMode(int dummy);
void mapColorToValue(int dummy);
void changeColorMap(int dummy);
void minmaxChanged(int dummy);
void resetMinMax(int dummy);
void setScansColored(int dummy);

void saveSelection(int dummy);
void clearSelection(int dummy);

void updateControls();
void updateTopViewControls();
void resetRotationButton();
void updateCamControls();
bool isInterrupted();
void checkForInterrupt();
void interruptDrawing();
void cycleLOD();

enum { ROTATION_X, ROTATION_RX, ROTATION_Y, ROTATION_RY, ROTATION_Z, ROTATION_RZ };
  /** enumeration for translation */
enum { TRANS_X, TRANS_RX, TRANS_Y, TRANS_RY, TRANS_Z, TRANS_RZ };
  /** enumeration for the menue */
enum { MENU_SCREEN, MENU_LIST, MENU_QUIT };

#endif

