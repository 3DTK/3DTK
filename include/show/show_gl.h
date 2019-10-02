/*
 * show_gl implementation
 *
 * Copyright (C) Kai Lingemann, Andreas Nuechter, Jan Elseberg, Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#ifndef __SHOW_GL_H__
#define __SHOW_GL_H__


#include <cstring>
#include <cstdlib>

#include "show/scancolormanager.h"
#include "show/show_menu.h"
#include "show/viewcull.h"

using namespace show;
extern bool   fullydisplayed;       // true if all points have been drawn to
                                    // the screen
extern bool   mousemoving;          // true if a mouse button has been pressed
                                    // inside a window,
                                    // but has not been released
extern bool   keypressed;           // true if a key button has been pressed
                                    // inside a window,
                                    // but has not been released
extern double ptstodisplay;
extern double lastfps;              // last frame rate
extern int    pointmode;

extern bool   smallfont;
extern bool   label;

/**
 * Displays all data (i.e., points) that are to be displayed
 * @param mode specification for drawing to screen or in selection mode
 */
void DrawPoints(GLenum mode, bool interruptable);


void DrawObjects(GLenum mode);

/**
 * Draw a smooth path passing from all the camera points.
 *
 */
void DrawPath();

/**
 * Draw the camera boxes in the viewer
 *
 */
void DrawCameras(void);

/**
 * Draw the coordinate system of each scan
 *
 */
void DrawCoordinateSystems(void);

void DrawScala();


//-----------------------------------------------------------------------------------


/**
 * Display function
 */
void DisplayItFunc(GLenum mode, bool interruptable);

void DrawUrl();

/**
 * Function topview. Set the screen for top view.
 */
void topView();

/**
 * Function rotateview. Set the screen for rotate view.
 */
void rotateView();

//---------------------------------------------------------------------------
/**
 * This function is called when the user wants to
 * delete a camera.
 */
void callDeleteCamera(int dummy);


//---------------------------------------------------------------------------
/**
 * Function to reset the viewer window.
 */

void resetView(int dummy);

/**
 * Function to set the viewer window back to a previously saved state.
 */
void setView(double pos[3], double new_quat[4],
             double newMouseRotX, double newMouseRotY, double newMouseRotZ,
             double newCangle,
             int sVM, bool cNMM, double pzoom_new,
             bool s_points, bool s_path, bool s_cameras, double ps, int
             sf, double fD, bool inv);


/**
 * This function handles the rotation of the view
 */
void update_view_rotate(int t);

/**
 * This function handles the translation of view.
 */
void update_view_translation(int t);


/**
 * handles the animation button
 * @param dummy not needed necessary for glui
 */
void startAnimation(int dummy);

/**
 * calls the resetView function
 * @param dummy not needed necessary for glui
 */
void callResetView(int dummy);

/**
 * calls the resetView function
 * @param dummy not needed necessary for glui
 */
void invertView(int dummy);

/**
 * calls the topView function
 * @param dummy not needed necessary for glui
 */
void callTopView(int dummy);

/**
 * calls the rotateView function
 * @param dummy not needed necessary for glui
 */
void callRotateView(int dummy);

/**
 * calls the cameraView function
 * @param dummy not needed necessary for glui
 */
void callAddCamera(int dummy);

void selectPoints(int x, int y);


void moveCamera(double x, double y, double z,
                double rotx, double roty, double rotz);

void initScreenWindow();

void load_url_texture();

/* +++++++++-------------++++++++++++
 * NAME
 *   glDumpWindowPPM
 * DESCRIPTION
 *   writes an ppm file of the window
 *   content
 * PARAMETERS
 *   filename
 * RESULT
 *  writes the framebuffer content
 *  to a ppm file
+++++++++-------------++++++++++++ */
void glDumpWindowPPM(const char *filename, GLenum mode);

/* +++++++++-------------++++++++++++
 * NAME
 *   glWriteImagePNG
 * DESCRIPTION
 *   writes a png file of the window
 *   content
 *   size is scale times the window size
 * PARAMETERS
 *   filename
 * RESULT
 *  writes the framebuffer content
 *  to a png file
+++++++++-------------++++++++++++ */
void glWriteImagePNG(const char *filename, int scale, GLenum mode);

/**
 *  Prints out which points were clicked on
 */
void ProcessHitsFunc(GLint hits, GLuint buffer[]);


//------------------------------------------------------------------
/**
 * This function deals with all our keyboard activities
 */

void InterfaceFunc(unsigned char key);


/**
 * Function drawRobotPath
 * \brief This functions draws the path where the
 * robot has travelled along while taking the scans
 */
void drawRobotPath(int dummy);

/**
  * Calculates the positions of the interpolated camera path positions on the
  * Nurbs path. There will be an equal number of intermediate positions between
  * neighboring cameras.
  */
void calcInterpolatedCameras(std::vector<PointXY> vec1, std::vector<PointXY> vec2);

/**
  * Calculates the number of interpolation points for the camera path based on
  * the length of the path
  */
int calcNoOfPoints(std::vector<PointXY> vec1, std::vector<PointXY> vec2);

void mapColorToValue(int dummy);

void changeColorMap(int dummy);

void minmaxChanged(int dummy);

void resetMinMax(int dummy);

void setScansColored(int dummy);

void changePointMode(int dummy);

void callCameraUpdate(int dummy);

void calcPointSequence(std::vector<int> &sequence, int frameNr);

#endif
