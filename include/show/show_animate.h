/*
 * show_animate implementation
 *
 * Copyright (C) Kai Lingemann, Andreas Nuechter, Jan Elseberg, Dorit Borrmann
 *
 * Released under the GPL version 3.
 *
 */

#include "show/show_common.h"

#include <fstream>
using std::fstream;
using std::ofstream;
using std::ios;
#include <iostream>
using std::cout;
using std::cerr;
using std::endl;

extern int nr_interpolations;

void calcUpPath();

void calcLookAtPath();

void calcPath();

void updateCamera();


//-----------------------------------------------------------------------------

/**
 * This function saves the path drawn on the
 * screen to a file.
 */
void savePath(int dummy);

 
//----------------------------------------------------------------------------

/**
 * This function loads the path from the given
 * path file.
 */
void loadPath(int dummy);

/**
 * This function loads the camera pose from the given
 * pose file.
 */

void loadPose(int dummy);

/**
 * This function saves the current camera pose to a file.
 */

void savePose(int dummy);

/**
  * This function saves the current view into a ppm-file with variable scale
  * factor.
  */
void saveImage(int dummy);

/**
 * Save the current view into the given PPM file with a scaled size from the global `factor`.
 */
void saveImageAt(const std::string& imageFileName);

/**
 * Saves the currently selected points
 *
 */
void saveSelection(int dummy);
