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
#include <iostream>

extern int nr_interpolations;
extern int imageNr;

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
 * Suggest a filename (for use with saveImageAt).
 */
std::string suggestImageFileName();

/**
 * This function saves the current view into a ppm-file with variable scale
 * factor. Callback fora button in GLUIshow.
 */
void saveImage(int dummy);

/**
 * Save the current view into the given PPM file with a scaled size from the global `factor`.
 * Currently used by QtShow.
 */
void saveImageAt(const std::string& imageFileName);

/**
 * Saves the currently selected points
 *
 */
void saveSelection(int dummy);
