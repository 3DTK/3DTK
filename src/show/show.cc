/*
 * show_gl implementation
 *
 * Copyright (C) Kai Lingemann, Andreas Nuechter, Jan Elseberg, Dorit Borrmann.
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief Implementation for displaying of a matched 3D scene
 * @author Andreas Nuechter. Institute of CS, University of Osnabrueck, Germany.
 * @author Kai Lingemann. Institute of CS, University of Osnabrueck, Germany.
 * @author Jan Elseberg. Jacobs University bremen gGmbH, Germany.
 * @author Dorit Borrmann. Jacobs University bremen gGmbH, Germany.
 */

#include "show/show_common.h"
#include "show/program_options.h"

void saveImageAndExit(int dummy)
{
	// set pointmode to 1 to enforce rendering of all visible points
	pointmode = 1;
	if (screenshot_filename.empty()) {
		saveImage(0);
	} else {
		saveImageAt(screenshot_filename);
	}
	exit(0);
}

/**
 * Main function.
 * Reads the scan (scan000.3d, ...) and frames files (scan000.frames, ...)
 * from the data directory.
 * The frames are used for animation of the matching process.
 */
int main(int argc, char **argv)
{
  setSignalHandling();

  glutInit(&argc, argv);

  dataset_settings dss;
  window_settings ws;
  display_settings ds;

  try {
    parse_args(argc, argv, dss, ws, ds);
  } catch (std::exception& e) {
    std::cerr << "Error while parsing settings: " << e.what() << std::endl;
    exit(1);
  }

  initScreenWindow();
  initShow(dss, ws, ds);

  if (!nogui && !takescreenshot)
    newMenu();
  else if (takescreenshot) {
    glutTimerFunc(0, &saveImageAndExit, 0);
  }
  glutMainLoop();
}


void updateCamControls() {
  cam_spinner->set_int_limits( 1, cams.size());
  cam_spinner->set_int_val(cam_choice);
}

void resetRotationButton() {
  rotButton->reset();
}

void updateViewModeControls() {
  if(showViewMode == 0) {
    cangle_spinner->enable();
    pzoom_spinner->disable();
    rzoom_spinner->disable();
  }
  else if(showViewMode == 1) {
    cangle_spinner->disable();
    pzoom_spinner->enable();
    rzoom_spinner->disable();
  }
  else {
    cangle_spinner->disable();
    pzoom_spinner->disable();
    rzoom_spinner->enable();
  }
}


void updateControls() {
  glui1->sync_live();
  glui1->show();
  glui2->sync_live();
  glui2->show();
}

static bool interrupted = false;
void interruptDrawing() {
  interrupted = true;
}
void checkForInterrupt() {
  interrupted = false;
}
bool isInterrupted() {
#ifndef __APPLE__
  glutMainLoopEvent();
#endif
  glutSetWindow(window_id);
  return interrupted;
}

void updatePointModeControls() {
  switch(pointmode) {
    case -1:
      always_box->set_int_val(0);
      never_box->set_int_val(1);
      break;
    case 0:
      always_box->set_int_val(0);
      never_box->set_int_val(0);
      break;
    case 1:
      always_box->set_int_val(1);
      never_box->set_int_val(0);
      break;
  }
}
