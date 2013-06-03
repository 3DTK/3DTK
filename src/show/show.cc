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

#include "show_common.cc"

#include <csignal>

void signal_segv(int v)
{
  static bool segfault = false;
  if(!segfault) {
    segfault = true;
    cout << endl << "Segmentation fault" << endl;
    deinitShow();
  }
  exit(-1);
}

void signal_interrupt(int v)
{
  static bool segfault = false;
  if(!segfault) {
    segfault = true;
    cout << endl << "Exiting by interrupt" << endl;
    deinitShow();
  }
  exit(-1);
}

/**
 * Main function.
 * Reads the scan (scan000.3d, ...) and frames files (scan000.frames, ...)
 * from the data directory.
 * The frames are used for animation of the matching process.
 */
int main(int argc, char **argv)
{
  signal(SIGSEGV, signal_segv);
  signal(SIGINT,  signal_interrupt);
  signal(SIGTERM, signal_interrupt);

  initShow(argc, argv);
  initScreenWindow();

  newMenu();
  glutMainLoop();
}


void updateCamControls() {
  cam_spinner->set_int_limits( 1, cams.size());
  cam_spinner->set_int_val(cam_choice);
}

void resetRotationButton() {
  rotButton->reset();
}

void updateTopViewControls() {
  if(showTopView) {
    pzoom_spinner->enable();
    cangle_spinner->disable();
  } else {
    pzoom_spinner->disable();
    cangle_spinner->enable();
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
#ifdef WITH_FREEGLUT
#ifndef __APPLE__
  glutMainLoopEvent(); 
#endif
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
