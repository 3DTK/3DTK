/**
 * @file
 * @brief Implementation for displaying of a matched 3D scene
 * @author Andreas Nuechter. Institute of Computer Science, University of Osnabrueck, Germany.
 * @author Amuz T, Jacobs University Bremen, German
 * @author Kai Lingemann. Institute of Computer Science, University of Osnabrueck, Germany.
 */

#include "show_common.cc"

/**
 * Main function.
 * Reads the scan (scan000.3d, ...) and frames files (scan000.frames, ...) from the data directory.
 * The frames are used for animation of the matching process.
 */
int main(int argc, char **argv){

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


void updateControls() {
  glui1->sync_live();
  glui1->show();
  glui2->sync_live();
  glui2->show();
}
