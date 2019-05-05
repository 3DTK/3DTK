#include "show/callbacks_glut.h"
#include "show/show_gl.h"

namespace callbacks {
namespace glut {

void display() {
  if ((cangle_spinner != 0 && (fabs(cangle_old - cangle) > 0.5)) ||
      (pzoom_spinner != 0 && (fabs(pzoom_old - pzoom) > 0.5))) {

    cangle_old = cangle;
    pzoom_old = pzoom;
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    reshape(viewport[2], viewport[3]);
#ifdef _WIN32
    Sleep(25);
#else
    usleep(250000);
#endif
  }

  glDrawBuffer(buffermode);
  // delete framebuffer and z-buffer

  // Call the display function
  DisplayItFunc(GL_RENDER);

  // show the rednered scene
  glutSwapBuffers();

}

void idle(void) {

#ifdef _WIN32
  Sleep(1);
#else
  usleep(1000);
#endif

  if (glutGetWindow() != window_id)
    glutSetWindow(window_id);

  // return as nothing has to be updated
  if (haveToUpdate == 0) {
    if (!fullydisplayed && !mousemoving && !keypressed && pointmode == 0
      ) {
      glDrawBuffer(buffermode);
      //Call the display function
      DisplayItFunc(GL_RENDER, true);
    }
    return;
  }

  // case: display is invalid - update it
  if (haveToUpdate == 1) {
    update_callback();
    haveToUpdate = 0;
    return;
  }
  // case: display is invalid - update it with all points
  /*  if (haveToUpdate == 7) {
    showall = true;
    update_callback();
    haveToUpdate = 0;
    return;
  }*/

  // case: camera angle is changed - instead of repeating code call Reshape,
  // since these OpenGL commands are the same
  if (haveToUpdate == 2) {
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    reshape(viewport[2], viewport[3]);
    update_callback();
    haveToUpdate = 0;
    return;
  }

  // case: animation
  if (haveToUpdate == 3) {
    frameNr += 1;
    if (!(MetaMatrix.size() > 1 && frameNr < (int) MetaMatrix[1].size())) {
      frameNr = 0;
      haveToUpdate = 4;
      return;
    }
    update_callback();

    if (save_animation) {
      std::string filename = scan_dir + "animframe" + to_string(frameNr, 5) + ".png";
      std::cout << "write " << filename << std::endl;
      int tmpUpdate = haveToUpdate;
      glWriteImagePNG(filename.c_str(), factor, 0);
      haveToUpdate = tmpUpdate;
    }
  }

#ifdef _WIN32
  // Extra Sleep is NOT necessary!
  //Sleep(300);
  Sleep(anim_delay);
#else
  usleep(anim_delay * 10000);
#endif

  if (haveToUpdate == 4) { // stop animation
    frameNr = 0;  // delete these lines if you want a 'continue' functionality.
    haveToUpdate = 1;
  }

  // case: scan matching and path animation in lock-step
  if (haveToUpdate == 8) {
    if (path_iterator == 0) {
      oldcamNavMode = cameraNavMouseMode;  // remember state of old mousenav
      cameraNavMouseMode = 0;
    }
    frameNr += 1;
    if (!(MetaMatrix.size() > 1 && frameNr < (int) MetaMatrix[1].size())) {
      frameNr = 0;
      haveToUpdate = 4;
      return;
    }
    std::cout << path_iterator << " " << ups.size() << std::endl;
    if ((false && path_iterator < path_vectorX.size()) ||
        (true && path_iterator < ups.size())) {   // standard animation case

      // call the path animation function
      // hide both the cameras and the path
      show_cameras = 0;
      show_path = 0;
      show_poses = 0;
      // increase the iteration count

      path_iterator += 1;
      // repaint the screen
      update_callback();

      // save the animation
      if (save_animation) {
        std::string filename = scan_dir + "animframe"
                                        + to_string(path_iterator, 5) + ".png";
        std::cout << "written " << filename << " of "
                  << path_vectorX.size() << " files" << std::endl;
        glWriteImagePNG(filename.c_str(), factor, 0);
        haveToUpdate = 8;
      }
    } else {                             // animation has just ended
      cameraNavMouseMode = oldcamNavMode;
      show_cameras = 1;
      show_path = 1;
      show_poses = 1;
      haveToUpdate = 0;
    }
  }

  // case: path animation
  if (haveToUpdate == 6) {

    if (path_iterator == 0) {
      oldcamNavMode = cameraNavMouseMode;  // remember state of old mousenav
      cameraNavMouseMode = 0;
    }

    // check if the user wants to animate both
    // scan matching and the path at the same
    // time

    // cout << "path_iterator: " << path_iterator << endl;
    if (path_iterator < path_vectorX.size()) {   // standard animation case

      // call the path animation function
      // hide both the cameras and the path
      show_cameras = 0;
      show_path = 0;
      // increase the iteration count

      path_iterator += 1;
      // repaint the screen
      update_callback();

      // save the animation
      if (save_animation) {
        std::string filename = scan_dir + "animframe"
                                        + to_string(path_iterator, 5) + ".png";

        std::cout << "written " << filename << " of "
                  << path_vectorX.size() << " files" << std::endl;
        glWriteImagePNG(filename.c_str(), factor, 0);
        haveToUpdate = 6;
      }
    } else {                             // animation has just ended
      cameraNavMouseMode = oldcamNavMode;
      show_cameras = 1;
      show_path = 1;
      haveToUpdate = 0;
    }
  }
}

void CallBackMouseFuncMoving(int button, int state, int x, int y) {

  if (state == GLUT_DOWN) {
    mousemoving = true;
  } else {
    mousemoving = false;
  }
}

void mouseButton(int button, int state, int x, int y) {
  // Are we selecting points or moving the camera?
  if (cameraNavMouseMode != 1) { // selecting points
    if (state == GLUT_DOWN &&
        (button == GLUT_LEFT_BUTTON || button == GLUT_RIGHT_BUTTON)) {
      selectPoints(x, y);
    }
  } else {
    if (state == GLUT_DOWN) {

      mouseNavX = x;
      mouseNavY = y;
      mouseNavButton = button;

      mousemoving = true;
    } else {
      mouseNavButton = -1;
      mousemoving = false;
    }
  }

  if(state == GLUT_DOWN) {
    mousePresX = x;
    mousePresY = y;
  }
  else if(state == GLUT_UP && showViewMode == 2) {
    for(int i = 0; i < 4; i++)
      now_quat[i] = tmp_quat[i];
    mouseNavX = mousePresX;
    mouseNavY = mousePresY;
  }
}


void quit() {
  // this gets called when window is closed without using the "Quit" button - ensure regular shutdown to avoid crash
  deinitShow();
}

void mouseMove(int x, int y) {
  double deltaMouseX = x - mouseNavX;
  double deltaMouseY = mouseNavY - y;

  // Save last position
  mouseNavX = x;
  mouseNavY = y;

  if (cameraNavMouseMode == 1) {
    mouseMoveDelta(deltaMouseX, deltaMouseY);
  } else {
    selectPoints(x, y);
  }
}

void mouseMoveDelta(double deltaMouseX, double deltaMouseY) {
  if (invertMouseX) deltaMouseX = -deltaMouseX;
  if (invertMouseY) deltaMouseY = -deltaMouseY;

  if (mouseNavButton == GLUT_RIGHT_BUTTON) {
    if (showViewMode == 1) {
      deltaMouseX *= 5;
      deltaMouseY *= 5;
    }
    // moving 10 pixels is equivalent to one key stroke
    deltaMouseX *= movementSpeed / 10.0;
    deltaMouseY *= movementSpeed / 10.0;
    moveCamera(deltaMouseX, deltaMouseY, 0, 0, 0, 0);
  } else if (mouseNavButton == GLUT_MIDDLE_BUTTON) {
    if (showViewMode != 1) {
      deltaMouseY *= -5;
    }
    // moving 10 pixels is equivalent to one key stroke
    deltaMouseX *= movementSpeed / 10.0;
    deltaMouseY *= movementSpeed / 10.0;
    moveCamera(deltaMouseX, 0, deltaMouseY, 0, 0, 0);
  } else if (mouseNavButton == GLUT_LEFT_BUTTON) {
    moveCamera(0, 0, 0, deltaMouseY, deltaMouseX, 0);
  } else {
    return;
  }
}

void reshape(int width, int height) {
  if (!fullscreen) {
    current_height = height;
    current_width = width;
  }
  aspect = (double) width / (double) height;

  if (showViewMode != 1) {
    // usage of the vsize of a structiewport
    glViewport(0, 0, (GLint) width, (GLint) height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // angle, aspect, near clip, far clip
    // get matrix
    gluPerspective(cangle, aspect, neardistance, fardistance);

    // now use modelview-matrix as current matrix
    glMatrixMode(GL_MODELVIEW);

    haveToUpdate = 1;

  } else {

    // usage of the viewport
    glViewport(0, 0, width, height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    // get matrix
    glOrtho(-aspect * pzoom, aspect * pzoom,
            -1 * pzoom, pzoom,
            1.0, 32000.0);

    // now use modelview-matrix as current matrix
    glMatrixMode(GL_MODELVIEW);

    haveToUpdate = 1;

  }

  //  glDepthMask(false);
  glEnable(GL_BLEND); // TODO
  glBlendFunc(GL_ONE, GL_ZERO); // TODO
  // glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); // TODO
  // glBlendFunc(GL_ONE, GL_ONE); // TODO
  // glBlendFunc(GL_SRC_COLOR, GL_DST_COLOR);
  glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);
  // TODO glDepthFunc(GL_LEQUAL);
  glDepthFunc(GL_LESS); //TODO
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_POINT_SMOOTH);
}

void keyReleased(unsigned char key, int x, int y) {
  keymap[key] = false;
  if (key >= 'A' && key <= 'Z') {
    keymap[key + ('a' - 'A')] = false;
  }
  if (key >= 'a' && key <= 'z') {
    keymap[key + ('A' - 'a')] = false;
  }

  for (unsigned int i = 0; i < 256; i++) {
    if (keymap[i]) {
      keypressed = true;
      return;
    }
  }
  keypressed = false;
}

void keyPressed(unsigned char key, int x, int y) {
  keymap[key] = true;
  keypressed = true;
  bool ctrl, alt, shift;
  ctrl = glutGetModifiers() & GLUT_ACTIVE_CTRL;
  alt = glutGetModifiers() & GLUT_ACTIVE_ALT;
  shift = glutGetModifiers() & GLUT_ACTIVE_SHIFT;
  if (ctrl) {
    key += 96;
  }
  keyHandler(key, ctrl, alt, shift);
}

void keyHandler(int key, bool control, bool alt, bool shift) {
  double stepsize = movementSpeed;
  if (shift) stepsize *= 10.0;
  if (control) stepsize *= 0.1;
  if (alt) stepsize *= 100.0;

  double rotsize = 0.2 * stepsize;

  switch (key) {
    case 'w':
    case 'W':
      moveCamera(0,0,stepsize,0,0,0);
      break;
    case 'a':
    case 'A':
      moveCamera(stepsize,0,0,0,0,0);
      break;
    case 's':
    case 'S':
      moveCamera(0,0,-stepsize,0,0,0);
      break;
    case 'd':
    case 'D':
      moveCamera(-stepsize,0,0,0,0,0);
      break;
    case 'c':
    case 'C':
      moveCamera(0,stepsize,0,0,0,0);
      break;
    case 32:  // WXK_SPACE
      moveCamera(0,-stepsize,0,0,0,0);
      break;
    case 314: // WXK_LEFT
      moveCamera(0,0,0,0,rotsize,0);
      break;
    case 315: // WXK_UP
      moveCamera(0,0,0,rotsize,0,0);
      break;
    case 316: // WXK_RIGHT
      moveCamera(0,0,0,0,-rotsize,0);
      break;
    case 317: // WXK_DOWN
      moveCamera(0,0,0,-rotsize,0,0);
      break;
    case 'q':
    case 'Q':
    case 366: // WXK_PAGEUP
      moveCamera(0,0,0,0,0,rotsize);
      break;
    case 'e':
    case 'E':
    case 367: // WXK_PAGEDOWN
      moveCamera(0,0,0,0,0,-rotsize);
      break;
    case 'f':
      if (!fullscreen) {
        fullscreen = true;
        glutFullScreen();
      } else {
        fullscreen = false;
        glutReshapeWindow(current_width, current_height);
      }
      break;
    default:
      break;
  }
}

}
}
