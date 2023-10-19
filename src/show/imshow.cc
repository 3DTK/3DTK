/*
 * imshow implementation
 *
 * Copyright (C) Fabian Arzberger.
 *
 * Released under the GPL version 3.
 *
 */

/**
 * @file
 * @brief This is show, but without GLUI. Instead, ImGui is used.
 * @author Fabian Arzberger. Institute of CS, University of Wuerzburg, Germany.
 *
 * Original authors of show.cc and show_gl.cc:
 * @author Andreas Nuechter. Institute of CS, University of Osnabrueck, Germany.
 * @author Kai Lingemann. Institute of CS, University of Osnabrueck, Germany.
 * @author Jan Elseberg. Jacobs University bremen gGmbH, Germany.
 * @author Dorit Borrmann. Jacobs University bremen gGmbH, Germany.
 */

#include "imgui.h"
#include "imgui_impl_glut.h"
#include "imgui_impl_opengl2.h"
#define GL_SILENCE_DEPRECATION

#include "show/show_common.h"
#include "show/program_options.h"
#include "show/show_gl.h"
#include "show/callbacks_glut.h"

#include <thread>
#ifdef __linux__
	#include <linux/input.h>
#endif
#ifndef __APPLE__
#ifdef SPACEMOUSE
    #include "show/show_gl.h"
    #include <sys/types.h>
    #include <sys/stat.h>
    #include <fcntl.h>
    #include <stdio.h>
    #include <math.h>
    #include <stdlib.h>
    #include <stdint.h>
    #include <thread>
    #include <linux/input.h>
    #include <spnav.h>

    #define SPNAV_DEAD_THRESH_R 50 // Adjust to your liking
    #define SPNAV_DEAD_THRESH_T 30 // Adjust to your liking
#endif

static bool interrupted = false;

// GLUT Interrupt handlers:

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

// Spacemouse handler:
int spacenavHandler(){
#ifdef SPACEMOUSE
    int fixRoation = 0;
    int fixTranslation = 0;
    float translationMultiplier = 0.05; // adjust to your liking
    float rotationMultipler = 0.005;
    spnav_event sev;
    if(spnav_open()==-1) {
        fprintf(stderr, "failed to connect to the space navigator daemon\n");
        return 1;
    }
    //spnav_sensitivity(0.03); // Saturates any input on my spacemouse... use multipliers above instead - Fabi
    while(true){
        int event = spnav_poll_event(&sev);
        if(event != 0){
          // Use for Debug:
          // printf("got motion event: t(%d, %d, %d) ", sev.motion.x, sev.motion.y, sev.motion.z);
          // printf("r(%d, %d, %d)\n", sev.motion.rx, sev.motion.ry, sev.motion.rz);
            if(sev.type == SPNAV_EVENT_MOTION) {

                // Set rotation deadzone
                if (fabs(sev.motion.rx) < SPNAV_DEAD_THRESH_R)
                  sev.motion.rx = 0;
                if (fabs(sev.motion.ry) < SPNAV_DEAD_THRESH_R)
                  sev.motion.ry = 0;
                if (fabs(sev.motion.rz) < SPNAV_DEAD_THRESH_R)
                  sev.motion.rz = 0;
                if (fabs(sev.motion.x) < SPNAV_DEAD_THRESH_T)
                  sev.motion.x = 0;
                if (fabs(sev.motion.y) < SPNAV_DEAD_THRESH_T)
                  sev.motion.y = 0;
                if (fabs(sev.motion.z) < SPNAV_DEAD_THRESH_T)
                  sev.motion.z = 0;

                // Handle the event
                if(fixRoation){
                    moveCamera(-sev.motion.x*translationMultiplier, -sev.motion.y*translationMultiplier, sev.motion.z*translationMultiplier, 0, 0, 0, true);
                }
                else if(fixTranslation){
                    moveCamera(0, 0, 0, -0.5*sev.motion.rx*rotationMultipler, 0.5*sev.motion.ry*rotationMultipler, -0.5*sev.motion.rz*rotationMultipler, true);
                }else
                    moveCamera(-sev.motion.x*translationMultiplier, -sev.motion.y*translationMultiplier, sev.motion.z*translationMultiplier, -rotationMultipler*sev.motion.rx, rotationMultipler*sev.motion.ry, -rotationMultipler*sev.motion.rz, true);
            } else {	/* SPNAV_EVENT_BUTTON */
                if(sev.button.press && sev.button.bnum == 0){
                    fixRoation = !fixRoation;
                }
                if(sev.button.press && sev.button.bnum == 1){
                    fixTranslation = !fixTranslation;
                }
            }
        }
    }
#else
#ifdef __linux__
	struct input_id device_info;
	int fd = open("/dev/input/by-id/usb-3Dconnexion_SpaceMouse_Compact-event-if00", O_RDONLY);
	if (fd == -1) {
		fprintf(stderr, "3D Mouse not connected\n");
		return 1;
	}
	if(ioctl(fd, EVIOCGID, &device_info)) {
		perror("3D Mouse EVIOCGID ioctl failed");
		return 1;
	}
	if (device_info.vendor != 0x256f) {
		fprintf(stderr, "unexpected vendor: %d\n", device_info.vendor);
		return 1;
	}
	if (device_info.product != 0xc635) {
		fprintf(stderr, "unexpected product: %d\n", device_info.product);
		return 1;
	}
	if (device_info.bustype != BUS_USB) {
		fprintf(stderr, "unexpected bus type: %d\n", device_info.bustype);
		return 1;
	}
	int axes[6] = {0,0,0,0,0,0};
	int buttons[2] = {0, 0};
	struct input_event ev;
	uint8_t evtype_bitmask[EV_MAX/8 + 1];
	int ev_type;

	memset(evtype_bitmask, 0, sizeof(evtype_bitmask));
	if (ioctl(fd, EVIOCGBIT(0, sizeof(evtype_bitmask)), evtype_bitmask) < 0) {
		perror("3D Mouse EVIOCGBIT ioctl failed");
		return 1;
	}
	for (;;) {
		int n = read(fd, &ev, sizeof(struct input_event));
		if(n < sizeof(struct input_event)) {
			fprintf(stderr, "unexpected event size\n");
			return 1;
		}
		switch (ev.type) {
			case EV_KEY:
				if (ev.code != 0 and ev.code != 1) {
					fprintf(stderr, "unexpected button code: %d\n", ev.code);
				}
				buttons[ev.code] = ev.value;
				break;
			case EV_REL:
				switch(ev.code) {
					case 0: moveCamera(-ev.value /4, 0, 0, 0, 0, 0); break;
					case 1: moveCamera(0, 0, -ev.value /4, 0, 0, 0); break;
					case 2: moveCamera(0, ev.value /4, 0, 0, 0, 0); break;
					case 3: moveCamera(0, 0, 0, -ev.value/40, 0, 0); break;
					case 4: moveCamera(0, 0, 0, 0, 0, ev.value/40); break;
					case 5: moveCamera(0, 0, 0, 0, -ev.value/40, 0); break;
				}
		}
	}
#endif
#endif
	return 0;
}
#endif

// Screenshot handler
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

// TODO: Updating the controlls, which are to be implemented

void updateCamControls() {
  // TODO: implement cam controls first
}

void resetRotationButton() {
  // TODO: implement reset-rotation button first
}

void updateViewModeControls() {
  // TODO: implement view mode controlls first
}

void updateControls() {
  // TODO: implement controlls first
}

void updatePointModeControls() {
  // TODO: implement point mode controlls first
}

// GLUT Function wrappers:

/* Wrapper for mouse button handling. Uses both the internal callback and imgui*/
void mouseButtonIm(int button, int state, int x, int y) {
  ImGui_ImplGLUT_MouseFunc(button, state, x, y);
  callbacks::glut::mouseButton(button, state, x, y);
}

/* Wrapper for reshape handling. Uses both the internal callback and imgui*/
void reshapeIm(int width, int height) {
  ImGui_ImplGLUT_ReshapeFunc(width, height);
  callbacks::glut::reshape(width, height);
}

/* Wrapper for mouse movement. Uses both the internal callback and imgui*/
void mouseMoveIm(int x, int y) {
  ImGui_ImplGLUT_MotionFunc(x, y);
  ImGuiIO& io = ImGui::GetIO();
  if (!io.WantCaptureMouse)
    callbacks::glut::mouseMove(x, y);
}

/* Modified display loop function for GLUT, uses common show funcs. and imgui
displayIm alters the OpenGL rendering cycle from the original 3dtk to include ImGui.*/
void displayIm() {

  // ImGUI Renderings First:
  ImGui_ImplOpenGL2_NewFrame();
  ImGui_ImplGLUT_NewFrame();
  ImGui::NewFrame();
  ImGuiIO& io = ImGui::GetIO();

  // TODO: This is a dummy window with arbitrary conent.
  // TODO: Replace with a functioning Controlls panel.
  // 1. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
  {
    ImGui::Begin("3D Viewer - Selection");
    ImGui::SetNextWindowPos(ImVec2(START_WIDTH * 0.10, START_HEIGHT * 0.30), ImGuiCond_FirstUseEver);

    // Checkboxes
    static bool show_points_bool = true, show_objects_bool = true, show_cameras_bool = true, show_path_bool = true, show_poses_bool = true;
    ImGui::Checkbox("Draw Points", &show_points_bool); show_points = show_points_bool;
    ImGui::Checkbox("Draw Objects", &show_objects_bool); show_objects = show_objects_bool;
    ImGui::Checkbox("Draw Camera", &show_cameras_bool); show_cameras = show_cameras_bool;
    ImGui::Checkbox("Draw Path", &show_path_bool); show_path = show_path_bool;
    ImGui::Checkbox("Draw Poses", &show_poses_bool); show_poses = show_poses_bool;

    // Point size
    ImGui::SliderFloat("Point size", &pointsize, 0.0000001f, 10.0f);            

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::End();
  }

  // TODO: This is a dummy window with arbitrary conent.
  // TODO: Replace with a functioning Selections panel.
  // 2. Show another simple window.
  {
    ImGui::Begin("3D Viewer - Controls");   // Pass a pointer to our bool variable (the window will have a closing button that will clear the bool when clicked)
    ImGui::Text("Hello from another window!");
    ImGui::End();
  }
  // Render into buffer
  ImGui::Render();

  // GLUT / OpenGL2 camera and aspect handling:
  if ((cangle_spinner != 0 && (fabs(cangle_old - cangle) > 0.5)) ||
    (pzoom_spinner != 0 && (fabs(pzoom_old - pzoom) > 0.5))) {
    cangle_old = cangle;
    pzoom_old = pzoom;
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    reshapeIm(viewport[2], viewport[3]);
#ifdef _WIN32
    Sleep(25);
#else
    usleep(250000);
#endif
  }

  // After reshaping, clear everything on screen
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  // Filling the buffer with OpenGL point cloud renderings
  DisplayItFunc(GL_RENDER);
  // Draw the buffer
  glDrawBuffer(buffermode);
  // Finally Draw ImGui contents as well
  ImGui_ImplOpenGL2_RenderDrawData(ImGui::GetDrawData());

  // show the rendered scene
  glutSwapBuffers();
  glutPostRedisplay();
}


/* Modified initScreenWindow function.
** This function installs the backend functions for GLUT.
** Some of them are modified wrappers to include ImGui.
*/
void initScreenWindowIm()
{
  // init display
  glutInitDisplayMode(GLUT_DEPTH | GLUT_RGBA | GLUT_DOUBLE);

  // define the window position and size
  glutInitWindowPosition(START_X, START_Y);
  glutInitWindowSize( START_WIDTH, START_HEIGHT );

  // create window
  window_id = glutCreateWindow("3D_Viewer");
  // Set the loop function for GLUT
  glutDisplayFunc(displayIm);

  // Initialize ImGui, get context and backend
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  ImGui_ImplGLUT_Init();
  ImGui_ImplOpenGL2_Init();

  // Disable imgui.ini file which messes with initial positions sometimes
  ImGui::GetIO().IniFilename = nullptr;

  // Install GLUT functions
  // ImGui wraps the original 3dtk GLUT handlers:
  glutReshapeFunc(reshapeIm);
  glutMouseFunc(mouseButtonIm);
  glutMotionFunc(mouseMoveIm);

  // Keyboard and Idle are not handled by 3dtk.
  // Original 3dtk installed GLUT handler functions:
  glutKeyboardFunc(callbacks::glut::keyPressed);
  glutKeyboardUpFunc(callbacks::glut::keyReleased);
  glutIdleFunc(callbacks::glut::idle);
#ifdef __APPLE__
  glutWMCloseFunc(quit); // apple works different
#else
  glutCloseFunc(callbacks::glut::quit);
#endif

  // Setup view and 3DTK logo
  update_view_rotate(0);
  glClearColor(0.0, 0.0, 0.0, 0.0);
  load_url_texture();
}

/**
 * Main function.
 * Reads the scan (scan000.3d, ...) and frames files (scan000.frames, ...)
 * from the data directory.
 * The frames are used for animation of the matching process.
 */
int main(int argc, char **argv)
{
  // Signal handling and GLUT init
  setSignalHandling();
  glutInit(&argc, argv);

  // Program options
  dataset_settings dss;
  window_settings ws;
  display_settings ds;
  try {
    parse_args(argc, argv, dss, ws, ds);
  } catch (std::exception& e) {
    std::cerr << "Error while parsing settings: " << e.what() << std::endl;
    exit(1);
  }

  // Connect backend and init show window
  initScreenWindowIm();
  initShow(dss, ws, ds);

  // TODO: screenshot handling should disable ImGui content
  if (takescreenshot) {
    glutTimerFunc(0, &saveImageAndExit, 0);
  }
#ifndef __APPLE__
    std::thread t1(spacenavHandler);
#endif

  // Starting the Glut Main Loop
  glutMainLoop();
}
