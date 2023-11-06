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

#define IMGUI_DEFINE_MATH_OPERATORS
#include "imgui.h"
#include "imgui_impl_glut.h"
#include "imgui_impl_opengl2.h"
#define GL_SILENCE_DEPRECATION
#include "imGuIZMOquat.h"

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

static int SCREEN_WIDTH;
static int SCREEN_HEIGHT;
static int START_WIDTH_IMGUI;
static int START_HEIGHT_IMGUI;
static int START_X_IMGUI;
static int START_Y_IMGUI;

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

void keyButtonIm() {
  //TODO: addd keyboardd support for IMgUi
}

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
  io.ConfigWindowsMoveFromTitleBarOnly = true;
  
  // 1. Selection Window
  {
    ImGui::SetNextWindowPos(ImVec2(START_WIDTH_IMGUI * 0.85, START_HEIGHT_IMGUI * 0.05), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(START_WIDTH_IMGUI * 0.12, START_HEIGHT_IMGUI * 0.80), ImGuiCond_FirstUseEver);
    ImGui::Begin("Selection");

    if (ImGui::TreeNode("Draw")) {
      // Drawing Controlls
      static bool show_points_bool = true, show_objects_bool = true, show_cameras_bool = true, show_path_bool = true, show_poses_bool = true;
      // Checkboxes
      ImGui::Checkbox("Draw Points", &show_points_bool); show_points = show_points_bool;
      ImGui::Checkbox("Draw Objects", &show_objects_bool); show_objects = show_objects_bool;
      ImGui::Checkbox("Draw Camera", &show_cameras_bool); show_cameras = show_cameras_bool;
      ImGui::Checkbox("Draw Path", &show_path_bool); show_path = show_path_bool;
      ImGui::Checkbox("Draw Poses", &show_poses_bool); show_poses = show_poses_bool;
      ImGui::Separator();
      // Point size
      ImGui::Text("Point size:");
      ImGui::SliderFloat("##pixels", &pointsize, 0.1f, 10.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
      ImGui::Separator();

      //Scan range selection
      ImGui::Separator();
      ImGui::Text("Scan range selection:");
      ImGui::SliderInt("Id0", &startRangeScanIdx, startScanIdx, endRangeScanIdx);
      ImGui::SliderInt("Idx", &endRangeScanIdx, startRangeScanIdx, endScanIdx);
      if (ImGui::Button("Step Up")) stepScansUp(0);
      ImGui::SameLine();
      if (ImGui::Button("Step Down")) stepScansDown(0);
      if (ImGui::Button("Reload Frames")) reloadFrames();
      ImGui::TreePop();
    }
    ImGui::Separator();

    // Fog Controlls
    if (ImGui::TreeNode("Fog")) {
      ImGui::RadioButton("Disable", &show_fog, 0);
      ImGui::RadioButton("Exp", &show_fog, 1);
      ImGui::RadioButton("Exp2", &show_fog, 2);
      ImGui::RadioButton("Lin", &show_fog, 3);
      ImGui::RadioButton("Inv Exp", &show_fog, 4);
      ImGui::RadioButton("Inv Exp2 ", &show_fog, 5);
      ImGui::RadioButton("Inv Lin", &show_fog, 6);
      ImGui::Separator();
      ImGui::TreePop();
    }
    ImGui::Separator();
    ImGui::Text("Fog Density:");
    ImGui::SliderFloat("##fog", &fogDensity, 0.0f, 1.0f, "%.5f", ImGuiSliderFlags_Logarithmic);
    ImGui::Separator();

    // Color Controlls
    if (ImGui::TreeNode("Color")) {
      ImGui::Checkbox("Invert", &invert);

      // Color Values
      ImGui::Separator();
      if (ImGui::TreeNode("Values")) {
        ImGui::RadioButton("Height", &listboxColorVal, 0);
        if(!(pointtype.hasReflectance())) ImGui::RadioButton("Reflectance", false);
        else ImGui::RadioButton("Reflectance", &listboxColorVal, 1);
        if(!(pointtype.hasTemperature())) ImGui::RadioButton("Temperature", false);
        else ImGui::RadioButton("Temperature", &listboxColorVal, 2);
        if(!(pointtype.hasAmplitude())) ImGui::RadioButton("Amplitude", false);
        else ImGui::RadioButton("Amplitude", &listboxColorVal, 3);
        if(!(pointtype.hasDeviation())) ImGui::RadioButton("Deviation", false);
        else ImGui::RadioButton("Deviation", &listboxColorVal, 4);
        if(!(pointtype.hasType())) ImGui::RadioButton("Type", false);
        else ImGui::RadioButton("Type", &listboxColorVal, 5);
        mapColorToValue(0); // 0 is a dummy
        ImGui::TreePop();
      }
      // Color Map
      ImGui::Separator();
      if (ImGui::TreeNode("Map")) {
        ImGui::RadioButton("Solid", &listboxColorMapVal, 0);
        ImGui::RadioButton("Grey", &listboxColorMapVal, 1);
        ImGui::RadioButton("HSV", &listboxColorMapVal, 2);
        ImGui::RadioButton("Jet", &listboxColorMapVal, 3);
        ImGui::RadioButton("Hot", &listboxColorMapVal, 4);
        ImGui::RadioButton("Rand", &listboxColorMapVal, 5);
        ImGui::RadioButton("SHSV", &listboxColorMapVal, 6);
        ImGui::RadioButton("TEMP", &listboxColorMapVal, 7);
        changeColorMap(0); // dummy 0
        ImGui::TreePop();
      }
      // Color Type
      ImGui::Separator();
      if (ImGui::TreeNode("Type")) {
        ImGui::RadioButton("Use Color Map", &colorScanVal, 0);
        ImGui::RadioButton("By Scan ID", &colorScanVal, 1);
        if (!(pointtype.hasColor())) ImGui::RadioButton("Point Color", false);
        else ImGui::RadioButton("Point Color", &colorScanVal, 2);
        setScansColored(0); //dummy 0
        ImGui::TreePop();
      }
      ImGui::Separator();
      ImGui::Text("Color values:");
      static float small_step = (cm->getMax()-cm->getMin())/100; // 1percent increase
      static float big_step = small_step*10; // 100percent increase
      ImGui::InputFloat("Min", &mincolor_value, small_step, big_step, "%.1f", ImGuiInputTextFlags_CharsDecimal);
      ImGui::InputFloat("Max", &maxcolor_value, small_step, big_step);
      // Unsure if this would be better: 
      // ImGui::SliderFloat("Min", &mincolor_value, cm->getMin(), cm->getMax(), "%.1f"); //, ImGuiSliderFlags_Logarithmic);
      // ImGui::SliderFloat("Max", &maxcolor_value, cm->getMin(), cm->getMax(), "%.1f"); //, ImGuiSliderFlags_Logarithmic);
      
      if (ImGui::Button("Reset Min/Max")) resetMinMax(0);
      minmaxChanged(0); // dummy0
      ImGui::TreePop();

    }

    // Anim panel
    static bool coloranimbool;
    ImGui::Checkbox("Force keep color", &coloranimbool); coloranim = coloranimbool;
    ImGui::Separator();
    ImGui::Text("Anim delay:");
    ImGui::InputInt("##Anim", &anim_delay, 1, 10); 
    if (ImGui::Button("Animate")) startAnimation(0);
    ImGui::Separator();

    // Camera path 
    if(ImGui::TreeNode("Camera Path")) {
      ImGui::Text("Path file:");
      ImGui::InputTextWithHint("##File1", "path.dat", path_file_name, 2048);
      ImGui::Text("Interpolation factor");
      ImGui::SliderFloat("##Interpolationfactor", &path_interp_factor, 0.001f, 10.0f, "%.3f", ImGuiSliderFlags_Logarithmic);
      if (ImGui::Button("Save Path")) {savePath(0);}
      ImGui::SameLine();
      if (ImGui::Button("Load Path")) {loadPath(0);}
      if (ImGui::Button("Load Robot Path")) {drawRobotPath(0);}
      ImGui::Separator();
      static bool save_anim_bool, interpolate_bool;
      ImGui::Checkbox("Save Animation", &save_anim_bool); save_animation=save_anim_bool;
      ImGui::Checkbox("Interpolate by Distance", &interpolate_bool); 
      // Check if updated
      // if ((bool)inter_by_dist != interpolate_bool) callCameraUpdate(0);
      inter_by_dist=interpolate_bool;
      if (ImGui::Button("Animate Path")) pathAnimate(0);
      if (ImGui::Button("Animate Path and Matching")) pathMatchingAnimate(0);
      ImGui::TreePop();
    }
    ImGui::Separator();

    // Screenshots and position config
    if (ImGui::TreeNode("Position & Screenshots")) {
      ImGui::Text("Pose file:");
      ImGui::InputTextWithHint("##PoseFile", "pose.dat", pose_file_name, 2048);
      if (ImGui::Button("Save")) savePose(0);
      ImGui::SameLine();
      if (ImGui::Button("Load")) loadPose(0);
      ImGui::Text("Scaling Factor");
      ImGui::SliderInt("##Factor", &factor, 1, 10);
      if (ImGui::Button("Save Image")) saveImage(0); // screenshot before imgui frame render x)
      ImGui::TreePop();
    }
    ImGui::Separator();
    
    // Selection Panel
    if (ImGui::TreeNode("Selection")) {
      ImGui::Text("Selection file:");
      ImGui::InputTextWithHint("##File2", "selected.3d", selection_file_name, 2048);
      if (ImGui::Button("Save")) saveSelection(0);
      ImGui::SameLine();
      if (ImGui::Button("Load")) loadSelection(0);
      ImGui::SameLine();
      if (ImGui::Button("Clear")) clearSelection(0);
      static bool select_unselect_bool = true, select_voxels_bool = false;
      selection_depth = 50;
      brush_size = 50;
      ImGui::Checkbox("Select/Unselect", &select_unselect_bool); selectOrunselect=select_unselect_bool;
      ImGui::Checkbox("Select Voxels", &select_voxels_bool); select_voxels=select_voxels_bool;
      ImGui::SliderInt("Depth:", &selection_depth, 1, 100, "%d", ImGuiSliderFlags_Logarithmic);
      ImGui::SliderInt("Brushsize:", &brush_size, 0, 100, "%d", ImGuiSliderFlags_Logarithmic);
      ImGui::TreePop();
    }
    ImGui::Separator();

    // Advanced panel
    if (advanced_controls && ImGui::TreeNode("Advanced")) {
      ImGui::SliderInt("Frame #:", &current_frame, 0, MetaMatrix[0].size()-1);
      ImGui::SliderFloat("FPS. Lim:", &idealfps, 0.0f, 240.0f, "%.1f");
      ImGui::SliderFloat("Farplane:", &maxfardistance, 1.0f, 1000000.0f, "%.1f", ImGuiSliderFlags_Logarithmic);
      ImGui::SliderFloat("Nearplane:", &neardistance, 0.01f, 1000000.0f, "%.01f", ImGuiSliderFlags_Logarithmic);
      if(ImGui::Button("Cylce LOD")) cycleLOD();
      ImGui::SliderFloat("LOD speed:", &adaption_rate, 0.0f, 3.0f, "%.1f");
      static bool pathshift_bool;
      ImGui::Checkbox("Shift Path for 3D", &pathshift_bool); path3D=pathshift_bool;
      ImGui::SliderFloat("3D Shift:", &shifted, 0.0f, 50.0f);
      ImGui::TreePop();
    
    // idk why this has not been there earlier
    } else if (!advanced_controls) {
      ImGui::Text("--advanced missing");
      ImGui::SameLine();
      if(ImGui::Button("Enable")) advanced_controls = true;
      ImGui::Separator();
    }

    ImGui::End(); // End of "3D-Viewer Selection" Panel
  }

  // 2. Similar to legacy shows controlls panel
  {
    ImGui::SetNextWindowPos(ImVec2(START_WIDTH_IMGUI * 0.175, START_HEIGHT_IMGUI * 0.75), ImGuiCond_FirstUseEver);
    ImGui::SetNextWindowSize(ImVec2(START_WIDTH_IMGUI * 0.65, START_HEIGHT_IMGUI * 0.20), ImGuiCond_FirstUseEver);
    if (ImGui::Begin("Controls")) {

      //ImGui::Text("Application average %.1f FPS", io.Framerate);
      ImGui::BeginTable("controls_table", 5, ImGuiTableFlags_ScrollX | ImGuiTableFlags_SizingStretchProp);
      
      // Column 1
      ImGui::TableNextColumn();
      ImGui::Text("Mode");
      if(ImGui::Button("Top view")) topView();
      if(ImGui::Button("Rotate view")) rotateView();
      if(ImGui::Button("Reset view")) resetView(0);
      
      // Column 2 
      ImGui::TableNextColumn();
      ImGui::Text("Zoom settings");
      if (showViewMode == 0) {
        ImGui::SliderFloat("Field of View", &cangle, 1.0, 180.0, "%.1f");
        ImGui::Text("Currently in: Normal view.");
      }
      if (showViewMode == 1) {
        ImGui::SliderFloat("Parallel Zoom", &pzoom, 0.1, 100000.0, "%.1f", ImGuiSliderFlags_Logarithmic);
        ImGui::Text("Currently in: Ortographic view.");
      }
      if (showViewMode == 2) {
        ImGui::SliderFloat("Rotate Zoom", &rzoom, 0.001, 100000.0, "%.3f", ImGuiSliderFlags_Logarithmic);
        //ImGui::Text("The navball does not work in rotate mode! ---->");
        ImGui::Text("Currently in: Rotate view.");
      } 
      if (ImGui::TreeNode("Keyboard bindings")) {
        ImGui::Text("w/a/s/d: fwd/back left/right");
        ImGui::Text("q/e: roll\t\tc/y: up/down");
        //ImGui::Text("c/y: up/down");
        ImGui::Text("Hold Shift/Ctrl: faster/slower");
        ImGui::TreePop();
      }
      // Column 3
      // Navball replacement
      ImGui::TableNextColumn();
      ImGui::Text("Navball");
      static vgm::Quat qRotVgm = vgm::Quat(1.f,0.f,0.f,0.f);
      static double t[3] = {0,0,0}, mat[16], rPT[3];
      if (ImGui::gizmo3D("##gizmo1", qRotVgm)) {
        if (showViewMode != 2) {
          // Convert gizmos vgm crap to 3dtk
          quat[0] = qRotVgm.w; quat[1] = qRotVgm.x; quat[2] = qRotVgm.y; quat[3] = qRotVgm.z;
          QuatToMatrix4((const double *)quat, t, mat);
          Matrix4ToEuler(mat, rPT);
          // rotate the camera
          mouseRotX = deg(rPT[0]);
          mouseRotY = deg(rPT[1]);
          mouseRotZ = deg(rPT[2]);
        } 
      } else {
        rPT[0] = rad(mouseRotX);
        rPT[1] = rad(mouseRotY);
        rPT[2] = rad(mouseRotZ);
        EulerToMatrix4(t, rPT, mat);
        // Convert back to vgm, which gets synched next rendering cycle
        if (showViewMode != 2) {
          Matrix4ToQuat(mat, quat);
          qRotVgm.w = quat[0]; 
          qRotVgm.x = quat[1]; 
          qRotVgm.y = quat[2]; 
          qRotVgm.z = quat[3];
        }
      }
      
      // Column 4
      ImGui::TableNextColumn();
      ImGui::Text("Camera");
      static int signed_int_cam_choice;
      ImGui::SliderInt("Choose camera", &signed_int_cam_choice, 0, cams.size()); 
      cam_choice = signed_int_cam_choice;
      if(ImGui::Button("Add camera")) callAddCamera(0);
      if(ImGui::Button("Delete camera")) callDeleteCamera(0);

      // Column 5
      ImGui::TableNextColumn();
      ImGui::Text("General behaviour");
      static bool cam_mouse_nav_bool = true;
      ImGui::Checkbox("MouseNav", &cam_mouse_nav_bool); 
      cameraNavMouseMode = cam_mouse_nav_bool;
      static bool always_all_pts = false, always_reduce_pts = false;
      bool always_tmp = always_all_pts, reduce_tmp = always_reduce_pts;
      ImGui::Checkbox("Always all points", &always_all_pts);
      ImGui::Checkbox("Always reduce points", &always_reduce_pts);
      // Check if changed all pts
      if (always_tmp != always_all_pts && always_all_pts) {
        always_reduce_pts = false;
        reduce_tmp = false;
        changePointMode(0);
      } // Check if changed reduced pts
      else if (reduce_tmp != always_reduce_pts && always_reduce_pts) {
        always_all_pts = false;
        always_tmp = false;
        changePointMode(1); 
      }
      ImGui::EndTable();
    } 
    ImGui::End(); // End of "3D Viewer - Controlls panel"
  }

  // Render IMGUI stuff into ImGuis internal buffer
  ImGui::Render();

  // GLUT / OpenGL2 camera and aspect handling:
  if (((fabs(cangle_old - cangle) > 0.5)) ||
    (fabs(pzoom_old - pzoom) > 0.5)) {
    cangle_old = cangle;
    pzoom_old = pzoom;
    int viewport[4];
    glGetIntegerv(GL_VIEWPORT, viewport);
    reshapeIm(viewport[2], viewport[3]);
#ifdef _WIN32
    Sleep(5); // legacy show: Sleep(25)
#else
    usleep(50000); // legacy show: usleep(250000)
#endif
  }

  // After reshaping, clear everything on screen
  // glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
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
  SCREEN_WIDTH = glutGet(GLUT_SCREEN_WIDTH);  // This works well, when there is only one monitor. 
  SCREEN_HEIGHT = glutGet(GLUT_SCREEN_HEIGHT); // For multimonitor this needs more treatment...
  START_WIDTH_IMGUI = 1700;//= (0.8*SCREEN_WIDTH);
  START_HEIGHT_IMGUI = 850;//= (0.8*SCREEN_HEIGHT);
  // Check if screen is large enough for HD...
  if (START_WIDTH_IMGUI > SCREEN_WIDTH || START_HEIGHT_IMGUI > SCREEN_HEIGHT) {
    // Use original 3dtk
    START_WIDTH_IMGUI = START_WIDTH;
    START_HEIGHT_IMGUI = START_HEIGHT;
  }
  BOOST_ASSERT_MSG(!(START_WIDTH_IMGUI > SCREEN_WIDTH || START_HEIGHT_IMGUI > SCREEN_HEIGHT), "Go get a wider screen.");
  START_X_IMGUI = (SCREEN_WIDTH - START_WIDTH_IMGUI) / 2;
  START_Y_IMGUI = (SCREEN_HEIGHT - START_HEIGHT_IMGUI) / 2;

  // define the window position and size
  glutInitWindowPosition(START_X_IMGUI, START_Y_IMGUI);
  glutInitWindowSize( START_WIDTH_IMGUI, START_HEIGHT_IMGUI );

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
#ifdef __FREEGLUT_EXT_H__
  glutMouseWheelFunc(ImGui_ImplGLUT_MouseWheelFunc);
#endif
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
