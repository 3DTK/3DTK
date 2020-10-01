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
#include <thread>
#ifdef __linux__
	#include <linux/input.h>
#endif
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
#ifndef __APPLE__
    #include <linux/input.h>
    #include <spnav.h>

int spacenavHandler(){
    int fixRoation = 0;
    int fixTranslation = 0;
    float translationMultiplier = 1;
    float rotationMultipler = 0.5;
#ifdef SPACEMOUSE
    spnav_event sev;
    if(spnav_open()==-1) {
        fprintf(stderr, "failed to connect to the space navigator daemon\n");
        return 1;
    }
    spnav_sensitivity(0.03);
    while(true){
        int event = spnav_poll_event(&sev);
        if(event != 0){
            if(sev.type == SPNAV_EVENT_MOTION) {
                if(fixRoation){
                    moveCamera(-sev.motion.x*translationMultiplier, -sev.motion.y*translationMultiplier, sev.motion.z*translationMultiplier, 0, 0, 0);
                }
                else if(fixTranslation){
                    moveCamera(0, 0, 0, -0.5*sev.motion.rx*rotationMultipler, 0.5*sev.motion.ry*rotationMultipler, -0.5*sev.motion.rz*rotationMultipler);
                }else
                    moveCamera(-sev.motion.x*translationMultiplier, -sev.motion.y*translationMultiplier, sev.motion.z*translationMultiplier, -rotationMultipler*sev.motion.rx, rotationMultipler*sev.motion.ry, -rotationMultipler*sev.motion.rz);
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
#endif

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
#if  defined(SPACEMOUSE) && !defined(__APPLE__)
    std::thread t1(spacenavHandler);
#endif

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
