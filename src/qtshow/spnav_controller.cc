#include <qtshow/spnav_controller.h>
#include <spnav.h>
#include <iostream>

bool SpaceNavController::initialize() {
  if(spnav_open() == -1) {
    std::cout << "Could not connect to Spacenav daemon. Continue without Spacenav\n";
    return false;
  } else {
    std::cout << "Connected to Spacenav daemon.\n";
    return true;
  }
}

void SpaceNavController::run() {
  spnav_event ev;

  double stepsize = 1;
  bool control  = false;
  bool alt      = false;
  bool shift    = false;
  bool fixRotation = false;
  bool fixTranslation = false;

  std::cout << "waiting for space mouse input\n";

    while(spnav_wait_event(&ev)) {
        if(ev.type == SPNAV_EVENT_MOTION) {
            /*if (shift) stepsize  = 10.0;
            else if (control) stepsize = 0.1;
            else if (alt) stepsize     = 100.0;
            else stepsize = 1;

            double rotsize = 0.2 *stepsize;*/
            if(!fixRotation && !fixTranslation) {
                m_glwidget->spaceNavEvent(-ev.motion.x**translationMultiplier,
                                          -ev.motion.y**translationMultiplier,
                                          ev.motion.z**translationMultiplier,
                                          -ev.motion.rx**rotationMultiplier,
                                          ev.motion.ry**rotationMultiplier,
                                          -ev.motion.rz**rotationMultiplier
                );
            } else if (fixRotation && !fixTranslation){
                m_glwidget->spaceNavEvent(-ev.motion.x**translationMultiplier,
                                          -ev.motion.y**translationMultiplier,
                                          ev.motion.z**translationMultiplier,
                                          0,0,0
                );
            } else if (fixTranslation && !fixRotation){
                m_glwidget->spaceNavEvent(
                        0,0,0,
                        -ev.motion.rx**rotationMultiplier,
                        ev.motion.ry**rotationMultiplier,
                        -ev.motion.rz**rotationMultiplier
                );
            }
        } else if(ev.type == SPNAV_EVENT_BUTTON) {
            if(ev.button.press) {
                std::cout << ev.button.bnum << "\n";
                switch(ev.button.bnum) {
                    case 0:
                        fixRotation = !fixRotation;
                        fixTranslation = false;
                        break;
                    case 1:
                        fixTranslation = !fixTranslation;
                        fixRotation = false;
                        break;
                    case 12:
                        // 12 -> 1, 13 -> 2, 14 -> 3, 15 -> 4
                        break;
                    case 23:
                        // alt
                        break;
                    case 24:
                        // shift
                        break;
                    case 25:
                        // control
                        break;
                    case 26:
                        // rotation stop
                        fixRotation = !fixRotation;
                        break;
                    default:
                        break;
                }
      } else {
            control  = false;
            alt      = false;
            shift    = false;
            stepsize = 1;
      }
    }
  }
  stop();
}

void SpaceNavController::stop() {
  if(m_mouse_found) {
    m_running = false;
    spnav_close();
    std::cout << "Unconnected from spacenav daemon\n";
  }
}
