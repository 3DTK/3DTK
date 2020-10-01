#ifndef __SPNAV_CONTROLLER_H__
#define __SPNAV_CONTROLLER_H__

#include <qtshow/GLWidget.h>
#include <spnav.h>
#include <thread>

class  SpaceNavController {
public:
  SpaceNavController(GLWidget *glwidget, float *translationMultiplier, float *rotationMultiplier): m_thread(nullptr), m_glwidget(glwidget) {
    this->translationMultiplier = translationMultiplier;
    this->rotationMultiplier = rotationMultiplier;
    m_mouse_found = initialize();
    if(m_mouse_found && m_glwidget) {
      m_thread.reset(new std::thread(&SpaceNavController::run, this));
    }
  }

  ~SpaceNavController(){
    stop();
    if(m_thread) {
      if(m_thread->joinable()) {
        m_thread->join();
      }
    }
  }

  void run();
  void stop();

private:
  std::unique_ptr<std::thread> m_thread;
  GLWidget* m_glwidget;

  float *translationMultiplier;
  float *rotationMultiplier;
  bool m_mouse_found = false;
  bool m_running = false;
  bool initialize();
};

#endif
