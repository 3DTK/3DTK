#include "qtshow/QtShow.h"

int main(int argc, char *argv[]){
  QtShow app(argc, argv);
  return app.exec();
}

// Legacy callbacks
void updateCamControls() {}
void resetRotationButton() {}
void updateTopViewControls() {}
void updateControls() {}
void interruptDrawing() {}
void checkForInterrupt() {}
bool isInterrupted() {return false;}
void updatePointModeControls() {}
