#include "qtshow/QtShow.h"
#include "show/show_common.h"

int main(int argc, char *argv[]){
  exitFunc = QtShow::exit;
  setSignalHandling();

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
void updateViewModeControls() {}
