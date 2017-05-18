#include <iostream>

#include <QApplication>

#include "qtshow/MainWindow.h"

#include "show/show_common.h"

int main(int argc, char *argv[]){
  QApplication app(argc, argv);
  initShow(argc, argv);
  MainWindow mainWindow;
  mainWindow.show();
  return app.exec();
}
void updateCamControls() {}
void resetRotationButton() {}
void updateTopViewControls() {}
void updateControls() {}
void interruptDrawing() {}
void checkForInterrupt() {}
bool isInterrupted() {return false;}
void updatePointModeControls() {}
