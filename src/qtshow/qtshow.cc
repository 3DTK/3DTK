#include <iostream>

#include <QApplication>

#include "qtshow/GLWidget.h"

#include "../show/show_common.cc"

int main(int argc, char *argv[]){
  QApplication app(argc, argv);
  initShow(argc, argv);
  GLWidget widget;
  widget.show();
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
