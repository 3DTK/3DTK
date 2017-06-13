#ifndef __QTSHOW_H__
#define __QTSHOW_H__

#include <QApplication>

#include "qtshow/MainWindow.h"

class QtShow : public QApplication {
public:
  QtShow(int &argc, char **argv);

protected:
  MainWindow *mainWindow;
};

#endif
