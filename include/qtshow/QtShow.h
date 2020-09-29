#ifndef __QTSHOW_H__
#define __QTSHOW_H__

#include <QApplication>

#include "qtshow/MainWindow.h"
#ifdef SPACEMOUSE
    #include "qtshow/spnav_controller.h"
#endif

class QtShow : public QApplication {
  Q_OBJECT

public:
  QtShow(int &argc, char **argv);
  ~QtShow() override;

public slots:
  void loadDifferentScan(dataset_settings dss);

protected:
  MainWindow *mainWindow;
  dataset_settings dss;
  window_settings ws;
  display_settings ds;
#ifdef SPACEMOUSE
  SpaceNavController *spnav_controller;
#endif
};

#endif
