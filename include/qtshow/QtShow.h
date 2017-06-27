#ifndef __QTSHOW_H__
#define __QTSHOW_H__

#include <QApplication>

#include "qtshow/MainWindow.h"

class QtShow : public QApplication {
  Q_OBJECT

public:
  QtShow(int &argc, char **argv);

public slots:
  void loadDifferentScan(dataset_settings ds);

protected:
  MainWindow *mainWindow;
  dataset_settings ds;
  window_settings ws;
};

#endif
