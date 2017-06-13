#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include <QMainWindow>

#include "show/program_options.h"

#include "ui_MainWindow.h"

class MainWindow : public QMainWindow, private Ui::MainWindow {
  Q_OBJECT

public:
  MainWindow(const window_settings& ws, QWidget *parent = 0);

public slots:
  void pickScanDirectory();
};

#endif
