#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include <QMainWindow>

#include "ui_MainWindow.h"

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QWidget *parent = 0);

public slots:
  void pickScanDirectory();

private:
  Ui::MainWindow ui;
};

#endif
