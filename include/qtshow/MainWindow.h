#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include <QMainWindow>

#include "ui_MainWindow.h"

class MainWindow : public QMainWindow {
public:
  MainWindow(QWidget *parent = 0);

private:
  Ui::MainWindow ui;
};

#endif
