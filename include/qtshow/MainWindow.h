#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include <QMainWindow>

#include "show/program_options.h"

#include "ui_MainWindow.h"

class MainWindow : public QMainWindow, private Ui::MainWindow {
  Q_OBJECT

public:
  MainWindow(const window_settings& ws, QWidget *parent = Q_NULLPTR, Qt::WindowFlags flags = Qt::WindowFlags());

signals:
  void scanDirectoryOpened(QString dir, QString format, int start, int end);

public slots:
  void openScanDirectory();
  void scanPicked(QString dir, QString format, int start, int end);
};

#endif
