#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include "ui_MainWindow.h"

#include "show/program_options.h"


class MainWindow : public QMainWindow, private Ui::MainWindow {
  Q_OBJECT

public:
  MainWindow(const dataset_settings& ds, const window_settings& ws, QWidget *parent = Q_NULLPTR, Qt::WindowFlags flags = Qt::WindowFlags());

signals:
  void scanDirectoryOpened(dataset_settings);

public slots:
  void openScanDirectory();

protected:
  dataset_settings ds;
};

#endif
