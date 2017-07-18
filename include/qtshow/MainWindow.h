#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include "ui_MainWindow.h"

#include <string>
#include <vector>

#include <QProgressBar>

#include "show/program_options.h"


class MainWindow : public QMainWindow, protected Ui::MainWindow {
  Q_OBJECT

public:
  MainWindow(const dataset_settings& ds, const window_settings& ws, QWidget *parent = Q_NULLPTR, Qt::WindowFlags flags = Qt::WindowFlags());

signals:
  void scanDirectoryOpened(dataset_settings);
  void pathFilePicked(QString);

public slots:
  void openScanDirectory();
  void openRecentDirectory();
  void clearRecentDirectories();

protected:
  std::vector<std::string> loadRecentDirectories();
  void updateRecentDirectoriesMenu(std::vector<std::string> directories);

  dataset_settings ds;
  QProgressBar *progressbar;

protected slots:
  void addRecentDirectory();

friend class QtShow;
};

#endif
