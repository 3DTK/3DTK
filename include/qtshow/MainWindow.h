#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include "ui_MainWindow.h"

#include <string>
#include <vector>

#include <QProgressBar>

#include "qtshow/SettingsDialog.h"
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

  void showSettingsDialog();

  void addCamCombobox(int index);
  void deleteCamCombobox(int index);

  void hideDockWidgets();
  void showDockWidgets();

  void setStateFilePath();
  void saveStates();
  void loadStates();

protected:
  std::vector<std::string> loadRecentDirectories();
  void updateRecentDirectoriesMenu(std::vector<std::string> directories);

  SettingsDialog *settingsDialog = Q_NULLPTR;
  bool hideWidgetsInFullscreen = true;

  dataset_settings ds;
  QProgressBar *progressbar;

  QMargins defaultMargins;

protected slots:
  void addRecentDirectory();

  void applySettings();

friend class QtShow;
};

#endif
