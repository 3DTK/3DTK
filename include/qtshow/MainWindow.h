#ifndef __MAINWINDOW_H__
#define __MAINWINDOW_H__

#include "ui_MainWindow.h"

#include <string>
#include <vector>

#include <QProgressBar>

#include "qtshow/SettingsDialog.h"
#include "qtshow/ShortcutsDialog.h"
#include "show/program_options.h"


class MainWindow : public QMainWindow, protected Ui::MainWindow {
  Q_OBJECT

public:
  MainWindow(const dataset_settings& dss, const window_settings& ws, const display_settings& ds, QWidget *parent = Q_NULLPTR, Qt::WindowFlags flags = Qt::WindowFlags());

signals:
  void scanDirectoryOpened(dataset_settings);
  void pathFilePicked(QString);

public slots:
  void openScanDirectory();
  void openRecentDirectory();
  void clearRecentDirectories();

  void showSettingsDialog();
  void showShortcutsDialog();

  void addCamCombobox(int index);
  void deleteCamCombobox(int index);

  void hideDockWidgets();
  void showDockWidgets();

  void setStateFilePath();
  void saveStates();
  void loadStates();

  void setSelectionFilePath();
  void saveSelectedPoints();
  void loadSelectedPoints();
  void setSelectionDepth(int value);
  void setSelectionBrushSize(int value);

  void setCurrentFrame(int value);
  void setFps(double value);
  void setFarDistance(double value);
  void setNearDistance(double value);
  void callCycleLOD();
  void setLodSpeed(double value);
  void setPath3d(bool checked);
  void set3dShift(double value);
  void setStartScanIndex(int value);
  void setEndScanIndex(int value);
  void callStepScansUp();
  void callStepScansDown();
  void callReloadFrames();

protected:
  std::vector<std::string> loadRecentDirectories();
  void updateRecentDirectoriesMenu(std::vector<std::string> directories);

  SettingsDialog *settingsDialog = Q_NULLPTR;

  ShortcutsDialog *shortcutsDialog = Q_NULLPTR;

  dataset_settings dss;
  display_settings ds;
  QProgressBar *progressbar;

  QMargins defaultMargins;

protected slots:
  void addRecentDirectory();

  void applySettings();

friend class QtShow;
};

#endif
