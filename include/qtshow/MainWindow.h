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
  MainWindow(const dataset_settings& dss, const window_settings& ws, const display_settings& ds, float *translationMultiplier = nullptr, float *rotationMultiplier= nullptr, QWidget *parent = Q_NULLPTR, Qt::WindowFlags flags = Qt::WindowFlags());

signals:
  void scanDirectoryOpened(dataset_settings);
  void pathFilePicked(QString);

public slots:
  void slider3DMouseReleased(int value);
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

  void setCylinderParasGUI();
  void setCylinderFilePath();
  void setCurrentCylinder(int value);
  void testCylinderParasGUIScale();
  void changeCylinderFittingParas();
  void changeCylinderParas();
  void createCylinderFromSelection();
  void setAutoCorrect(bool value);
  void deleteCurrentCylinder();
  void saveCylinderToFile();
  void loadCylinderFromFile();
  void clearCylinder();

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
private slots:
void on_horizontalSlider3DMouse_sliderReleased();

private:
    float *translationMultiplier;
    float *rotationMultiplier;
};

#endif
