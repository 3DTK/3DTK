#include <algorithm>
#include <fstream>

#include <QFileDialog>
#include <QTextStream>

#include "qtshow/MainWindow.h"
#include "qtshow/ScanPicker.h"
#include "show/show_common.h"

#include "show/program_options.h"

MainWindow::MainWindow(const dataset_settings& dss, const window_settings& ws, const display_settings& ds, QWidget *parent, Qt::WindowFlags flags)
  : QMainWindow(parent, flags)
  , dss(dss), ds(ds)
{
  if (!ws.nogui) {
    setupUi(this);

    // Setup status bar
    progressbar = new QProgressBar(statusbar);
    progressbar->hide();
    statusbar->addPermanentWidget(progressbar);

    // Setup recent directories menu
    updateRecentDirectoriesMenu(loadRecentDirectories());
    connect(this, &MainWindow::scanDirectoryOpened, this, &MainWindow::addRecentDirectory);

    // Setup view mode radio group
    buttonGroupViewMode->setId(radioButtonDefaultView, 0);
    buttonGroupViewMode->setId(radioButtonTopView, 1);
    buttonGroupViewMode->setId(radioButtonRotateView, 2);

    // Initialize widgets from arguments
    comboBoxFogType->setCurrentIndex(ds.fog.type % 4); // modulo to deal with inverted fog options
    checkBoxInvertFog->setChecked(ds.fog.type >= 4);
    doubleSpinBoxFogDensity->setValue(ds.fog.density * scale);

    // Initialize widgets from globals
    // They can't be initialized from arguments
    // because initShow sets them.
    doubleSpinBoxMinColorValue->setValue(mincolor_value);
    doubleSpinBoxMaxColorValue->setValue(maxcolor_value);

    // Create tabbed dock
    tabifyDockWidget(dockMode, dockNavigation);
    tabifyDockWidget(dockState, dockSelection);

    // Setup navigation mode cursor and direction of movement
    moveXPushButton->setType(0);
    moveYPushButton->setType(1);
    moveZPushButton->setType(2);

    // Set initial state file path
    QDir curDir(QDir::current());
    lineEditStateFile->setText(curDir.canonicalPath() + "/" + QString::fromStdString(dss.data_source) + "config.ini");
    lineEditSelectionFile->setText(curDir.canonicalPath() + "/selected.3d");
  } else {
    glWidget = new GLWidget(this);
    glWidget->setFocusPolicy(Qt::ClickFocus);
    setCentralWidget(glWidget);
    resize(ws.dimensions.w, ws.dimensions.h);
  }
}

void MainWindow::updateRecentDirectoriesMenu(std::vector<std::string> directories) {
  menuOpenRecent->clear();
  for (auto directory : loadRecentDirectories()) {
    QAction *action = menuOpenRecent->addAction(
      QString::fromStdString(directory), this,
      SLOT(openRecentDirectory()));
      // TODO uncomment this when Andreas upgrades to Qt 5.6
      // &MainWindow::openRecentDirectory);
    action->setData(QString::fromStdString(directory));
  }
  menuOpenRecent->addSeparator();
  menuOpenRecent->addAction(actionClearRecent);
}

void MainWindow::openScanDirectory() {
  ScanPicker sp(dss, this);
  if (sp.exec())
    emit scanDirectoryOpened(dss);
}

void MainWindow::openRecentDirectory() {
  QAction *action = qobject_cast<QAction *>(sender());
  if (action) {
    dss.data_source = action->data().toString().toStdString();
    emit scanDirectoryOpened(dss);
  }
}

std::string recent_directory_path() {
  std::string config_home;
#ifndef _MSC_VER
  // in $XDG_CONFIG_HOME/3dtk/qtshow-recent.txt
  char *home_c = getenv("HOME");
  char *config_home_c = getenv("XDG_CONFIG_HOME");
  if (config_home_c && *config_home_c != '\0') {
    config_home = config_home_c;
  } else {
    config_home = std::string(home_c) + "/.config";
  }
#else
  // in %APPDATA%/3dtk/qtshow-recent.txt
  config_home = std::string(getenv("APPDATA"));
#endif

  return (config_home + "/3dtk/qtshow-recent.txt").c_str();
}

void MainWindow::addRecentDirectory() {
  std::vector<std::string> directories = loadRecentDirectories();

  auto duplicate = std::find(directories.begin(), directories.end(), dss.data_source);
  if (duplicate != directories.end()) {
    directories.erase(duplicate);
  }
  directories.insert(directories.begin(), dss.data_source);

  std::ofstream directories_file(recent_directory_path());

  // Only remember 10 most recent directories
  for (auto directoryIt = directories.begin();
       directoryIt != directories.end()
         && directoryIt != directories.begin() + 10;
       directoryIt++) {
    directories_file << *directoryIt << std::endl;
  }

  updateRecentDirectoriesMenu(directories);
}

std::vector<std::string> MainWindow::loadRecentDirectories() {
  std::vector<std::string> result;

  // Read recent directories file
  std::ifstream directories_file(recent_directory_path());

  std::string line;
  while (std::getline(directories_file, line)) {
    if (!line.empty()) {
      result.push_back(line);
    }
  }

  directories_file.close();

  return result;
}

void MainWindow::clearRecentDirectories() {
  std::ofstream directories_file(recent_directory_path());
  directories_file.close();
  updateRecentDirectoriesMenu({});
}

void MainWindow::showSettingsDialog() {
  // Create new window if it doesn't exist yet
  if (!settingsDialog) {
    settingsDialog = new SettingsDialog(this);
    connect(settingsDialog, &SettingsDialog::accepted, this, &MainWindow::applySettings);
  }

  // Read settings from UI state
  settingsDialog->cbCaptureCursor->setChecked(captureMouseCursor);
  settingsDialog->cbInvertMouseX->setChecked(invertMouseX);
  settingsDialog->cbInvertMouseY->setChecked(invertMouseY);
  settingsDialog->cbHideWidgetsInFullscreen->setChecked(hideWidgetsInFullscreen);
  settingsDialog->cbEnableAdvancedMode->setChecked(advanced_controls);

  // Show dialog
  settingsDialog->show();
}

void MainWindow::showShortcutsDialog() {
  if (!shortcutsDialog) {
    shortcutsDialog = new ShortcutsDialog(this);
  }

  shortcutsDialog->show();
}

void MainWindow::applySettings() {
  captureMouseCursor = settingsDialog->cbCaptureCursor->isChecked();
  invertMouseX = settingsDialog->cbInvertMouseX->isChecked();
  invertMouseY = settingsDialog->cbInvertMouseY->isChecked();
  hideWidgetsInFullscreen = settingsDialog->cbHideWidgetsInFullscreen->isChecked();
  advanced_controls = settingsDialog->cbEnableAdvancedMode->isChecked();
  if(advanced_controls) {
    dockAdvanced->setFloating(true);
    dockAdvanced->show();
  } else {
    dockAdvanced->hide();
  }

  std::string configHome = getConfigHome();
  QFile file(QString::fromStdString(configHome + "/3dtk/show.ini"));
  if(!file.open(QIODevice::WriteOnly)) {
    return;
  }

  QTextStream out(&file);

  out << "advanced=" << (advanced_controls ? "true" : "false") << endl;
  out << "invertmousex=" << (invertMouseX ? "true" : "false") << endl;
  out << "invertmousey=" << (invertMouseY ? "true" : "false") << endl;
  out << "capturemouse=" << (captureMouseCursor ? "true" : "false") << endl;
  out << "hidewidgets=" << (hideWidgetsInFullscreen ? "true" : "false") << endl;
}

void MainWindow::addCamCombobox(int index) {
  QString s = QString("#%1").arg(index);
  comboBoxCam->addItem(s);
  comboBoxCam->setCurrentIndex(index - 1);
}

void MainWindow::deleteCamCombobox(int index) {
  comboBoxCam->removeItem(index);
}

void MainWindow::hideDockWidgets()
{
  if (hideWidgetsInFullscreen) {
    defaultMargins = centralwidget->layout()->contentsMargins();
    dockAnimation->hide();
    dockCamera->hide();
    dockState->hide();
    dockSelection->hide();
    dockSnapshot->hide();
    dockObjects->hide();
    dockFog->hide();
    dockColor->hide();
    dockMode->hide();
    dockNavigation->hide();
    if(advanced_controls)
      dockAdvanced->hide();
    statusbar->hide();
    menubar->hide();
    centralwidget->layout()->setContentsMargins(0, 0, 0, 0);
  }
}

void MainWindow::showDockWidgets()
{
  if (hideWidgetsInFullscreen) {
    dockAnimation->show();
    dockCamera->show();
    dockState->show();
    dockSelection->show();
    dockSnapshot->show();
    dockObjects->show();
    dockFog->show();
    dockColor->show();
    dockMode->show();
    dockNavigation->show();
    if(advanced_controls)
      dockAdvanced->show();
    statusbar->show();
    menubar->show();
    centralwidget->layout()->setContentsMargins(defaultMargins);
  }
}

void MainWindow::setStateFilePath()
{
  QString stateFilePath = QFileDialog::getOpenFileName();
  if(!stateFilePath.isEmpty()) {
    lineEditStateFile->setText(stateFilePath);
  }
}

void MainWindow::saveStates()
{
  QFile file(lineEditStateFile->text());
  QFileInfo checkFile(file);
  if(!file.open(QIODevice::WriteOnly) || (checkFile.exists() && !checkFile.isFile())) {
    return;
  }

  QTextStream out(&file);
  out << "advanced=true" << endl;
  out << "fov=" << doubleSpinBoxZoom->text().toUtf8().constData() << endl;
  out << "viewmode=" << buttonGroupViewMode->checkedId() << endl;
  out << "no-points=" << (checkDrawPoints->checkState() != Qt::Checked ? "true" : "false") << endl;
  out << "no-cameras=" << (checkDrawCameras->checkState() != Qt::Checked ? "true" : "false") << endl;
  out << "no-path=" << (checkDrawPath->checkState() != Qt::Checked ? "true" : "false") << endl;
  out << "no-poses=" << (checkDrawPoses->checkState() != Qt::Checked ? "true" : "false") << endl;
  out << "fog-type=" << comboBoxFogType->currentIndex() << endl;
  out << "fog-density=" << doubleSpinBoxFogDensity->text().toUtf8().constData() << endl;
  out << "position=" << labelCameraPosition->text().toUtf8().constData() << endl;
  out << "rotation=" << labelCameraQuaternion->text().toUtf8().constData() << endl;
  out << "pointsize=" << spinBoxPointSize->cleanText().toUtf8().constData() << endl;
  out << "colormap=" << comboBoxColorMap->currentText().toLower().toUtf8().constData() << endl;
  out << "scanscolored=" << comboBoxColorType->currentIndex() << endl;
  out << "max=" << dss.distance_filter.max << endl;
  out << "min=" << dss.distance_filter.min << endl;
}

void MainWindow::loadStates()
{
  using namespace boost::program_options;

  bool advanced, noPoints, noCameras, noPath, noPoses, noFog;
  float fov, fogDensity;
  int viewmode, fogType, pointsize;
  ShowColormap colormap;
  int scansColored;
  Position position;
  Quaternion rotation;

  bool nogui, invertMouseX, invertMouseY, color, noAnimColor, captureMouse, hideWidgets;
  float fps, colormin, colormax;
  double scale, reduce;
  double distMin, distMax;
  int octree, stepsize;
  Color bgcolor;
  WindowDimensions dimensions;

  options_description gui_options("GUI options");
  setGUIOptions(nogui, fps, dimensions, advanced,
		invertMouseX, invertMouseY,
		captureMouse, hideWidgets, gui_options);

  options_description display_options("Display options");
  setDisplayOptions(scale, fov, viewmode, noPoints, noCameras, noPath, noPoses,
		    noFog, fogType, fogDensity, position, rotation,
		    pointsize, display_options);

  options_description color_options("Point coloring");
  setColorOptions(bgcolor, color, colormap, colormin, colormax,
		  scansColored, noAnimColor, color_options);

  options_description reduction_options("Point reduction");
  setReductionOptions(distMin, distMax, reduce, octree, stepsize,
		      reduction_options);

  variables_map vm;
  options_description visible_options("");
  visible_options
    .add(gui_options)
    .add(display_options)
    .add(color_options)
    .add(reduction_options)
    ;
  QString filePath = lineEditStateFile->text();
  QFileInfo checkFile(filePath);
  if(checkFile.exists() && !checkFile.isFile()) return;

  std::ifstream config_file(filePath.toLatin1().data());
  if(config_file) {
    store(parse_config_file(config_file, visible_options), vm);
  }
  notify(vm);

  doubleSpinBoxZoom->setValue(fov);

  buttonGroupViewMode->buttonClicked(viewmode);
  buttonGroupViewMode->button(viewmode)->setChecked(true);

  checkDrawPoints->setChecked(!noPoints);
  checkDrawCameras->setChecked(!noCameras);
  checkDrawPath->setChecked(!noPath);
  checkDrawPoses->setChecked(!noPoses);

  comboBoxFogType->setCurrentIndex(fogType);

  doubleSpinBoxFogDensity->setValue(fogDensity);

  X = position.x; Y = position.y; Z = position.z;

  QuatToMouseRot(rotation, mouseRotX, mouseRotY, mouseRotZ);
  glWidget->cameraChanged();

  spinBoxPointSize->setValue(pointsize);

  comboBoxColorMap->setCurrentIndex(static_cast<int>(colormap));
  comboBoxColorType->setCurrentIndex(scansColored);

  glWidget->update();
}

void MainWindow::setSelectionFilePath()
{
  QString selectionFilePath = QFileDialog::getOpenFileName();
  if(!selectionFilePath.isEmpty()) {
    lineEditSelectionFile->setText(selectionFilePath);
  }
}

void MainWindow::saveSelectedPoints()
{
  QString fileName = lineEditSelectionFile->text();
  selection_file_name = fileName.toLatin1().data();

  saveSelection(0);
}

void MainWindow::loadSelectedPoints()
{
  QString fileName = lineEditSelectionFile->text();
  selection_file_name = fileName.toLatin1().data();

  loadSelection(0);
  update();
}

void MainWindow::setSelectionDepth(int value)
{
  selection_depth = value;
}

void MainWindow::setSelectionBrushSize(int value)
{
  brush_size = value;
}

void MainWindow::setCurrentFrame(int value)
{
  current_frame = value;
  update();
}

void MainWindow::setFps(double value)
{
  idealfps = value;
}

void MainWindow::setFarDistance(double value)
{
  maxfardistance = value;
  update();
}

void MainWindow::setNearDistance(double value)
{
  neardistance = value;
  update();
}

void MainWindow::callCycleLOD()
{
  cycleLOD();
}

void MainWindow::setLodSpeed(double value)
{
  adaption_rate = value;
}

void MainWindow::setPath3d(bool checked)
{
  path3D = checked;
}

void MainWindow::set3dShift(double value)
{
  shifted = value;
}

void MainWindow::setStartScanIndex(int value)
{
  startRangeScanIdx = value;
  update();
}

void MainWindow::setEndScanIndex(int value)
{
  endRangeScanIdx = value;
  update();
}

void MainWindow::callStepScansUp()
{
  stepScansUp(0);
  spinBoxStartIndex->setValue(startRangeScanIdx);
  spinBoxEndIndex->setValue(endRangeScanIdx);
  startRangeScanIdx = spinBoxStartIndex->value();
  endRangeScanIdx = spinBoxEndIndex->value();
}

void MainWindow::callStepScansDown()
{
  stepScansDown(0);
  spinBoxStartIndex->setValue(startRangeScanIdx);
  spinBoxEndIndex->setValue(endRangeScanIdx);
  startRangeScanIdx = spinBoxStartIndex->value();
  endRangeScanIdx = spinBoxEndIndex->value();
}

void MainWindow::callReloadFrames()
{
  reloadFrames();
}
